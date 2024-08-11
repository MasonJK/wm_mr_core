#include <functional>
#include <memory>

#include "wm_mr_core/core.hpp"

float calculate_distance(const UTMPose& pose1, const UTMPose& pose2) {
    return std::sqrt(std::pow(pose1.x - pose2.x, 2) + std::pow(pose1.y - pose2.y, 2));
}

float calculate_progress(const std::vector<UTMPose>& waypoints, float current_x, float current_y){
  if (waypoints.empty()) return 0.0;

  UTMPose current_pose;
  current_pose.x = current_x;
  current_pose.y = current_y;

  float total_distance = 0.0;
  float distance_covered = 0.0;

  // Calculate the total distance of the mission
  for (size_t i = 1; i < waypoints.size(); ++i) {
      total_distance += calculate_distance(waypoints[i-1], waypoints[i]);
  }

  // Find the closest waypoint and calculate distance covered
  float min_distance = std::numeric_limits<float>::max();
  size_t closest_index = 0;

  for (size_t i = 0; i < waypoints.size(); ++i) {
      float distance = calculate_distance(current_pose, waypoints[i]);
      if (distance < min_distance) {
          min_distance = distance;
          closest_index = i;
      }
  }

  // Calculate distance covered up to the closest waypoint
  for (size_t i = 1; i <= closest_index; ++i) {
      distance_covered += calculate_distance(waypoints[i-1], waypoints[i]);
  }

  // Add the distance from the closest waypoint to the robot's current position
  if (closest_index < waypoints.size() - 1) {
      distance_covered += calculate_distance(waypoints[closest_index], current_pose);
  }

  // Return the progress as a ratio of distance covered to total distance
  return (total_distance > 0) ? distance_covered / total_distance : 0.0;
}

MultiRobotCore::MultiRobotCore(const rclcpp::NodeOptions & node_options)
: Node("multi_robot_core_node", node_options)
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();
  this->declare_parameter("disconnection_threshold", 10);
  disconnection_threshold_ = this->get_parameter("disconnection_threshold").get_value<float>();

  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // topic pub, sub
  log_pub_ = this->create_publisher<mr_msgs::msg::Log>("log_topic", 10);
  fleet_robot_pose_sub_ = this->create_subscription<FleetRobotPose>(
    "fleet_robot_pose",
    QOS_RKL10V,
    std::bind(&MultiRobotCore::fleet_robot_pose_callback, this, _1));

  // service server, client
  single_goal_mission_server_ = this->create_service<SingleGoalMission>("single_goal_mission",
    std::bind(&MultiRobotCore::assign_single_goal_mission, this, _1, _2));
  gpp_client_ = this->create_client<PlanGlobalPath>("global_path_plan");

  timer_ = this->create_wall_timer(
    std::chrono::seconds(10), std::bind(&MultiRobotCore::check_disconnected_robots, this)
  );
}

MultiRobotCore::~MultiRobotCore()
{
}

void MultiRobotCore::publish_log(const int log_type, const std::string &log_content) const {
  auto log_msg = std::make_shared<mr_msgs::msg::Log>();
  log_msg->stamp = this->now();
  log_msg->content = log_content;
  log_msg->log_type = log_type;
  switch (log_type) {
    case mr_msgs::msg::Log::DEBUG:
      RCLCPP_DEBUG(this->get_logger(), log_content.c_str());
      break;
    case mr_msgs::msg::Log::INFO:
      RCLCPP_INFO(this->get_logger(), log_content.c_str());
      break;
    case mr_msgs::msg::Log::WARN:
      RCLCPP_WARN(this->get_logger(), log_content.c_str());
      break;
    case mr_msgs::msg::Log::ERROR:
      RCLCPP_ERROR(this->get_logger(), log_content.c_str());
      break;
    default:
      RCLCPP_INFO(this->get_logger(), log_content.c_str());
      break;
  }

  // Publish the log message to a topic
  log_pub_->publish(*log_msg);
}

void MultiRobotCore::fleet_robot_pose_callback(const mr_msgs::msg::FleetRobotPose::SharedPtr msg)
{
  // update robot poses
  // check if the robot is on a mission
  // if on a mission, estimate time left, and calculate progress

  const std::string& fleet_id = msg->fleet_id;
  if (robot_fleets_.find(fleet_id) == robot_fleets_.end()) {
      robot_fleets_[fleet_id] = std::map<std::string, Robot>();
  }

  for (const auto& robot_pose : msg->fleet_pose) {
      const std::string& robot_id = robot_pose.robot_id;
      if (robot_fleets_[fleet_id].find(robot_id) == robot_fleets_[fleet_id].end()) {
          Robot new_robot;
          new_robot.current_pose = robot_pose;
          new_robot.last_connected = msg->stamp;
          new_robot.state = Robot::State::IDLE;

          robot_fleets_[fleet_id][robot_id] = new_robot;
      } else {
          robot_fleets_[fleet_id][robot_id].current_pose = robot_pose;
          robot_fleets_[fleet_id][robot_id].last_connected = msg->stamp;

          if(robot_fleets_[fleet_id][robot_id].state == Robot::State::DISCONNECTED)
          {
            robot_fleets_[fleet_id][robot_id].state = Robot::State::IDLE;
          } else if(robot_fleets_[fleet_id][robot_id].state == Robot::State::ON_MISSION)
          {
            if (missions_.find(robot_fleets_[fleet_id][robot_id].mission_id) != missions_.end()) {
              Mission& mission = missions_[robot_fleets_[fleet_id][robot_id].mission_id];
              mission.progress = calculate_progress(mission.waypoints, robot_pose.utm_x, robot_pose.utm_y);
            }
          }
      }
  }
}

void MultiRobotCore::check_disconnected_robots()
{
  rclcpp::Time current_time = this->now();

  for (auto& fleet_pair : robot_fleets_) {
    for (auto& robot_pair : fleet_pair.second) {
      Robot& robot = robot_pair.second;

      // Calculate the duration since the last connection
      rclcpp::Duration time_since_last_connection = current_time - robot.last_connected;

      // Check if this duration exceeds the disconnection threshold
      if (time_since_last_connection.seconds() > disconnection_threshold_) {
        if (robot.state != Robot::State::DISCONNECTED) {
          std::string log_content = "Robot [" + robot_pair.first + "] in fleet [" + fleet_pair.first + "] is now DISCONNECTED";
          publish_log(mr_msgs::msg::Log::WARN, log_content);

          robot.state = Robot::State::DISCONNECTED;
        }
      }
    }
  }
}

void MultiRobotCore::assign_single_goal_mission(const std::shared_ptr<SingleGoalMission::Request> request,
  std::shared_ptr<SingleGoalMission::Response> response){
  // check if the robot is selected
  // if not, bid between currently connected robots
  // if robot has been chosen, send to gpp(it will send it to the api)
  // retrieve the navigation poses, and add to mission

  // Check if the robot ID is provided
  if(request->robot_id.empty()){
    // TODO : bid on one of IDLE robots for the mission
    return;
  }
  // Find the robot by ID
  const std::string& robot_id = request->robot_id;
  auto fleet_it = robot_fleets_.find(request->fleet_id);
  if (fleet_it == robot_fleets_.end()) {
    publish_log(mr_msgs::msg::Log::ERROR, "Fleet ID not found.");
    return;
  }

  auto robot_it = fleet_it->second.find(robot_id);
  if (robot_it == fleet_it->second.end()) {
    publish_log(mr_msgs::msg::Log::ERROR, "Robot ID not found in the fleet.");
    return;
  }

  Robot& robot = robot_it->second;

  auto gpp_request = std::make_shared<PlanGlobalPath::Request>();
  gpp_request->start_pose = robot.current_pose;
  gpp_request->goal_pose = request->goal_pose;

  // Send the request to the global path planner service
  if (!gpp_client_->wait_for_service(std::chrono::seconds(5))) {
    publish_log(mr_msgs::msg::Log::ERROR, "Global Path Planner service not available.");
    return;
  }

  auto result_future = gpp_client_->async_send_request(gpp_request);

  // Specify a timeout duration
  std::chrono::seconds timeout(5);

  if (result_future.wait_for(timeout) == std::future_status::ready) {
    try {
      auto result = result_future.get();

      // Log success and update mission details
      publish_log(mr_msgs::msg::Log::INFO, "Path successfully retrieved from Global Path Planner.");

      // Assuming you have a way to track missions, you might want to add this to your mission structure
      Mission mission;
      mission.start_time = this->now();
      mission.robot_id = robot_id;
      mission.mission_type = "single_goal";
      // TODO : change it to UTMPose
      mission.waypoints = result->poses;
      mission.progress = 0.0;
      missions_[robot.mission_id] = mission;

      // Update robot state
      robot.state = Robot::State::ON_MISSION;
    } catch (const std::exception &e) {
      publish_log(mr_msgs::msg::Log::ERROR, "Failed to retrieve path from Global Path Planner: " + std::string(e.what()));
    }
  } else {
    publish_log(mr_msgs::msg::Log::ERROR, "Timed out waiting for Global Path Planner response.");
  }
}




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiRobotCore>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
