#ifndef CORE_HPP_
#define CORE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "mr_msgs/msg/log.hpp"
#include "mr_msgs/msg/fleet_robot_pose.hpp"
#include "mr_msgs/msg/robot_pose.hpp"
#include "mr_msgs/srv/plan_global_path.hpp"
#include "mr_msgs/srv/single_goal_mission.hpp"


using std::placeholders::_1, std::placeholders::_2;

struct UTMPose
{
  float x;
  float y;
};

struct Robot
{
  enum class State
  {
    IDLE,
    ON_MISSION,
    DISCONNECTED
  };

  mr_msgs::msg::RobotPose current_pose;
  builtin_interfaces::msg::Time last_connected;
  std::string mission_id;
  State state;
};

struct Mission
{
  builtin_interfaces::msg::Time start_time;
  std::string mission_type;
  std::string robot_id;
  builtin_interfaces::msg::Duration estimated_time_left;
  std::vector<UTMPose> waypoints;
  float progress;
};

class MultiRobotCore : public rclcpp::Node
{
public:
  using Log = mr_msgs::msg::Log;
  using FleetRobotPose = mr_msgs::msg::FleetRobotPose;
  using RobotPose = mr_msgs::msg::RobotPose;
  using SingleGoalMission = mr_msgs::srv::SingleGoalMission;
  using PlanGlobalPath = mr_msgs::srv::PlanGlobalPath;

  explicit MultiRobotCore(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~MultiRobotCore();

private:
  void fleet_robot_pose_callback(const mr_msgs::msg::FleetRobotPose::SharedPtr msg);
  void assign_single_goal_mission(const std::shared_ptr<SingleGoalMission::Request> request,
    std::shared_ptr<SingleGoalMission::Response> response);

  void check_disconnected_robots();
  void publish_log(const int log_type, const std::string &log_content) const;

  std::map<std::string, std::map<std::string, Robot> > robot_fleets_;
  std::map<std::string, Mission> missions_;
  float disconnection_threshold_;

  rclcpp::Publisher<Log>::SharedPtr log_pub_;
  rclcpp::Subscription<FleetRobotPose>::SharedPtr fleet_robot_pose_sub_;

  rclcpp::Service<SingleGoalMission>::SharedPtr single_goal_mission_server_;
  rclcpp::Client<PlanGlobalPath>::SharedPtr gpp_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  // rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
};
#endif  // CORE_HPP_
