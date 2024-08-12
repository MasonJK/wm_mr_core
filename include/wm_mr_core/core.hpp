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
#include "mr_msgs/msg/mission_data.hpp"
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
  mr_msgs::msg::MissionData mission_data;
  std::string mission_type;

  std::vector<UTMPose> waypoints;
};

class MultiRobotCore : public rclcpp::Node
{
public:
  using Log = mr_msgs::msg::Log;
  using FleetRobotPose = mr_msgs::msg::FleetRobotPose;
  using RobotPose = mr_msgs::msg::RobotPose;
  using MissionData = mr_msgs::msg::MissionData;
  using SingleGoalMission = mr_msgs::srv::SingleGoalMission;

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
  int disconnection_threshold_;

  rclcpp::Publisher<Log>::SharedPtr log_pub_;
  rclcpp::Subscription<FleetRobotPose>::SharedPtr fleet_robot_pose_sub_;

  rclcpp::Service<SingleGoalMission>::SharedPtr single_goal_mission_server_;
  rclcpp::Client<SingleGoalMission>::SharedPtr single_goal_mission_adapter_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};
#endif  // CORE_HPP_
