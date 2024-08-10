#ifndef CORE_HPP_
#define CORE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/duration.hpp"
#include "mr_msgs/msg/fleet_robot_pose.hpp"
#include "mr_msgs/msg/robot_pose.hpp"
#include "mr_msgs/srv/single_goal_mission.hpp"


using std::placeholders::_1, std::placeholders::_2;

struct Mission
{
  builtin_interfaces::msg::Time start_time;
  std::string mission_id;
  std::string mission_type;
  std::string robot_id;
};

class MultiRobotCore : public rclcpp::Node
{
public:
  using FleetRobotPose = mr_msgs::msg::FleetRobotPose;
  using RobotPose = mr_msgs::msg::RobotPose;
  using SingleGoalMission = mr_msgs::srv::SingleGoalMission;

  explicit MultiRobotCore(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~MultiRobotCore();

private:
  void fleet_robot_pose_callback(const mr_msgs::msg::FleetRobotPose::SharedPtr msg);
  void assign_single_goal_mission(const std::shared_ptr<SingleGoalMission::Request> request,
    std::shared_ptr<SingleGoalMission::Response> response);

  std::map<std::string, std::vector<RobotPose> > fleet_pose;
  std::map<std::string, Mission> missions;

  // rclcpp::Publisher<ArithmeticArgument>::SharedPtr arithmetic_argument_publisher_;
  // rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<FleetRobotPose>::SharedPtr fleet_robot_pose_sub_;
  rclcpp::Service<SingleGoalMission>::SharedPtr single_goal_mission_server_;
  // rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
};
#endif  // CORE_HPP_
