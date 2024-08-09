#ifndef CORE_HPP_
#define CORE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "mr_msgs/msg/fleet_robot_pose.hpp"
#include "mr_msgs/msg/robot_pose.hpp"

using std::placeholders::_1;

class MultiRobotCore : public rclcpp::Node
{
public:
  using FleetRobotPose = mr_msgs::msg::FleetRobotPose;
  using RobotPose = mr_msgs::msg::RobotPose;

  explicit MultiRobotCore(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~MultiRobotCore();

private:
  void fleet_robot_pose_callback(const mr_msgs::msg::FleetRobotPose::SharedPtr msg) const;

  std::map<std::string, std::vector<RobotPose> > fleet_pose;

  // rclcpp::Publisher<ArithmeticArgument>::SharedPtr arithmetic_argument_publisher_;
  // rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<FleetRobotPose>::SharedPtr fleet_robot_pose_sub_;
  // rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
};
#endif  // CORE_HPP_
