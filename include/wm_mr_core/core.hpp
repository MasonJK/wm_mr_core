#ifndef CORE_HPP_
#define CORE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "mr_msgs/msg/fleet_robot_pose.hpp"

using std::placeholders::_1;

class MultiRobotCore : public rclcpp::Node
{
public:
  using FleetRobotPose = mr_msgs::msg::FleetRobotPose;

  explicit MultiRobotCore(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  virtual ~MultiRobotCore();

private:
  void update_fleet_data(std::string fleet);
  void fleet_robot_pose_callback(const FleetRobotPose::SharedPtr msg) const;

  // rclcpp::Publisher<ArithmeticArgument>::SharedPtr arithmetic_argument_publisher_;
  // rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<FleetRobotPose>::SharedPtr fleet_robot_pose_sub_;
  // rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
};
#endif  // CORE_HPP_
