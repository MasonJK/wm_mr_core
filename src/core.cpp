#include <functional>
#include <memory>

#include "wm_mr_core/core.hpp"

MultiRobotCore::MultiRobotCore(const rclcpp::NodeOptions & node_options)
: Node("multi_robot_core_node", node_options)
{
  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = this->get_parameter("qos_depth").get_value<int8_t>();

  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

  // topic pub, sub
  fleet_robot_pose_sub_ = this->create_subscription<FleetRobotPose>(
    "fleet_robot_pose",
    QOS_RKL10V,
    std::bind(&MultiRobotCore::fleet_robot_pose_callback, this, _1));

  // service server
  single_goal_mission_server_ = this->create_service<SingleGoalMission>("single_goal_mission",
    std::bind(&MultiRobotCore::assign_single_goal_mission, this, _1, _2));
}

MultiRobotCore::~MultiRobotCore()
{
}

void MultiRobotCore::fleet_robot_pose_callback(const mr_msgs::msg::FleetRobotPose::SharedPtr msg)
{
  std::vector<RobotPose> fleet_robot_poses;
  for (const auto& pose : msg->fleet_pose)
  {
    fleet_robot_poses.push_back(pose);
  }

  fleet_pose[msg->fleet_name] = fleet_robot_poses;
}

void MultiRobotCore::assign_single_goal_mission(const std::shared_ptr<SingleGoalMission::Request> request,
  std::shared_ptr<SingleGoalMission::Response> response){

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiRobotCore>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
