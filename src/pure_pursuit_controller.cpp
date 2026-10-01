#include "nav2_pure_pursuit_controller/pure_pursuit_controller.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.hpp"
#include <algorithm>
#include <cmath>

namespace nav2_pure_pursuit_controller
{

PurePursuitController::PurePursuitController() = default;

void PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> /*costmap_ros*/) 
{
  parent_node_ = parent;
  tf_ = tf;
  plugin_name_ = name;
  auto node = parent.lock();
  logger_ = node->get_logger();

  node->declare_parameter(plugin_name_ + ".linear_vel", 0.5);
  node->declare_parameter(plugin_name_ + ".k_sigma", 1.5);
  
  node->get_parameter(plugin_name_ + ".linear_vel", linear_vel_);
  node->get_parameter(plugin_name_ + ".k_sigma", k_sigma_);
}

void PurePursuitController::cleanup()
{
  global_path_.poses.clear();
}

void PurePursuitController::activate()
{
  RCLCPP_INFO(logger_, "Activating PurePursuitController plugin.");
}

void PurePursuitController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating PurePursuitController plugin.");
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_path_ = path;
}

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = rclcpp::Clock().now();
  cmd_vel.header.frame_id = "base_link";

  if (global_path_.poses.empty()) {
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    return cmd_vel;
  }

  auto target_pose = global_path_.poses[0];
  double min_dist = std::numeric_limits<double>::max();
  
  for (const auto & waypoint : global_path_.poses) {
    double dx = waypoint.pose.position.x - pose.pose.position.x;
    double dy = waypoint.pose.position.y - pose.pose.position.y;
    double dist = std::hypot(dx, dy);
    if (dist < min_dist) {
      min_dist = dist;
      target_pose = waypoint;
    }
  }

  double yaw = tf2::getYaw(pose.pose.orientation);
  double target_yaw = std::atan2(
    target_pose.pose.position.y - pose.pose.position.y,
    target_pose.pose.position.x - pose.pose.position.x);
  
  double heading_error = target_yaw - yaw;
  heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));

  cmd_vel.twist.linear.x = linear_vel_;
  cmd_vel.twist.angular.z = k_sigma_ * heading_error;

  return cmd_vel;
}

void PurePursuitController::setSpeedLimit(const double & /*speed_limit*/, const bool & /*is_percentage*/)
{
}

}  

PLUGINLIB_EXPORT_CLASS(
  nav2_pure_pursuit_controller::PurePursuitController,
  nav2_core::Controller)