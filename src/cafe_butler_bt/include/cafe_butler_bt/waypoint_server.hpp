#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <string>
#include <stdexcept>

namespace cafe_butler_bt
{


{
public:
  explicit WaypointServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    load();
  }

  geometry_msgs::msg::PoseStamped get(const std::string & name) const
  {
    auto it = waypoints_.find(name);
    if (it == waypoints_.end()) {
      throw std::runtime_error("Unknown waypoint: " + name);
    }
    auto pose = it->second;
    pose.header.stamp = node_->now();
    return pose;
  }

  bool has(const std::string & name) const
  {
    return waypoints_.count(name) > 0;
  }

private:
  void load()
  {
    node_->declare_parameter<std::vector<std::string>>(
      "waypoint_names", std::vector<std::string>{"home", "kitchen", "table1", "table2", "table3"});
    auto names = node_->get_parameter("waypoint_names").as_string_array();

    for (const auto & name : names) {

      node_->declare_parameter<std::vector<double>>(
        "waypoints." + name + ".position", std::vector<double>{0.0, 0.0});
      node_->declare_parameter<std::vector<double>>(
        "waypoints." + name + ".orientation", std::vector<double>{0.0, 0.0, 0.0, 1.0});

      auto position = node_->get_parameter("waypoints." + name + ".position").as_double_array();
      auto orientation = node_->get_parameter("waypoints." + name + ".orientation").as_double_array();

      if (position.size() != 2 || orientation.size() != 4) {
        RCLCPP_WARN(
          node_->get_logger(),
          "Waypoint '%s' has malformed position/orientation, defaulting to origin", name.c_str());
        position = {0.0, 0.0};
        orientation = {0.0, 0.0, 0.0, 1.0};
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = position[0];
      pose.pose.position.y = position[1];
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = orientation[0];
      pose.pose.orientation.y = orientation[1];
      pose.pose.orientation.z = orientation[2];
      pose.pose.orientation.w = orientation[3];
      waypoints_[name] = pose;
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> waypoints_;
};

}  // namespace cafe_butler_bt
