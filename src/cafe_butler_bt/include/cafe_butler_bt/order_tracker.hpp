#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <mutex>
#include <set>
#include <string>

namespace cafe_butler_bt
{

// Central, thread-safe place that remembers:
//  - which locations ("kitchen", "table1", "table2", "table3") have been
//    confirmed by a person pressing a button on the GUI
//  - which tables (or "all") have been cancelled
//
// This is the ONLY thing that talks to the /confirmation and /cancel_order
// topics. BT leaf nodes just poll this object, so the tree itself has no
// ROS-callback code in it -- keeps it generic and testable.
class OrderTracker
{
public:
  explicit OrderTracker(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    confirm_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/confirmation", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        confirmed_.insert(msg->data);
        RCLCPP_INFO(node_->get_logger(), "Confirmation received for '%s'", msg->data.c_str());
      });

    cancel_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/cancel_order", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        cancelled_.insert(msg->data);
        RCLCPP_WARN(node_->get_logger(), "Cancel received for '%s'", msg->data.c_str());
      });
  }

  bool isConfirmed(const std::string & location)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return confirmed_.count(location) > 0;
  }

  bool isCancelled(const std::string & table)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return cancelled_.count(table) > 0 || cancelled_.count("all") > 0;
  }

  // Reset state at the start of a new order / before waiting on a location again
  void clearConfirmation(const std::string & location)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    confirmed_.erase(location);
  }

  void resetForNewOrder(const std::vector<std::string> & tables)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    confirmed_.clear();
    cancelled_.erase("all");
    for (const auto & t : tables) {
      cancelled_.erase(t);
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr confirm_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_sub_;
  std::mutex mutex_;
  std::set<std::string> confirmed_;
  std::set<std::string> cancelled_;
};

}  // namespace cafe_butler_bt
