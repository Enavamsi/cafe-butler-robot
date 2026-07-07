#pragma once
#include <future>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "cafe_butler_bt/order_tracker.hpp"
#include "cafe_butler_bt/waypoint_server.hpp"

namespace cafe_butler_bt
{


class NavigateToWaypoint : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToWaypoint(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Node::SharedPtr node, std::shared_ptr<WaypointServer> waypoints);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("waypoint", "Named waypoint e.g. 'kitchen', 'table1'")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<WaypointServer> waypoints_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  GoalHandle::SharedPtr goal_handle_;
  bool goal_done_{false};
  bool goal_success_{false};
};


class WaitForConfirmation : public BT::StatefulActionNode
{
public:
  WaitForConfirmation(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<OrderTracker> tracker);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("location"),
      BT::InputPort<double>("timeout", 10.0, "seconds to wait before giving up")};
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<OrderTracker> tracker_;
  rclcpp::Time deadline_;
  std::string location_;
};


class IsTableCancelled : public BT::ConditionNode
{
public:
  IsTableCancelled(
    const std::string & name, const BT::NodeConfig & config,
    std::shared_ptr<OrderTracker> tracker);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("table")};
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<OrderTracker> tracker_;
};


class ForEachTable : public BT::ControlNode
{
public:
  ForEachTable(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  size_t index_{0};
  bool child_started_{false};
};


class PublishFeedback : public BT::SyncActionNode
{
public:
  PublishFeedback(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("state")};
  }

  BT::NodeStatus tick() override;
};


void registerCafeButlerNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<WaypointServer> waypoints,
  std::shared_ptr<OrderTracker> tracker);

}  
