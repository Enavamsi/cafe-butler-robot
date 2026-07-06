#pragma once
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

#include "cafe_butler_bt/order_tracker.hpp"
#include "cafe_butler_bt/waypoint_server.hpp"

namespace cafe_butler_bt
{

// ---------------------------------------------------------------------------
// NavigateToWaypoint  (generic: works for "home", "kitchen", "table1", ...)
// Sends a NavigateToPose goal to Nav2 for the named waypoint and waits for
// the robot to arrive. This is the ONLY node that talks to Nav2, so every
// milestone reuses it unmodified.
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// WaitForConfirmation  (generic: works for "kitchen" or any "tableN")
// Polls OrderTracker until someone confirms at `location`, or `timeout`
// seconds pass. SUCCESS = confirmed, FAILURE = timed out.
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// IsTableCancelled  -- SUCCESS if the table (or whole order) was cancelled
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// ForEachTable  -- generic control node.
// Iterates the "tables_queue" (std::vector<std::string>) stored on the
// blackboard, one element at a time, exposing the active one as
// "current_table". Ticks its single child once per table.
//   child == SUCCESS -> table recorded as delivered, advance
//   child == FAILURE -> table recorded as skipped,   advance
//   child == RUNNING -> propagate RUNNING, don't advance
// Returns SUCCESS once the queue is empty (this is what makes milestones
// 5/6/7 "generic": 1 table or N tables run through the exact same node).
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// PublishFeedback  -- forwards a state string to the action server feedback
// (looked up from the blackboard as a std::function set up by the executor)
// ---------------------------------------------------------------------------
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

// Registers every node above with the factory, wiring in the shared ROS
// resources (node handle, waypoint server, order tracker) via captured
// lambdas -- this keeps BT.CPP's plugin API happy while letting our nodes
// use real ROS objects instead of globals.
void registerCafeButlerNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<WaypointServer> waypoints,
  std::shared_ptr<OrderTracker> tracker);

}  // namespace cafe_butler_bt
