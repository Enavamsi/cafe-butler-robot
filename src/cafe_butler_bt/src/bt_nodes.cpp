#include "cafe_butler_bt/bt_nodes.hpp"

namespace cafe_butler_bt
{

// ---------------------------------------------------------------------------
// NavigateToWaypoint
// ---------------------------------------------------------------------------
NavigateToWaypoint::NavigateToWaypoint(
  const std::string & name, const BT::NodeConfig & config,
  rclcpp::Node::SharedPtr node, std::shared_ptr<WaypointServer> waypoints)
: BT::StatefulActionNode(name, config), node_(node), waypoints_(waypoints)
{
  client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
}

BT::NodeStatus NavigateToWaypoint::onStart()
{
  std::string waypoint;
  if (!getInput("waypoint", waypoint)) {
    RCLCPP_ERROR(node_->get_logger(), "NavigateToWaypoint: missing required input [waypoint]");
    return BT::NodeStatus::FAILURE;
  }

  if (!client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available");
    return BT::NodeStatus::FAILURE;
  }

  NavigateToPose::Goal goal;
  goal.pose = waypoints_->get(waypoint);

  goal_done_ = false;
  goal_success_ = false;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
  options.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      goal_done_ = true;
      goal_success_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
    };

  goal_future_ = client_->async_send_goal(goal, options);
  RCLCPP_INFO(node_->get_logger(), "Navigating to '%s'", waypoint.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToWaypoint::onRunning()
{
  if (!goal_handle_ && goal_future_.valid() &&
      goal_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
  {
    goal_handle_ = goal_future_.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(node_->get_logger(), "Nav2 rejected the goal");
      return BT::NodeStatus::FAILURE;
    }
  }

  if (goal_done_) {
    return goal_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void NavigateToWaypoint::onHalted()
{
  if (goal_handle_) {
    client_->async_cancel_goal(goal_handle_);
  }
}

// ---------------------------------------------------------------------------
// WaitForConfirmation
// ---------------------------------------------------------------------------
WaitForConfirmation::WaitForConfirmation(
  const std::string & name, const BT::NodeConfig & config,
  std::shared_ptr<OrderTracker> tracker)
: BT::StatefulActionNode(name, config), tracker_(tracker)
{
}

BT::NodeStatus WaitForConfirmation::onStart()
{
  double timeout_s = 10.0;
  getInput("timeout", timeout_s);
  getInput("location", location_);
  tracker_->clearConfirmation(location_);
  deadline_ = rclcpp::Clock().now() + rclcpp::Duration::from_seconds(timeout_s);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForConfirmation::onRunning()
{
  if (tracker_->isConfirmed(location_)) {
    return BT::NodeStatus::SUCCESS;
  }
  if (rclcpp::Clock().now() >= deadline_) {
    return BT::NodeStatus::FAILURE;  // timed out
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForConfirmation::onHalted() {}

// ---------------------------------------------------------------------------
// IsTableCancelled
// ---------------------------------------------------------------------------
IsTableCancelled::IsTableCancelled(
  const std::string & name, const BT::NodeConfig & config,
  std::shared_ptr<OrderTracker> tracker)
: BT::ConditionNode(name, config), tracker_(tracker)
{
}

BT::NodeStatus IsTableCancelled::tick()
{
  std::string table;
  getInput("table", table);
  return tracker_->isCancelled(table) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// ForEachTable
// ---------------------------------------------------------------------------
ForEachTable::ForEachTable(const std::string & name, const BT::NodeConfig & config)
: BT::ControlNode(name, config)
{
}

BT::NodeStatus ForEachTable::tick()
{
  auto tables = config().blackboard->get<std::vector<std::string>>("tables_queue");

  if (index_ >= tables.size()) {
    index_ = 0;
    child_started_ = false;
    return BT::NodeStatus::SUCCESS;  // all tables processed
  }

  config().blackboard->set("current_table", tables[index_]);

  TreeNode * child = children_nodes_[0];
  BT::NodeStatus status = child->executeTick();

  if (status == BT::NodeStatus::RUNNING) {
    return BT::NodeStatus::RUNNING;
  }

  // record outcome
  auto delivered = config().blackboard->get<std::vector<std::string>>("delivered_tables");
  auto skipped = config().blackboard->get<std::vector<std::string>>("skipped_tables");
  if (status == BT::NodeStatus::SUCCESS) {
    delivered.push_back(tables[index_]);
  } else {
    skipped.push_back(tables[index_]);
  }
  config().blackboard->set("delivered_tables", delivered);
  config().blackboard->set("skipped_tables", skipped);

  haltChild(0);
  index_++;
  return BT::NodeStatus::RUNNING;  // re-tick ourselves next step to process the next table
}

void ForEachTable::halt()
{
  index_ = 0;
  child_started_ = false;
  haltChildren();
  setStatus(BT::NodeStatus::IDLE);
}

// ---------------------------------------------------------------------------
// PublishFeedback
// ---------------------------------------------------------------------------
PublishFeedback::PublishFeedback(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
}

BT::NodeStatus PublishFeedback::tick()
{
  std::string state;
  getInput("state", state);
  config().blackboard->set("fb_state", state);
  return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------
void registerCafeButlerNodes(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Node::SharedPtr node,
  std::shared_ptr<WaypointServer> waypoints,
  std::shared_ptr<OrderTracker> tracker)
{
  factory.registerBuilder<NavigateToWaypoint>(
    "NavigateToWaypoint",
    [node, waypoints](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<NavigateToWaypoint>(name, config, node, waypoints);
    });

  factory.registerBuilder<WaitForConfirmation>(
    "WaitForConfirmation",
    [tracker](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<WaitForConfirmation>(name, config, tracker);
    });

  factory.registerBuilder<IsTableCancelled>(
    "IsTableCancelled",
    [tracker](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<IsTableCancelled>(name, config, tracker);
    });

  factory.registerNodeType<ForEachTable>("ForEachTable");
  factory.registerNodeType<PublishFeedback>("PublishFeedback");
}

}  // namespace cafe_butler_bt
