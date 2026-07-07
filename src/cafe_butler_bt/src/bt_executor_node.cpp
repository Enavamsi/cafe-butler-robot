#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/bt_factory.h>

#include "cafe_butler_interfaces/action/order_task.hpp"
#include "cafe_butler_bt/bt_nodes.hpp"
#include "cafe_butler_bt/order_tracker.hpp"
#include "cafe_butler_bt/waypoint_server.hpp"

using OrderTask = cafe_butler_interfaces::action::OrderTask;
using GoalHandleOrder = rclcpp_action::ServerGoalHandle<OrderTask>;

class CafeButlerBtNode : public rclcpp::Node
{
public:
  CafeButlerBtNode()
  : rclcpp::Node("butler_bt_navigator")
  {
    declare_parameter<std::string>(
      "bt_xml_path", "");  // must be set via launch file per milestone

    tracker_ = std::make_shared<cafe_butler_bt::OrderTracker>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {}));
    waypoints_ = std::make_shared<cafe_butler_bt::WaypointServer>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {}));

    cafe_butler_bt::registerCafeButlerNodes(
      factory_, std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {}), waypoints_, tracker_);

    action_server_ = rclcpp_action::create_server<OrderTask>(
      this, "butler_order",
      std::bind(&CafeButlerBtNode::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&CafeButlerBtNode::handleCancel, this, std::placeholders::_1),
      std::bind(&CafeButlerBtNode::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "butler_bt_navigator ready, waiting for orders on /butler_order");
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const OrderTask::Goal> goal)
  {
    if (goal->tables.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting order with no tables");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleOrder>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleOrder> goal_handle)
  {
    std::thread{std::bind(&CafeButlerBtNode::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleOrder> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    RCLCPP_INFO(
      get_logger(), "Order %s received for %zu table(s)", goal->order_id.c_str(),
      goal->tables.size());

    std::string xml_path = get_parameter("bt_xml_path").as_string();
    if (xml_path.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'bt_xml_path' not set -- cannot run any milestone");
      auto result = std::make_shared<OrderTask::Result>();
      result->success = false;
      result->message = "bt_xml_path not configured";
      goal_handle->abort(result);
      return;
    }

    tracker_->resetForNewOrder(goal->tables);

    auto blackboard = BT::Blackboard::create();
    blackboard->set("tables_queue", goal->tables);
    blackboard->set(
      "delivered_tables", std::vector<std::string>{});
    blackboard->set("skipped_tables", std::vector<std::string>{});
    blackboard->set("fb_state", std::string("STARTING"));
    // Milestones 1-4 are single-table trees and read "current_table" directly
    // (never touching ForEachTable), so seed it with the first table here.
    // Milestones 5-7 overwrite it themselves via ForEachTable each iteration.
    blackboard->set("current_table", goal->tables.front());

    BT::Tree tree;
    try {
      tree = factory_.createTreeFromFile(xml_path, blackboard);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Failed to load BT xml '%s': %s", xml_path.c_str(), e.what());
      auto result = std::make_shared<OrderTask::Result>();
      result->success = false;
      result->message = std::string("BT load error: ") + e.what();
      goal_handle->abort(result);
      return;
    }

    rclcpp::Rate rate(10.0);
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    std::string last_state, last_table;

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
      if (goal_handle->is_canceling()) {
        tree.haltTree();
        auto result = std::make_shared<OrderTask::Result>();
        result->success = false;
        result->message = "Order cancelled by client";
        goal_handle->canceled(result);
        return;
      }

      status = tree.tickOnce();

      std::string state = blackboard->get<std::string>("fb_state");
      std::string table = blackboard->get<std::string>("current_table");
      if (state != last_state || table != last_table) {
        auto fb = std::make_shared<OrderTask::Feedback>();
        fb->state = state;
        fb->current_table = table;
        goal_handle->publish_feedback(fb);
        last_state = state;
        last_table = table;
      }

      rate.sleep();
    }

    auto result = std::make_shared<OrderTask::Result>();
    result->delivered_tables = blackboard->get<std::vector<std::string>>("delivered_tables");
    result->skipped_tables = blackboard->get<std::vector<std::string>>("skipped_tables");
    result->success = (status == BT::NodeStatus::SUCCESS);
    result->message = result->success ? "Order completed" : "Order failed";

    if (status == BT::NodeStatus::SUCCESS) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  }

  BT::BehaviorTreeFactory factory_;
  std::shared_ptr<cafe_butler_bt::OrderTracker> tracker_;
  std::shared_ptr<cafe_butler_bt::WaypointServer> waypoints_;
  rclcpp_action::Server<OrderTask>::SharedPtr action_server_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CafeButlerBtNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
