import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from cafe_butler_interfaces.action import OrderTask

TABLES = ["table1", "table2", "table3"]


class ButlerGuiNode(Node):
    def __init__(self):
        super().__init__("cafe_butler_gui")
        self._client = ActionClient(self, OrderTask, "butler_order")
        self._confirm_pub = self.create_publisher(String, "/confirmation", 10)
        self._cancel_pub = self.create_publisher(String, "/cancel_order", 10)
        self._goal_handle = None
        self.status_cb = lambda text: None  # set by the GUI

    def send_order(self, tables, order_id):
        if not self._client.wait_for_server(timeout_sec=2.0):
            self.status_cb("butler_order action server not available")
            return

        goal = OrderTask.Goal()
        goal.tables = tables
        goal.order_id = order_id

        send_future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)
        self.status_cb(f"Order '{order_id}' sent for {tables}")

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.status_cb("Order rejected")
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.status_cb(f"[{fb.current_table or '-'}] {fb.state}")

    def _on_result(self, future):
        result = future.result().result
        self.status_cb(
            f"DONE success={result.success} delivered={list(result.delivered_tables)} "
            f"skipped={list(result.skipped_tables)}"
        )

    def confirm(self, location):
        self._confirm_pub.publish(String(data=location))
        self.status_cb(f"Confirmed: {location}")

    def cancel(self, target):
        self._cancel_pub.publish(String(data=target))
        self.status_cb(f"Cancelled: {target}")

    def cancel_action(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self.status_cb("Requested ROS-level goal cancel")


def build_gui(node: ButlerGuiNode):
    root = tk.Tk()
    root.title("Café Butler - Order Simulator")

    order_var = {t: tk.BooleanVar() for t in TABLES}
    order_id_counter = {"n": 0}

    # ---- Host: place order ----
    host_frame = ttk.LabelFrame(root, text="Host: place order")
    host_frame.pack(fill="x", padx=8, pady=6)
    for t in TABLES:
        ttk.Checkbutton(host_frame, text=t, variable=order_var[t]).pack(side="left", padx=4)

    def on_send_order():
        tables = [t for t in TABLES if order_var[t].get()]
        if not tables:
            return
        order_id_counter["n"] += 1
        node.send_order(tables, f"order-{order_id_counter['n']}")

    ttk.Button(host_frame, text="Send Order", command=on_send_order).pack(side="left", padx=8)
    ttk.Button(host_frame, text="Cancel goal (ROS level)", command=node.cancel_action).pack(
        side="left", padx=8
    )

    # ---- Kitchen ----
    kitchen_frame = ttk.LabelFrame(root, text="Kitchen")
    kitchen_frame.pack(fill="x", padx=8, pady=6)
    ttk.Button(
        kitchen_frame, text="Confirm (food handed to robot)",
        command=lambda: node.confirm("kitchen")
    ).pack(side="left", padx=4)
    ttk.Button(
        kitchen_frame, text="Cancel whole order",
        command=lambda: node.cancel("all")
    ).pack(side="left", padx=4)

    # ---- Tables ----
    for t in TABLES:
        frame = ttk.LabelFrame(root, text=t)
        frame.pack(fill="x", padx=8, pady=4)
        ttk.Button(frame, text="Confirm receipt", command=lambda t=t: node.confirm(t)).pack(
            side="left", padx=4
        )
        ttk.Button(frame, text="Cancel this table", command=lambda t=t: node.cancel(t)).pack(
            side="left", padx=4
        )

    # ---- Status ----
    status_frame = ttk.LabelFrame(root, text="Status / feedback")
    status_frame.pack(fill="both", expand=True, padx=8, pady=6)
    status_text = tk.Text(status_frame, height=10, width=70)
    status_text.pack(fill="both", expand=True)

    def append_status(line):
        def _do():
            status_text.insert("end", line + "\n")
            status_text.see("end")
        root.after(0, _do)

    node.status_cb = append_status
    return root


def main():
    rclpy.init()
    node = ButlerGuiNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = build_gui(node)
    try:
        root.mainloop()
    finally:
        rclpy.shutdown()


