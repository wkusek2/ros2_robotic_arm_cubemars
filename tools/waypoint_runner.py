#!/usr/bin/env python3
"""
Waypoint runner — plans and executes joint-space waypoints via MoveIt2 /move_action.
Requires the full launch stack to be running (ros2 launch arm6dof main.launch.py).

Usage:
    python3 tools/waypoint_runner.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    JointConstraint,
    PlanningOptions,
)
import time

# ─── Waypoints ────────────────────────────────────────────────────────────────
# Each entry: [joint1, joint2, joint3, joint4, joint5, joint6]  (radians)
# Keep values small until you verify your URDF limits and physical clearances.

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

WAYPOINTS = [
    [ 0.0,   0.0,   0.0,   0.0,   -0.5,   0.5],   # home
    [ 0.4,   1.3,   0.0,   1.0,   0.5,   -0.5],   # A
    [ 0.4,   0.3,   0.3,   1.2,   -0.5,   0.5],   # B
    [ 0.0,   0.3,   1.3,   0.2,   0.5,   -0.5],   # C
    [-1.4,   1.3,   0.0,   1.0,   -0.5,   0.5],   # D
    [ 0.0,   0.0,   1.0,   0.0,   0.5,   -0.5],   # home
]

# ─── Timing ───────────────────────────────────────────────────────────────────
VELOCITY_SCALING     = 1.0   # 0.0–1.0
ACCELERATION_SCALING = 1.0
PLANNING_TIME_SEC    = 5.0
PAUSE_BETWEEN_SEC    = 0.5
LOOP                 = True   # set False to run once

# ─── MoveGroup goal builder ───────────────────────────────────────────────────

def make_goal(joint_positions: list[float]) -> MoveGroup.Goal:
    joint_constraints = [
        JointConstraint(
            joint_name=name,
            position=pos,
            tolerance_above=0.01,
            tolerance_below=0.01,
            weight=1.0,
        )
        for name, pos in zip(JOINT_NAMES, joint_positions)
    ]

    request = MotionPlanRequest(
        group_name="arm",
        goal_constraints=[Constraints(joint_constraints=joint_constraints)],
        allowed_planning_time=PLANNING_TIME_SEC,
        max_velocity_scaling_factor=VELOCITY_SCALING,
        max_acceleration_scaling_factor=ACCELERATION_SCALING,
        num_planning_attempts=3,
    )

    options = PlanningOptions()
    options.plan_only = False
    options.replan = False

    goal = MoveGroup.Goal()
    goal.request = request
    goal.planning_options = options
    return goal


# ─── Node ─────────────────────────────────────────────────────────────────────

class WaypointRunner(Node):
    def __init__(self):
        super().__init__("waypoint_runner")
        self._client = ActionClient(self, MoveGroup, "/move_action")

    def run(self):
        self.get_logger().info("Czekam na /move_action...")
        self._client.wait_for_server()
        self.get_logger().info("Gotowy. Start trasy.")

        idx = 0
        while rclpy.ok():
            wp = WAYPOINTS[idx % len(WAYPOINTS)]
            self.get_logger().info(
                f"Waypoint {idx % len(WAYPOINTS)}: {[f'{v:.2f}' for v in wp]}"
            )

            goal = make_goal(wp)
            future = self._client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, future)

            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("Goal odrzucony, pomijam.")
                idx += 1
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result().result
            code = result.error_code.val
            if code == 1:
                self.get_logger().info("OK")
            else:
                self.get_logger().warn(f"Blad planowania: error_code={code}")

            idx += 1
            if not LOOP and idx >= len(WAYPOINTS):
                break

            time.sleep(PAUSE_BETWEEN_SEC)


def main():
    rclpy.init()
    node = WaypointRunner()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
