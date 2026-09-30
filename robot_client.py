#!/usr/bin/env python3
"""Small MoveIt client shared by the agent motion tools.

The agent tools express targets for hand_tcp. MoveIt's planning group ends at
link7, so the position constraint uses link7 plus the fixed TCP offset from the
OpenArm URDF. This file owns all MoveGroup action and final TF verification
logic; individual tools only prepare targets and sequence operations.
"""
import math
import time

import numpy as np

import arm_common as C


def moveit_error_detail(code):
    """Return an agent-facing explanation for MoveIt error codes."""
    details = {
        99999: (
            "MoveIt 规划失败：在当前姿态、关节限位和碰撞约束下，"
            "规划器找不到有效的关节解"
        ),
        -1: (
            "MoveIt 规划失败：在当前姿态、关节限位和碰撞约束下，"
            "规划器找不到有效的关节解"
        ),
        -2: "MoveIt 规划失败：生成的运动计划无效",
        -12: "MoveIt 规划失败：目标位姿处于碰撞状态",
        -14: "MoveIt 规划失败：目标不满足路径约束",
        -31: "MoveIt 规划失败：目标位姿没有可用的 IK 关节解",
        -4: "MoveIt 执行失败：控制器没有完成轨迹",
        -6: "MoveIt 执行超时",
        -7: "MoveIt 执行被取消或抢占",
    }
    return details.get(
        int(code),
        f"MoveIt 返回错误码 {int(code)}，未能完成规划或执行",
    )


class MoveItClient:
    def __init__(self, node, live, cfg):
        from geometry_msgs.msg import Pose
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import (
            Constraints,
            OrientationConstraint,
            PositionConstraint,
        )
        from rclpy.action import ActionClient

        self.node = node
        self.live = live
        self.cfg = cfg
        self.MoveGroup = MoveGroup
        self.Constraints = Constraints
        self.OrientationConstraint = OrientationConstraint
        self.PositionConstraint = PositionConstraint
        self.Pose = Pose
        self.client = ActionClient(node, MoveGroup, cfg["moveit"]["action"])

    @staticmethod
    def _quat_normalize(q):
        q = np.asarray(q, dtype=np.float64)
        n = float(np.linalg.norm(q))
        if not np.isfinite(n) or n < 1e-9:
            raise ValueError("四元数无效")
        return q / n

    @staticmethod
    def _orientation_error(q_a, q_b):
        a = MoveItClient._quat_normalize(q_a)
        b = MoveItClient._quat_normalize(q_b)
        dot = min(1.0, max(-1.0, abs(float(np.dot(a, b)))))
        return 2.0 * math.acos(dot)

    def _goal_constraints(self, side, xyz_world, quat_wxyz,
                          position_tolerance, orientation_tolerance):
        from shape_msgs.msg import SolidPrimitive

        frames = self.cfg["frames"]
        link = frames["link_left"] if side == "left" else frames["link_right"]
        q = self._quat_normalize(quat_wxyz)

        constraints = self.Constraints()

        pos = self.PositionConstraint()
        pos.header.frame_id = frames["world"]
        pos.link_name = link
        offset = self.cfg["moveit"]["tcp_offset_in_link7_m"]
        pos.target_point_offset.x = float(offset[0])
        pos.target_point_offset.y = float(offset[1])
        pos.target_point_offset.z = float(offset[2])
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(max(position_tolerance, 0.003))]
        pos.constraint_region.primitives.append(sphere)
        pose = self.Pose()
        pose.position.x, pose.position.y, pose.position.z = [float(v) for v in xyz_world]
        pose.orientation.w = 1.0
        pos.constraint_region.primitive_poses.append(pose)
        pos.weight = 1.0
        constraints.position_constraints.append(pos)

        ori = self.OrientationConstraint()
        ori.header.frame_id = frames["world"]
        ori.link_name = link
        ori.orientation.x = float(q[1])
        ori.orientation.y = float(q[2])
        ori.orientation.z = float(q[3])
        ori.orientation.w = float(q[0])
        ori.absolute_x_axis_tolerance = float(max(orientation_tolerance, 0.01))
        ori.absolute_y_axis_tolerance = float(max(orientation_tolerance, 0.01))
        ori.absolute_z_axis_tolerance = float(max(orientation_tolerance, 0.01))
        ori.parameterization = ori.ROTATION_VECTOR
        ori.weight = 1.0
        constraints.orientation_constraints.append(ori)
        return constraints

    def move_tcp(self, side, xyz_world, quat_wxyz, timeout_s=20.0,
                 position_tolerance=0.012, orientation_tolerance=0.06,
                 velocity_scaling=0.15):
        """Plan and execute one hand_tcp target, then verify the real TCP TF."""
        from moveit_msgs.msg import MoveItErrorCodes

        if side not in ("left", "right"):
            return {"success": False, "error": "arm 必须是 left/right"}
        xyz_world = np.asarray(xyz_world, dtype=np.float64)
        if xyz_world.shape != (3,) or not np.all(np.isfinite(xyz_world)):
            return {"success": False, "error": "目标 xyz 无效"}
        try:
            quat_wxyz = self._quat_normalize(quat_wxyz)
        except ValueError as exc:
            return {"success": False, "error": str(exc)}

        if not self.client.wait_for_server(timeout_sec=5.0):
            return {
                "success": False,
                "error": "MoveIt action 不可用",
                "action": self.cfg["moveit"]["action"],
            }

        goal = self.MoveGroup.Goal()
        req = goal.request
        req.group_name = (
            self.cfg["moveit"]["group_left"]
            if side == "left"
            else self.cfg["moveit"]["group_right"]
        )
        req.goal_constraints = [self._goal_constraints(
            side, xyz_world, quat_wxyz, position_tolerance, orientation_tolerance)]
        req.num_planning_attempts = 3
        req.allowed_planning_time = float(self.cfg["moveit"].get("planning_time_s", 5.0))
        req.max_velocity_scaling_factor = float(velocity_scaling)
        req.max_acceleration_scaling_factor = float(velocity_scaling)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False

        send_future = self.client.send_goal_async(goal)
        C.rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        if not send_future.done() or send_future.result() is None:
            return {"success": False, "error": "MoveIt goal 发送超时"}
        handle = send_future.result()
        if not handle.accepted:
            return {"success": False, "error": "MoveIt 拒绝 goal"}

        result_future = handle.get_result_async()
        deadline = time.monotonic() + float(timeout_s)
        while not result_future.done() and time.monotonic() < deadline:
            C.rclpy.spin_once(self.node, timeout_sec=0.05)
        if not result_future.done() or result_future.result() is None:
            cancel_future = handle.cancel_goal_async()
            C.rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=2.0)
            return {"success": False, "error": "MoveIt 执行超时"}

        result = result_future.result().result
        code = int(result.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            detail = moveit_error_detail(code)
            return {
                "success": False,
                "error": detail,
                "moveit_error_detail": detail,
                "moveit_error_code": code,
                "state": getattr(result, "state", ""),
            }

        final = self.live.get_tcp(side, timeout_s=3.0)
        if final is None:
            return {
                "success": False,
                "error": "执行完成但读不到最终 TCP TF",
                "moveit_error_code": code,
            }
        final_xyz = np.asarray(final["xyz"], dtype=np.float64)
        position_error = float(np.linalg.norm(final_xyz - xyz_world))
        orientation_error = self._orientation_error(final["quat"], quat_wxyz)
        success = (
            position_error <= float(position_tolerance)
            and orientation_error <= float(orientation_tolerance)
        )
        return {
            "success": success,
            "error": None if success else "执行完成但末端误差超限",
            "final_xyz": [round(float(v), 4) for v in final_xyz],
            "final_quat_wxyz": [round(float(v), 6) for v in final["quat"]],
            "position_error_m": round(position_error, 4),
            "orientation_error_rad": round(orientation_error, 4),
            "moveit_error_code": code,
            "planning_time_s": round(float(result.planning_time), 3),
        }


def quat_from_matrix(R):
    """Return a normalized quaternion in [w, x, y, z] order."""
    R = np.asarray(R, dtype=np.float64)
    trace = float(np.trace(R))
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] - R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


def matrix_from_quat(q):
    w, x, y, z = MoveItClient._quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])
