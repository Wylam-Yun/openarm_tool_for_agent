#!/usr/bin/env python3
"""set_gripper 后端：原地开合夹爪，手臂不动。

用法：python3 set_gripper.py '{"arm":"right","gripper":1,"steps":10,"interval":0.05}'
只打 GripperCommand Action，不发布任何手臂话题。
`release` 由 TS 层固定 gripper=-1 调用本脚本。
"""
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_common as C

import numpy as np


def current_width(node, live, side, cfg):
    """从 joint_states 按名找夹爪开口；找不到返回 None（调用方用对端兜底）。"""
    msg = live.joint_msg or live.wait_joints(timeout_s=3.0)
    if msg is None:
        return None
    for n, p in zip(msg.name, msg.position):
        ln = n.lower()
        if ("grip" in ln or "finger" in ln) and side in ln:
            return float(p)
    return None


def drive_gripper(node, live, side, target_m, steps, interval, cfg):
    """按 steps 步插值下发 GripperCommand，返回 (ok, last_width)。"""
    from control_msgs.action import GripperCommand
    from rclpy.action import ActionClient

    topic = (
        cfg["topics"]["gripper_left"]
        if side == "left"
        else cfg["topics"]["gripper_right"]
    )
    client = ActionClient(node, GripperCommand, topic)
    if not client.wait_for_server(timeout_sec=5.0):
        return False, None

    start = current_width(node, live, side, cfg)
    if start is None:  # 读不到就从另一端插值，结果一致，只是多走几步
        g = cfg["gripper"]
        start = g["open_m"] if target_m < g["open_m"] / 2 else g["closed_m"]
    traj = np.linspace(start, target_m, steps + 1)[1:]
    ok, width = True, start
    for v in traj:
        goal = GripperCommand.Goal()
        goal.command.position = float(v)
        goal.command.max_effort = float(cfg["gripper"]["max_effort"])
        fut = client.send_goal_async(goal)
        rclpy = C.rclpy
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        if not fut.done():
            return False, width
        handle = fut.result()
        if handle is None or not handle.accepted:
            ok = False
            break
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res_fut, timeout_sec=10.0)
        if not res_fut.done():
            cancel_fut = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_fut, timeout_sec=2.0)
            ok = False
            break
        res = res_fut.result()
        if res is None:
            ok = False
            break
        width = float(res.result.position)
        if not res.result.reached_goal:
            ok = False
            break
        time.sleep(interval)
    return ok, width


def main():
    C.check_ros()
    import rclpy

    try:
        args = json.loads(sys.argv[1]) if len(sys.argv) > 1 else {}
    except json.JSONDecodeError as e:
        C.die(f"参数不是合法 JSON：{e}")
    arm = args.get("arm")
    gripper = args.get("gripper")
    steps = int(args.get("steps", 10))
    interval = float(args.get("interval", 0.05))
    if arm not in ("left", "right"):
        C.die("arm 必须显传 left/right", got=arm)
    if gripper not in (1, -1):
        C.die("gripper 只能是 +1(闭)/-1(开)", got=gripper)
    if not (1 <= steps <= 100):
        C.die("steps 必须在 1~100", got=steps)
    if not (0.0 <= interval <= 2.0):
        C.die("interval 必须在 0~2 秒", got=interval)

    cfg = C.load_config()
    target = (
        cfg["gripper"]["closed_m"] if gripper == 1 else cfg["gripper"]["open_m"]
    )
    rclpy.init()
    node = rclpy.create_node("arm_tools_gripper")
    try:
        live = C.LiveState(node, cfg)
        ok, width = drive_gripper(node, live, arm, target, steps, interval, cfg)
        C.emit({"success": ok, "arm": arm, "width": width})
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
