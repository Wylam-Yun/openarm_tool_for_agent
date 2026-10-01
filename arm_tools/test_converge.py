#!/usr/bin/env python3
"""Mocked convergence-loop test for MoveItClient.move_tcp.

Stubs _move_tcp_once (the only ROS-touching part) with droop models, drives the
real move_tcp loop, and asserts stop reasons / telemetry / external residual.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

from robot_client import MoveItClient

CFG = {
    "moveit": {
        "converge_enabled": True,
        "converge_max_iters": 3,
        "converge_max_compensation_m": 0.08,
        "converge_retry_timeout_s": 8.0,
    },
    "limits": {"max_single_move_m": 0.30, "min_z_m": 0.0},
}

PTOL = 0.005
OTOL = 0.02
QUAT = [1.0, 0.0, 0.0, 0.0]
T = np.array([0.30, -0.05, 0.25])
TIMEOUTS = []


def make_result(final_xyz, commanded, ori_err=0.0, code=1):
    pos_err = float(np.linalg.norm(final_xyz - commanded))
    ok = pos_err <= PTOL and ori_err <= OTOL
    return {
        "success": ok,
        "error": None if ok else "执行完成但末端误差超限",
        "final_xyz": [float(v) for v in final_xyz],
        "final_quat_wxyz": QUAT,
        "position_error_m": round(pos_err, 4),
        "orientation_error_rad": ori_err,
        "moveit_error_code": code,
        "planning_time_s": 1.0,
    }


def client(land):
    c = object.__new__(MoveItClient)
    c.cfg = CFG
    c.calls = []

    def fake(side, xyz_world, quat, timeout_s, ptol, otol, vel, **kw):
        cmd = np.asarray(xyz_world, dtype=np.float64)
        c.calls.append((cmd.copy(), float(timeout_s)))
        return land(cmd)

    c._move_tcp_once = fake
    return c


def run(c, **kw):
    args = dict(timeout_s=15.0, position_tolerance=PTOL,
                orientation_tolerance=OTOL, start_xyz=T.copy())
    args.update(kw)
    return c.move_tcp("right", T.copy(), QUAT, **args)


def check(name, cond, info=""):
    assert cond, f"{name} FAILED {info}"
    print(f"  PASS {name} {info}")


def common_schema(res):
    for k in ("iterations", "attempts", "convergence_enabled",
              "convergence_stop_reason"):
        assert k in res, f"missing {k}"
    assert res["iterations"] == len(res["attempts"])
    for i, a in enumerate(res["attempts"]):
        assert a["attempt"] == i + 1
        assert "commanded_xyz" in a
        if a.get("final_xyz") is not None:
            assert "residual_m" in a and "residual_xyz_m" in a


# 1. constant 3 cm droop -> retry lands on T, success, 2 iters
c = client(lambda cmd: make_result(cmd - np.array([0.0, 0.0, 0.03]), cmd))
res = run(c)
common_schema(res)
check("const-droop converges", res["success"] and res["iterations"] == 2
      and res["convergence_stop_reason"] == "target_reached",
      res.get("convergence_stop_reason"))
check("const-droop residual~0", res["position_error_m"] <= PTOL,
      res.get("position_error_m"))
check("retry timeout capped", c.calls[1][1] <= 8.0, c.calls)
check("attempt1 recorded compensation",
      "compensation_xyz_m" in res["attempts"][0], res["attempts"][0])

# 2. droop grows with compensation offset -> integral action still converges
base_droop = 0.03


def growing(cmd):
    extra = 0.3 * float(np.linalg.norm(cmd - T))
    return make_result(cmd - np.array([0.0, 0.0, base_droop + extra]), cmd)


c = client(growing)
res = run(c)
common_schema(res)
check("growing-droop converges", res["success"]
      and res["convergence_stop_reason"] == "target_reached",
      (res["iterations"], res.get("position_error_m")))

# 3. hard stop: always lands at same point -> residual_not_decreasing
fixed_land = T - np.array([0.0, 0.0, 0.03])
c = client(lambda cmd: make_result(fixed_land, cmd))
res = run(c)
common_schema(res)
check("hard-stop diverges", not res["success"]
      and res["convergence_stop_reason"] == "residual_not_decreasing",
      res.get("convergence_stop_reason"))

# 4. MoveIt planning failure -> moveit_failed, single attempt
c = client(lambda cmd: {"success": False, "error": "规划失败",
                        "moveit_error_code": 99999})
res = run(c)
common_schema(res)
check("moveit-fail no retry", not res["success"] and res["iterations"] == 1
      and res["convergence_stop_reason"] == "moveit_failed",
      res.get("convergence_stop_reason"))

# 5. converge disabled -> single attempt
c = client(lambda cmd: make_result(cmd - np.array([0.0, 0.0, 0.03]), cmd))
res = run(c, converge_enabled=False)
common_schema(res)
check("disabled single-shot", not res["success"] and res["iterations"] == 1
      and res["convergence_stop_reason"] == "convergence_disabled"
      and res["convergence_enabled"] is False,
      res.get("convergence_stop_reason"))

# 6. lands first try -> target_reached, 1 iter
c = client(lambda cmd: make_result(cmd + np.array([0.001, 0.0, 0.0]), cmd))
res = run(c)
common_schema(res)
check("first-try success", res["success"] and res["iterations"] == 1
      and res["convergence_stop_reason"] == "target_reached",
      res.get("convergence_stop_reason"))

# 7. droop > max compensation -> oscillates -> residual_not_decreasing
c = client(lambda cmd: make_result(cmd - np.array([0.0, 0.0, 0.10]), cmd))
res = run(c)
common_schema(res)
check("over-cap stops", not res["success"] and res["convergence_stop_reason"]
      in ("residual_not_decreasing", "max_iters"),
      res.get("convergence_stop_reason"))
check("over-cap clamped", abs(c.calls[1][0][2] - (T[2] + 0.08)) < 1e-9,
      c.calls[1][0])

# 8. orientation error beyond tol -> orientation_error, no retry
c = client(lambda cmd: make_result(cmd - np.array([0.0, 0.0, 0.03]), cmd,
                                 ori_err=0.10))
res = run(c)
common_schema(res)
check("ori-fail no retry", not res["success"] and res["iterations"] == 1
      and res["convergence_stop_reason"] == "orientation_error",
      res.get("convergence_stop_reason"))

# 9. upward droop near min_z -> compensation_safety_limit
low_T = np.array([0.30, -0.05, 0.02])
c = client(lambda cmd: make_result(cmd + np.array([0.0, 0.0, 0.05]), cmd))
res = c.move_tcp("right", low_T.copy(), QUAT, timeout_s=15.0,
                 position_tolerance=PTOL, orientation_tolerance=OTOL,
                 start_xyz=low_T.copy())
common_schema(res)
check("min-z safety stop", not res["success"]
      and res["convergence_stop_reason"] == "compensation_safety_limit",
      res.get("convergence_stop_reason"))

# 10. reported errors always vs ORIGINAL target on converged result
c = client(lambda cmd: make_result(cmd - np.array([0.0, 0.0, 0.03]), cmd))
res = run(c)
a2 = res["attempts"][1]
check("final err vs original", res["position_error_m"] <= PTOL
      and a2["command_position_error_m"] > PTOL,
      (res["position_error_m"], a2.get("command_position_error_m")))

print("\nALL PASS")
