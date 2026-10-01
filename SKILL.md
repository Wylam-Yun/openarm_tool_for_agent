---
name: openarm-agent
description: 控制 RBM OpenArm 双臂机器人，执行移动、夹爪、复位和环境感知动作；当任务需要真实机器人动作或读取其状态时使用。
---

# OpenArm Agent

Use this skill when a task requires the OpenArm robot to move, grasp, release,
return to a safe posture, or inspect its cameras and world state.

## Operating sequence

1. Read the current state with `view_env_state` before choosing a target. Treat
   its returned TCP poses, joint state, and image timestamps as the starting
   state for the action.
2. Choose one explicit capability: arm motion, gripper action, or reset. Keep
   gripper state outside motion calls.
3. Before a multi-stage physical task, form the phase sequence first (for
   example: hover, descend, close, lift), then execute one phase at a time and
   verify the external state after each phase. For a known free-space target,
   use one `move_to` rather than manufacturing 2-3 cm micro-steps.
4. Check the tool result's `success` field and MoveIt or controller return code.
   A log line or accepted goal is not completion.
5. After a successful action, verify the reported final TCP or joint state;
   use `view_env_state` again when the next decision depends on live state.

## Capability boundaries

- `move_to`: move one arm's `hand_tcp` to an absolute `world` position while
  keeping its current orientation.
- `move_delta`: read the current TCP once, add a `world` displacement, then use
  the shared `move_to` path.
- `rotate_pitch`: keep TCP position and set an absolute pitch angle.
- `set_gripper`: open or close one gripper in place. Use `state=open|close`.
- `release`: the explicit open shortcut for one gripper.
- `scripted_grasp`: the fixed open, hover, descend, close, lift sequence.
- `reset_arm`: move `left`, `right`, or `both` arms to the configured
  `hands_up` posture. It leaves grippers unchanged and verifies final joint
  error.
- `back_project_batch` and `query_world_map`: perception helpers; they do not
  move the robot.

## Motion convergence

`move_to` and `rotate_pitch` run through the shared MoveIt client. When MoveIt
succeeds but the measured position is outside tolerance, the client may
run a bounded position-only correction: it keeps the original target fixed,
measures the real TCP, and adds the latest residual to the compensated command
(bounded integral correction, not a fresh retry of the same target).
It never retries a planning failure or an orientation failure. The correction
stops when the residual grows, a safety limit is reached, or the configured
iteration count is exhausted. The JSON result includes `iterations`,
`attempts`, and `convergence_stop_reason`.

The caller's `timeout_s` remains the timeout for the primary action. Correction
rounds use the internal `converge_retry_timeout_s` limit and do not consume the
timeouts of later phases. `move_delta` and the `scripted_grasp` hover, descend,
and lift phases disable correction: contact can look like a position residual,
and `move_delta` already self-corrects by re-reading the TCP on every call.

Argument schemas and result examples are in
[`references/tools.md`](references/tools.md). Safety and recovery rules are in
[`references/safety.md`](references/safety.md); read them before the first
physical action and whenever an action fails.

## Recovery

When a motion fails or the arm must be put in a known configuration, inspect
the failure result and call `reset_arm` for the affected arm. A reset is a
planned MoveIt motion and requires the same final-state verification as any
other motion. For immediate physical danger, use the robot's hardware
emergency-stop control.
