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
3. Check the tool result's `success` field and MoveIt or controller return code.
   A log line or accepted goal is not completion.
4. After a successful action, verify the reported final TCP or joint state;
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
