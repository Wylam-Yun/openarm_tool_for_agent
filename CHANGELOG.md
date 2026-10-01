# Changelog

## 2026-09-30 - Add OpenArm agent skill

- Added a repository-level `SKILL.md` with explicit motion, gripper, reset, perception, and recovery boundaries.
- Added progressive references for tool schemas and safety verification.

## 2026-09-30 - Separate gripper actions and add safety reset

- Removed gripper side effects from `move_to`, `move_delta`, and `rotate_pitch`.
- Added `reset_arm` to move one or both arms to the configured `hands_up` safety posture and verify final joint error.
- Kept `set_gripper`, `release`, and `scripted_grasp` as the explicit gripper and composite-action tools.

## 2026-09-30 - Simplified agent-facing parameters

- Removed unused movement parameters (`step_clip`, `max_steps`, `n`) and exposed precision settings from the agent-facing tools.
- Moved position and orientation tolerances into `arm_tools/config.yaml` while retaining final TCP error verification.
- Replaced numeric gripper commands and interpolation controls with `state=open/close`; `release` remains the explicit open shortcut.
- Changed gripper execution to one action goal and documented all interactive shell parameters, reducing avoidable command latency.

## 2026-09-30 - Initial arm_tools baseline

- Added the standalone agent control tools and interactive shell for OpenArm.
- Reused the existing ROS 2, MoveIt, joint trajectory controller, and gripper action paths.
- Added world-frame motion tools, TCP verification, pitch control, gripper control, perception helpers, and safety checks.
- Added explicit MoveIt planning and execution error explanations for agent callers.
