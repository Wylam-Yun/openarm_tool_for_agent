# Changelog

## 2026-10-01 - Add bounded TCP convergence and motion planning guidance

- Added accumulated position-residual compensation after successful MoveIt
  motions, with fixed original targets, a capped total compensation, safety
  bounds, divergence detection, per-attempt telemetry, and short retry timeouts.
- Set motion defaults to 15 seconds, kept correction retries at 8 seconds, and
  disabled correction in `move_delta` and contact-prone `scripted_grasp` phases.
- Documented phase-first planning and the rule to avoid artificial micro-steps
  for known free-space targets.

## 2026-10-01 - 收拢为标准 skill 文件夹

- 把散落根目录的 `SKILL.md`、`CHANGELOG.md`、`arm_tools.sh`、`arm_tools/`、`references/` 收拢进 `openarm-agent/`，skill 自包含。
- 删除 `.opencode/tools/*.ts`（共 10 个）：Python 后端可直调，不再需要 TS 注册层；同步清理残留 `.opencode/` 目录。
- 更新 `arm_tools/README.md` 的目录树与运行说明，修正 `.gitignore` 的 `__pycache__`/`snapshots` 路径。

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
