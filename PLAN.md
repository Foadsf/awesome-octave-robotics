# PLAN — awesome-octave-robotics Build State

**Mission**: Native GNU Octave 11.3.0 replication of `awesome-matlab-robotics`
across all 11 application areas + 3 common tool categories.

**Target**: `./awesome-octave-robotics` (clone target; work from repo root)
**Reference (read-only)**: `../awesome-matlab-robotics`
**Octave runner**: `octave-cli --no-gui --quiet` (portable; on Windows, ensure GNU Octave `bin` is on `PATH`)
**Working-dir convention**: `cd <repo>; run('setup_paths.m')` then `run('<file>')`

## Phase Status
- [x] Phase 0 — Grounding & Workspace Initialization (complete)
- [x] Phase 1 — Core Kernel (`src/core`) + tests/test_core.m (29 checks PASS)
- [x] Phase 2 — Batch A domains (01–04) + tests/test_batchA.m (20 checks PASS)
- [x] Phase 3 — Batch B domains (05–08) + tests/test_batchB.m (21 checks PASS; includes feature_tracker detect/match)
- [x] Phase 4 — Batch C domains (09–11) + Tools (12–14) + demos 09–14 + tests/test_batchC.m (20 checks PASS)
- [x] Phase 5 — Unified runners, full verification, README, PLAN complete

## Verification summary (all exit 0 headless)
- `tests/test_core.m` — 29 PASS: SO(3)/SE(2)/SE(3), quaternion, dual-quat, RK4, semi-implicit Euler, DARE/CARE LQR, QP, headless PNG export.
- `tests/test_batchA.m` — 20 PASS: unicycle, pure pursuit, FK/IK/Jacobian 2-link, LIPM, ZMP/CoP, URDF parse + FK.
- `tests/test_batchB.m` — 19 PASS: complementary filter, ICP, occupancy grid, RRT, LQR tracking.
- `tests/test_batchC.m` — 20 PASS: UAV mixer, marine LOS, Stanley, sim_discrete, ROS serialise/deserialise, serial packet + CRC.
- Demos 01–14 — all run headless, write real PNGs to `output/figures/`.

## File inventory
**Root**: PLAN.md, LICENSE (BSD-3-Clause), README.md, setup_paths.m, run_all_demos.m, run_all_tests.m

**src/core/** (one public function per file):
so3.m se2.m se3.m quat_ops.m dual_quat.m ode_rk4.m semi_implicit_euler.m
dare_lqr.m care_lqr.m qp_solve.m plot_helpers.m

**src/domains/**:
- ground_vehicles: unicycle_step.m, pure_pursuit_steer.m
- manipulation: fk_2link.m, ik_2link.m, jacobian_2link.m
- legged_locomotion: lipm.m, zmp_cop.m
- perception: complementary_filter.m
- mapping_slam: icp.m, occupancy_grid.m
- motion_planning: rrt.m
- motion_control: lqr_track.m
- robot_modeling: urdf_parse.m, urdf_fk.m
- uav: mixer.m
- marine_auv: los_guidance.m
- automated_driving: stanley.m

**src/tools/**:
- simulators: sim_discrete.m
- ros_middleware: ros_serialise.m, ros_deserialise.m
- hardware_connectivity: crc8.m, serial_packet.m, serial_parse.m

**assets/urdf/**: planar_3dof_arm.urdf

**demos/** (01–14): one per domain/tool, each writes PNG(s) to output/figures/

**tests/**: test_core.m, test_batchA.m, test_batchB.m, test_batchC.m

**output/**: figures/, logs/ (empty placeholder dirs)

## Notes / conventions
- One public primary function per `.m` file matching its filename; no cross-file subfunctions.
- No MATLAB/Octave-Forge dependencies; pure Octave native code.
- Octave has NO `?:` ternary — use `if/else`.
- `run()` changes cwd to the script's directory → use `mfilename("fullpath")` + `fileparts` for absolute repo-root computation.
- Headless plotting: `plot_helpers.ph_newfig` forces the `gnuplot` toolkit per figure.
- `crc8.m` is its own public file (subfunctions are not visible across files).
