# awesome-octave-robotics

[![License](https://img.shields.io/badge/license-BSD--3--Clause-blue.svg)](LICENSE)
[![GNU Octave](https://img.shields.io/badge/Octave-11.3%2B-green.svg)](https://www.gnu.org/software/octave/)
[![CI](https://github.com/Foadsf/awesome-octave-robotics/actions/workflows/ci.yml/badge.svg)](https://github.com/Foadsf/awesome-octave-robotics/actions)
[![Dependencies](https://img.shields.io/badge/dependencies-zero%20proprietary-brightgreen.svg)](#readme)

A complete, self-contained, natively executable **GNU Octave 11.3.0** replication of the
[awesome-matlab-robotics](https://github.com/liangfenggit/awesome-matlab-robotics)
collection.

Zero dependencies on MATLAB or Octave-Forge — every algorithm is pure Octave native code.
All execution is headless via `octave-cli.exe --no-gui --quiet`; figures are exported as real
PNG files using the gnuplot graphics toolkit.

---

## Attribution & clean-room notice

The category taxonomy (ground vehicles, manipulation, legged locomotion, robot modeling,
perception, mapping/SLAM, motion planning, motion control, UAV, marine AUV, automated
driving, plus simulators, ROS middleware and hardware connectivity) was inspired by
`mathworks/awesome-matlab-robotics`. However, **every line of code, every URDF asset, and
every icon in `awesome-octave-robotics` is an original clean-room BSD-3-Clause
implementation for GNU Octave**. None of it derives from MathWorks-licensed material, and
the repository carries no MathWorks `license.txt`. The 14 category icons under `images/`
were generated natively in Octave (`scripts/generate_category_icons.m`) and contain no
third-party assets.

---

## Build / run

```powershell
git clone https://github.com/Foadsf/awesome-octave-robotics.git
cd awesome-octave-robotics
run('setup_paths.m')
```

The octave runner:

```
octave-cli --no-gui --quiet
```

Run a single demo or test:

```
octave-cli --no-gui --quiet --eval "run('setup_paths.m'); run('demos/09_uav.m');"
octave-cli --no-gui --quiet --eval "run('setup_paths.m'); run('tests/test_core.m');"
```

## Unified runners (Phase 5)

- `tests/run_all_tests.m` — runs every test file in a fresh headless octave process;
  exit code 0 iff all pass.
- `demos/run_all_demos.m` — runs every demo (01–14) in a fresh headless octave process
  and confirms each writes its figure PNG; exit code 0 iff all pass.

```powershell
octave-cli --no-gui --quiet --eval "run('tests/run_all_tests.m');"
octave-cli --no-gui --quiet --eval "run('demos/run_all_demos.m');"
```

## Coverage

**11 application areas** (`src/domains/*`):

| # | Area | Module | Demo |
|---|------|--------|------|
| 01 | Ground vehicles | `unicycle_step.m`, `pure_pursuit_steer.m` | `demos/01_ground_vehicle.m` |
| 02 | Manipulation | `fk_2link.m`, `ik_2link.m`, `jacobian_2link.m` | `demos/02_manipulation.m` |
| 03 | Legged locomotion | `lipm.m`, `zmp_cop.m` | `demos/03_legged_locomotion.m` |
| 04 | Robot modeling | `urdf_parse.m`, `urdf_fk.m` (+ `assets/urdf/`) | `demos/04_robot_modeling.m` |
| 05 | Perception | `complementary_filter.m` | `demos/05_perception.m` |
| 06 | Mapping / SLAM | `icp.m`, `occupancy_grid.m` | `demos/06_mapping_slam.m` |
| 07 | Motion planning | `rrt.m` | `demos/07_motion_planning.m` |
| 08 | Motion control | `lqr_track.m` | `demos/08_motion_control.m` |
| 09 | UAV | `mixer.m` | `demos/09_uav.m` |
| 10 | Marine AUV | `los_guidance.m` | `demos/10_marine_auv.m` |
| 11 | Automated driving | `stanley.m` | `demos/11_automated_driving.m` |

**3 common tool categories** (`src/tools/*`):

| # | Category | Module | Demo |
|---|----------|--------|------|
| 12 | Simulators | `sim_discrete.m` | `demos/12_simulators.m` |
| 13 | ROS middleware | `ros_serialise.m`, `ros_deserialise.m` | `demos/13_ros_middleware.m` |
| 14 | Hardware connectivity | `crc8.m`, `serial_packet.m`, `serial_parse.m` | `demos/14_hardware_connectivity.m` |

**Core kernel** (`src/core/*`): `so3.m`, `se2.m`, `se3.m`, `quat_ops.m`, `dual_quat.m`,
`ode_rk4.m`, `semi_implicit_euler.m`, `dare_lqr.m`, `care_lqr.m`, `qp_solve.m`,
`plot_helpers.m`.

## Verification gallery

All demos run headless and write real PNG figures to `output/figures/`:

| Manipulation IK trajectory | 2D SLAM pose graph | Motion planning tree |
|---|---|---|
| `output/figures/02_manipulation.png` | `output/figures/06_mapping_slam.png` | `output/figures/07_motion_planning.png` |

| UAV mixer thrust allocation | AUV LOS track following | Automated driving Stanley |
|---|---|---|
| `output/figures/09_uav.png` | `output/figures/10_marine_auv.png` | `output/figures/11_automated_driving.png` |

## Verification

`tests/test_core.m` (29 checks), `tests/test_batchA.m` (20), `tests/test_batchB.m` (19),
`tests/test_batchC.m` (20). Every check is a real numerical assertion (no stubs/TODOs);
every demo writes a real PNG to `output/figures/`. Full suite exit code 0.

See `PLAN.md` for the 6-phase build protocol and the complete file inventory.

## License

BSD-3-Clause (see `LICENSE`).
