# Headless native integration smoke result

Date: 2026-09-09 (Asia/Shanghai). Executed on `ssh bling`, inside the existing
`comedy` ROS1 Docker container. No source edits, user images, or user calibration
results were used. The container was left running.

## Environment and fixture

- Binary: `/root/chenyu/ws_chenyu/recon/handhold-recon/direct_visual_lidar_calibration_ws/devel/lib/direct_visual_lidar_calibration/calibrate`
- Environment: sourced `/opt/ros/noetic/setup.bash` and workspace `devel/setup.bash`,
  then `unset DISPLAY` and `export OMP_NUM_THREADS=2`.
- Temporary artifacts: `/tmp/dual_mei_headless_smoke.C0sfLs` inside `comedy`.
- The task's Python `prepare_camera()` generated the native `000000.png`, binary
  little-endian XYZI `000000.ply`, `calib.json`, and relative `mask.png` input.
  The current local Python scripts were copied into the isolated test directory's
  `python_current/` before the final run, without editing the remote source tree.
- Entirely synthetic 160 × 160 sinusoidal grayscale texture, circular image mask,
  and 1,499 corresponding points at varied ranges. MEI parameters were
  `[fx,fy,cx,cy,xi]=[60,60,79.5,79.5,1]`, five zero distortion coefficients, and
  `max_theta_deg=95`. The synthetic true and initial transforms were identity.
- Every case initially contained a stale successful `T_lidar_camera` with
  translation `[9,9,9]`, to verify replacement or removal.

## Commands and results

Each command below used the binary above and `--headless`. Paths are relative to
the temporary artifact directory.

| Case / additional arguments | Exit | Saved status | Result |
| --- | ---: | --- | --- |
| `bfgs --max_outer_iterations 10 --max_inner_iterations 256` | 0 | `success` | Inner/outer converged; 11 inner iterations; NID 0.42809782 → 0.40720964; finite transform replaced stale output |
| `nelder_mead --registration_type nid_nelder_mead --max_outer_iterations 10 --max_inner_iterations 256` | 0 | `success` | Inner/outer converged; 20 inner iterations; NID 0.29436180 → 0.29407674; finite transform replaced stale output |
| `nonconverged --registration_type nid_nelder_mead --max_outer_iterations 1 --max_inner_iterations 1` | 1 | `failed` | Reports iteration limit / no convergence; stale `T_lidar_camera` removed |
| `no_overlap --max_outer_iterations 1 --max_inner_iterations 1` | 1 | `failed` | Rear-facing points rejected; visible count 0; reports no image/LiDAR overlap; stale output removed |
| `invalid_initial --max_outer_iterations 1 --max_inner_iterations 1` | 1 | `failed` | Zero quaternion rejected as invalid norm; stale output removed |

All five processes finished with `DISPLAY` absent. No GLFW, X11 display, or OpenGL
initialization errors appeared. Both normal optimization paths completed without
a display; no relaxed convergence threshold was used for the successful cases.

An assertion pass checked all exit codes, saved statuses, convergence flags,
finite successful matrices, nonincreasing costs, stale-result removal, and the
relative mask path. The task's Python `read_calibration_result()` accepted both
successful native outputs, converted them to LiDAR → camera, and rejected all
three failed outputs.

Raw commands, saved diagnostics, elapsed times, and transforms are retained in
`results.json`; individual process output is in `<case>.log` at the temporary
artifact directory. These are smoke fixtures, not calibration accuracy evidence
for the real device. Real images and final extrinsics remain for user validation.
