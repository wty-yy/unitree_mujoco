# Changelog

## v0.1.1 - 2026-11-25

1. Add Linux `PacedGlfwAdapter` (`glfwSwapInterval(0)` + `mjr_finish()` + `steady_clock` 60 Hz frame pacing, with `UNITREE_MUJOCO_GLX_VSYNC=1` fallback) to eliminate NVIDIA X11 double-VSync visualization stutters when running alongside CUDA/TensorRT inference
2. Support optional global (`motor_torque_scale`) and per-joint-group (`--hip-torque-scale`, `--thigh-torque-scale`, `--calf-torque-scale`) motor torque scaling after actuator command clipping
3. Add `PolicySelectionSubscriber` (`rt/go2/policy_selection`) to display the active router policy in the viewer overlay
4. Add `policy_height_map.h` and `camera_height_map.h` (with `camera_height_map_test` / `camera_height_map_probe`), configurable forward elevation shifting (`shift_elevation_forward`, `elevation_forward_offset`, `forward_elevation`), and custom viewer geometry rendering via `simulate_with_user_geoms.cc`
5. Add `climb.xml` (replacing `four_platforms.xml`), `mixed.xml`, `stones.xml`, `gap.xml`, and `lab.xml` scenes with shared terrain includes under `unitree_robots/go2/terrains/`, and add lower-calf collision cylinders in `go2.xml`

## v0.1.0 - 2026-10-17

1. Keep Go2 models, terrain scenes, and the C++ simulator; remove depth camera support and unrelated components
2. Support MuJoCo `3.16.0`, Go2 DDS communication, gamepad input, and the base velocity overlay
3. Add per-level terrain labels to `cross_stairs.xml` and `cross_slope.xml`
4. Add bilingual READMEs with build instructions, configuration options, and six optimized scene previews
5. Publish yaw-aligned relative elevation and an all-zero missing mask through `rt/go2/policy_height_map` and `rt/go2/policy_height_map_missing_mask` at `50 Hz`
6. Add finite headless simulation with `--headless --no-joystick --duration SECONDS`
7. Display valid policy height-map samples as green surface markers with `show_policy_height_map` and `--show-policy-height-map` controls
