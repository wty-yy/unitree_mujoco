# UPDATE

## v0.1.1 - 2026-11-25

1. Add Linux `PacedGlfwAdapter` (`glfwSwapInterval(0)` + `mjr_finish()` + `steady_clock` 60 Hz frame pacing, with `UNITREE_MUJOCO_GLX_VSYNC=1` fallback) to eliminate NVIDIA X11 double-VSync visualization stutters when running alongside CUDA/TensorRT inference
2. Add `PolicySelectionSubscriber` (`rt/g1/policy_selection`) to display the dominant MoE expert and per-expert weights in the viewer overlay, and synchronize overlay handoff under `Simulate::mtx` in `simulate_with_user_geoms.cc`
3. Exclude display group `4` (lab roof) from policy height-map terrain raycasts in `simulate/src/height_map.h`
4. Add `mixed.xml`, `stones.xml`, `gap.xml`, and `lab.xml` scenes with shared terrain includes under `unitree_robots/g1/terrains/` (`climb_boxes.xml`, `stones.xml`, `gap.xml`, `gap_rings.xml`, `gap_ring_meshes.xml`)

## v0.1 - 2026-11-13

1. Replace the Mode 15 model with `g1_29dof_mode_15_unitree_model_collision.xml` and its matching URDF, providing 34 visual geometries and 33 collision geometries (`15` mesh, `10` sphere, `6` cylinder, `2` box) in display group `3`
2. Remove the shoes variant, `complete-shoes` model selection, the legacy SRDF files, and the `collision_*.STL` meshes
3. Add 15 URDF-derived collision meshes and point `flat.xml`, `stairs.xml`, `cross_stairs.xml`, `cross_slope.xml`, `race_track.xml`, and `scene_29dof.xml` to the new model
4. Add `simulate/src/height_map.h`: yaw-aligned `17×11` terrain sampling grid with `0.1` m resolution, relative heights clipped to `±2` m, and world-welded geometry only
5. Publish `rt/g1/policy_height_map` and `rt/g1/policy_height_map_missing_mask` (`unitree_go::msg::dds_::HeightMap_`) at `15` Hz
6. Add options `--show-policy-height-map` and `--height-map-offset-x`, exposed as `show_policy_height_map` and `height_map_offset_x` in `config.yaml`
7. Add the base-frame velocity overlay (`vx`, `vy`, `wz`) refreshed every `0.1` s, controlled by `show_base_velocity`
8. Add `--waist-pitch-torque-scale` to scale the final `waist_pitch` torque in `[0, 1]` after motor saturation
9. Extend the MuJoCo viewer through a generated `simulate_with_user_geoms.cc`, copying `user_scn_geoms_` into the managed scene
10. Add `g1_height_map_test` and rewrite `g1_model_selection_test` to compare collision type, pose, size, friction, and mesh topology against the source URDF
11. Add height reference sites in `stairs.xml` for `5cm` to `38cm` step levels
12. Rename `UPDATE.md` to `CHANGELOG.md`

## 20260305

1. 将g1的手部重新加上碰撞mesh

## 20260301

1. 新增了默认相机朝向base_link (go2)或torso_link (g1)
2. 修改flat.yaml中天空为蓝色 (原来为黑色)

## 20260224

1. g1添加flat, cross_slope, cross_stairs, race_track, stairs场景
2. 在g1_29dof配置中加入对足部摩擦的设置，目前设置为0.8

## 20260210

1. 修复默认mujoco按键无法使用的bug，例如tab,shift+tab将侧栏折叠
2. 默认的domain_id设置为0
3. `view->cam`自动追踪base_link
4. 加入更多go2场景
5. 将go2足部摩擦改为0.8，适配全部地形难度
