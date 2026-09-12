# CHANGELOG

## v0.1.4 2026-09-12
1. 新增 [depth_camera_publisher.cc](simulate/src/depth_camera_publisher.cc)，将 `d435i` 原始深度以 `32FC1` 经 DDS 话题 `rt/front_depth_observation` 发布，默认 `640x360`、`15 Hz`，可用 OpenCV 灰度窗口预览（`camera.show_depth`、`camera.display_max_depth`）
2. 新增 [d435i.xml](unitree_robots/go2/d435i.xml) 相机附件与 mesh，加载 go2 场景时自动挂载到 `base_link`，深度渲染中过滤机器人本体几何
3. 新增 go2 [four_platforms.xml](unitree_robots/go2/four_platforms.xml) 场景，起点四周平台高度分别为 `0.30`/`0.40`/`0.50`/`0.60 m`
4. [config.yaml](simulate/config.yaml) 新增 `camera` 配置段（`enabled`/`model`/`name`/`topic`/`width`/`height`/`publish_hz`/`show_depth`/`display_max_depth`），默认 `robot`/`robot_scene` 改为 `go2` + `four_platforms.xml`，`enable_elastic_band` 改为 `0`
5. 相机启用时使用隐藏离屏 GLFW 窗口渲染深度，并禁用模型拖放/重载
6. [CMakeLists.txt](simulate/CMakeLists.txt) 增加 OpenCV `core`/`imgproc`/`highgui` 依赖
7. 修复 `jstest` 缺少 `<cstdint>` 的编译错误
8. 将 `UPDATE.md` 重命名为 `CHANGELOG.md`
9. `.gitignore` 增加 `MUJOCO_LOG.TXT`

## v0.1.3 2026-03-05
1. 将g1的手部重新加上碰撞mesh

## v0.1.2 2026-03-01
1. 新增了默认相机朝向base_link (go2)或torso_link (g1)
2. 修改flat.yaml中天空为蓝色 (原来为黑色)

## v0.1.1 2026-02-24
1. g1添加flat, cross_slope, cross_stairs, race_track, stairs场景
2. 在g1_29dof配置中加入对足部摩擦的设置，目前设置为0.8

## v0.1.0 2026-02-10
1. 修复默认mujoco按键无法使用的bug，例如tab,shift+tab将侧栏折叠
2. 默认的domain_id设置为0
3. `view->cam`自动追踪base_link
4. 加入更多go2场景
5. 将go2足部摩擦改为0.8，适配全部地形难度
