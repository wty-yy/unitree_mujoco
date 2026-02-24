# UPDATE
## 20260224
1. g1添加flat, cross_slope, cross_stairs, race_track, stairs场景
2. 在g1_29dof配置中加入对足部摩擦的设置，目前设置为0.8
## 20260210
1. 修复默认mujoco按键无法使用的bug，例如tab,shift+tab将侧栏折叠
2. 默认的domain_id设置为0
3. `view->cam`自动追踪base_link
4. 加入更多go2场景
5. 将go2足部摩擦改为0.8，适配全部地形难度
