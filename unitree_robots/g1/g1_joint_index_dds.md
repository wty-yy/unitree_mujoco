# G1 29DOF Mode 15 仿真关节索引

`LowCmd_.motor_cmd`、`LowState_.motor_state` 和 MJCF 执行器使用相同索引。消息命名空间为 `unitree_hg::msg::dds_`，控制模式为 `mode_pr = 0`（关节角模式），`mode_machine = 5`。

| 索引 | MJCF 执行器 |
| --- | --- |
| 0 | `left_hip_pitch` |
| 1 | `left_hip_roll` |
| 2 | `left_hip_yaw` |
| 3 | `left_knee` |
| 4 | `left_ankle_pitch` |
| 5 | `left_ankle_roll` |
| 6 | `right_hip_pitch` |
| 7 | `right_hip_roll` |
| 8 | `right_hip_yaw` |
| 9 | `right_knee` |
| 10 | `right_ankle_pitch` |
| 11 | `right_ankle_roll` |
| 12 | `waist_yaw` |
| 13 | `waist_roll` |
| 14 | `waist_pitch` |
| 15 | `left_shoulder_pitch` |
| 16 | `left_shoulder_roll` |
| 17 | `left_shoulder_yaw` |
| 18 | `left_elbow` |
| 19 | `left_wrist_roll` |
| 20 | `left_wrist_pitch` |
| 21 | `left_wrist_yaw` |
| 22 | `right_shoulder_pitch` |
| 23 | `right_shoulder_roll` |
| 24 | `right_shoulder_yaw` |
| 25 | `right_elbow` |
| 26 | `right_wrist_roll` |
| 27 | `right_wrist_pitch` |
| 28 | `right_wrist_yaw` |
