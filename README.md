# iCAN 射击练习仿真初始工作空间

这是发给参赛用户的练习仿真工作空间，包含仿真环境安装包、说明文档、一键脚本和示例 ROS 2 源码。

## 目录

```text
ican_shoot_user_workspace_example_only/
  src/
    ican_example_navigation/
    ican_example_ring_aim/
    ican_example_mission/
  packages/
    ican-shoot-sim-practice_0.1.0_amd64.deb
    SHA256SUMS
  scripts/
    00_install_ros2_gazebo.sh
    01_install_sim_deb.sh
    setup_env.sh
    run_sim.sh
    run_example_mission.sh
    check_sim_env.sh
  docs/
    快速开始.md
    仿真环境与接口说明.md
```

## 推荐系统

- Ubuntu 22.04 LTS amd64
- ROS 2 Humble
- Gazebo Classic

## 首次安装

进入本目录后执行：

```bash
bash scripts/00_install_ros2_gazebo.sh
bash scripts/01_install_sim_deb.sh
```

安装完成后打开一个新终端，或在当前终端执行：

```bash
source scripts/setup_env.sh
```

## 启动仿真

```bash
bash scripts/run_sim.sh
```

无图形界面或远程服务器可用：

```bash
bash scripts/run_sim.sh gui:=false
```

## 检查环境

```bash
bash scripts/check_sim_env.sh
```

## 示例代码

`ican_shoot_user_workspace_example_only` 本身就是示例代码的 colcon 工作空间；首次运行或修改源码后，先在本目录构建：

```bash
colcon build --symlink-install
source install/setup.bash
```

启动示例流程：

```bash
bash scripts/run_example_mission.sh
```

脚本会打开三个终端标签，分别启动仿真、Nav2 导航和完整任务程序。启动器会先等待 Gazebo `/clock` 连续稳定，再启动 Nav2；只有关键 Nav2 生命周期节点全部为 `active` 且 `/navigate_to_pose` 只有一个 action server 后，才会启动完整任务。这样可以避免 Gazebo 加载期间时钟回跳或 Nav2 尚未就绪造成“目标已发送但机器人不走”。请在 `example` 标签页按提示输入 2/3/4 号靶配置。机器人将依次导航到四个任务点，每次射击前发布对应的 `shoot_1`～`shoot_4` 裁判消息：1 号点使用相机检测环形靶，2 号点跟踪指定旋转叶片，3/4 号点跟踪指定移动靶区域；四次射击结束后自动返回指挥中心。

也可以不使用交互提示，直接指定本轮目标：

```bash
ros2 launch ican_example_mission full_mission.launch.py \
  prompt_target_config:=false target_2_id:=1 target_3_id:=6 target_4_id:=8
```

完整任务节点针对本练习仿真订阅 `/gazebo/model_states` 跟踪动态靶。为避免流程卡死，默认瞄准超时后仍会射击并继续下一点；如需严格禁止未对准射击，可传入 `fire_without_aim:=false`。

完整任务启动时也会启动裁判计分节点。实时查看总分、计分明细和事件：

```bash
ros2 topic echo /referee/score
ros2 topic echo /referee/score_detail
ros2 topic echo /referee/events
```

重新开始一轮任务前可清零计分：

```bash
ros2 service call /referee/reset std_srvs/srv/Trigger {}
```

示例导航仍然调用 ROS 2 官方 `nav2_bringup`，但默认使用适配当前练习场地和麦轮小车的参数：

```text
src/ican_example_navigation/config/nav2_params.yaml
```

如需对比 ROS 2 官方默认参数，可启动时传入：

```bash
ros2 launch ican_example_navigation navigation.launch.py params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```

RViz 中很多绿色小箭头通常是 AMCL 粒子云显示。当前示例默认使用 `src/ican_example_navigation/rviz/example_nav.rviz`，不显示粒子云。

为避免虚拟机中 RViz 抢占 Nav2 控制循环，示例默认不启动 RViz。需要调试地图时可手动启用：

```bash
ros2 launch ican_example_navigation navigation.launch.py rviz:=true
```

更多接口说明见 [docs/仿真环境与接口说明.md](docs/仿真环境与接口说明.md)。
三、陆地障碍物清除任务
获取障碍物精准位置后，开展定向清除作业，打通救援通道，
保障救援人员安全进入核心受困区域开展施救。
（一）比赛场地
障碍物清除任务点（陆地障碍物清除任务） 障碍物标靶位置（陆地障碍物清除任务）
指挥中心（陆地障碍物清除任务）
比赛场地为长宽高 2.7m×4m×0.4m。
场地设置指挥中心区域，尺寸为 40×55cm。障碍物清除任
务点尺寸为 40×55cm。
11
1 号障碍物清除标靶示意图 2 号障碍物清除标靶示意图
3 号、4 号障碍物清除标靶示意图
2 号障碍物清除标靶靶位二维码为 1-5 号图片中二维码
3 号、4 号障碍物清除标靶靶位二维码为 6-8 号图片中二维码
（需将二维码依次张贴在标靶靶位正上方）
1 2 3 4
5 6 7 8
12
（二）比赛任务
任务一 从指挥中心出发，到达第 1 个障碍物清除任务点得 10 分。
射击前方①号环形障碍物目标（6分/7分/8分/9分/10分），
击中几环得几分。
任务二 到达第 2 个障碍物清除任务点得 10 分，击中前方指定②
号旋转障碍物目标得 10 分。
任务三 到达第 3 个障碍物清除任务点得 10 分，击中前方指定③
号移动障碍物目标得 10 分。
任务四 到达第 4 个障碍物清除任务点得 10 分，击中前方指定④
号移动障碍物目标得 10 分。
任务五 障碍物清除任务完成成功回到指挥中心得 10 分。
