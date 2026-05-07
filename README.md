2026.5.7

当前 `final_feature` 默认启用完整系统：Zero-DCE++ ONNX C++ 低照增强、Depth-to-map 约束、Voxblox 稠密建图。不要用 `sudo` 进容器。

## 快速开始

先在宿主机编译镜像并进入容器。`run_container.sh` 的参数是要挂载到 `/data/` 的 bag 文件。

```bash
cd /home/mhenwa/slam/VINS-RGBD
docker build -t vins-rgbd:melodic -f docker/Dockerfile .
./docker/run_container.sh /home/mhenwa/slam/bags/Normal.bag
```

容器内编译工作空间：

```bash
./docker/build_in_container.sh
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
```

运行输出统一写到仓库根目录的 `output/`：

- `output/vins/`：`vins_result_no_loop.csv`、`vins_result_loop.csv` 和外参结果
- `output/eval/`：轨迹评估指标、对齐轨迹和误差图
- `output/plots/`：只画运行轨迹的图
- `output/gt/`：GT 提取与可视化结果
- `output/pose_graph/`：pose graph 保存/加载目录
- `output/pcd/`：pose graph 按键导出的 PCD
- `output/voxblox/`：Voxblox `map.vxblx` 和 `mesh.ply`
- `output/ablation/`：自动消融评测结果

默认 launch 参数已经全部打开：

- `use_zero_dce:=true`
- `zero_dce_use_onnx:=true`
- `use_depth_to_map:=1`
- `use_depth_to_map_pose_graph:=1`

需要关模块时直接覆盖对应参数。例如纯原版前端：

```bash
roslaunch vins_estimator realsense_color.launch \
  use_zero_dce:=false \
  use_depth_to_map:=0 \
  use_depth_to_map_pose_graph:=0
```

## Release / Normal.bag

`Normal.bag` 代表 Release/RealSense 数据，默认使用 `config/realsense/realsense_color_config.yaml` 和 `config/realsense/realsense_depth_config.yaml`。

终端 1，启动完整系统：

```bash
cd /workspace/VINS-RGBD
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
roslaunch vins_estimator realsense_color.launch
```

终端 2，打开 RViz。这个配置会同时显示当前输入画面和 Zero-DCE++ 增强后画面：

```bash
docker exec -it vins-rgbd bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
roslaunch vins_estimator vins_rviz.launch \
  rviz_config:=/workspace/VINS-RGBD/config/zero_dce_compare.rviz
```

终端 3，播放 `Normal.bag`：

```bash
docker exec -it vins-rgbd bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
rosbag play /data/Normal.bag
```

正常 `Ctrl+C` 退出算法终端会自动保存：

- `output/voxblox/map.vxblx`
- `output/voxblox/mesh.ply`
- `output/vins/vins_result_loop.csv`

打开 mesh：

```bash
meshlab output/voxblox/mesh.ply
```

`Normal.bag` 可从 bag 内 `/vrpn_client_node/jackal/pose` 提取 GT。提取脚本依赖 ROS/rosbag，先在容器内执行：

```bash
docker exec -it vins-rgbd bash
source /opt/ros/melodic/setup.bash
cd /workspace/VINS-RGBD
python tools/extract_pose_stamped_gt.py \
  --bag /data/Normal.bag \
  --topic /vrpn_client_node/jackal/pose \
  --out output/gt/Normal_vrpn_gt.txt
```

然后在宿主机用通用轨迹评估工具对比：

```bash
cd /home/mhenwa/slam/VINS-RGBD
python3 tools/eval_trajectory.py \
  --est output/vins/vins_result_loop.csv \
  --gt output/gt/Normal_vrpn_gt.txt \
  --out-dir output/eval/normal \
  --name normal_full
```

## Ground-Challenge / darkroom1.bag

`darkroom1.bag` 代表 Ground-Challenge 数据，使用 `config/ground_challenge/groundchallenge_config.yaml` 和 `config/ground_challenge/groundchallenge_depth_config.yaml`。

先用 darkroom bag 启动容器：

```bash
cd /home/mhenwa/slam/VINS-RGBD
./docker/run_container.sh /home/mhenwa/slam/bags/darkroom1.bag
```

终端 1，启动完整系统：

```bash
cd /workspace/VINS-RGBD
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
roslaunch vins_estimator realsense_color.launch \
  config_path:=/workspace/VINS-RGBD/config/ground_challenge/groundchallenge_config.yaml \
  depth_config_path:=/workspace/VINS-RGBD/config/ground_challenge/groundchallenge_depth_config.yaml
```

终端 2，打开 RViz：

```bash
docker exec -it vins-rgbd bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
roslaunch vins_estimator vins_rviz.launch \
  rviz_config:=/workspace/VINS-RGBD/config/zero_dce_compare.rviz
```

终端 3，播放 `darkroom1.bag`：

```bash
docker exec -it vins-rgbd bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
rosbag play /data/darkroom1.bag
```

用 Ground-Challenge pseudo GT 评估：

```bash
python3 tools/eval_trajectory.py \
  --est output/vins/vins_result_loop.csv \
  --gt /home/mhenwa/slam/Ground-Challenge/psudo_gt/darkroom1.txt \
  --out-dir output/eval/darkroom1 \
  --name darkroom1_full
```

也可以单独画 pseudo GT：

```bash
python3 tools/plot_ground_challenge_gt.py \
  --gt /home/mhenwa/slam/Ground-Challenge/psudo_gt/darkroom1.txt \
  --out-dir output/gt/darkroom1 \
  --name darkroom1_gt
```

## Voxblox 建图

默认会在 `pose_graph` 后端内嵌 Voxblox，使用关键帧深度采样生成 TSDF/ESDF。关键帧深度点会从同步彩色图像采样 RGB；legacy `/pose_graph/octree`、按键保存的 PCD、Voxblox mesh 和 `mesh.ply` 都保留颜色。

相关话题：

- `/pose_graph/voxblox/mesh`
- `/pose_graph/voxblox/surface_pointcloud`
- `/pose_graph/voxblox/tsdf_pointcloud`
- `/pose_graph/voxblox/esdf_pointcloud`
- `/pose_graph/voxblox/tsdf_slice`
- `/pose_graph/voxblox/esdf_slice`
- `/pose_graph/voxblox/esdf_map_out`

参数在 `config/voxblox/voxblox_config.yaml`。运行中在算法终端按 `s`，或在容器里调用下面的服务，会保存 `map.vxblx` 和 `mesh.ply`：

```bash
rosservice call /pose_graph/save_map
```

## 消融评测

自动化脚本会完整跑 `Normal.bag` 和 `darkroom1.bag`，每个 bag 跑 4 组：

- `baseline`：关闭 Zero-DCE++，关闭 Depth-to-map
- `zero_dce_only`：只开启 Zero-DCE++ ONNX C++
- `depth_to_map_only`：只开启 Depth-to-map
- `full`：Zero-DCE++ ONNX C++ 和 Depth-to-map 全部开启

完整命令：

```bash
cd /home/mhenwa/slam/VINS-RGBD
python3 tools/run_ablation_eval.py \
  --normal-bag /home/mhenwa/slam/bags/Normal.bag \
  --darkroom-bag /home/mhenwa/slam/bags/darkroom1.bag \
  --gt-root /home/mhenwa/slam/Ground-Challenge/psudo_gt
```

如果已经编译过，可跳过编译：

```bash
python3 tools/run_ablation_eval.py --skip-build
```

只跑某一个组合：

```bash
python3 tools/run_ablation_eval.py \
  --skip-build \
  --sequence darkroom1 \
  --variant full
```

每个 variant 会输出：

- `vins/vins_result_loop.csv`
- `voxblox/map.vxblx`
- `voxblox/mesh.ply`
- `perf_monitor.json`
- `eval/<variant>_metrics.json`
- `eval/<variant>_traj_xy.png`
- `eval/<variant>_traj_xyz_time.png`
- `eval/<variant>_trans_error_time.png`

汇总文件：

- `output/ablation/<timestamp>/summary.csv`
- `output/ablation/<timestamp>/summary.md`

本次完整消融结果，输出目录为 `output/ablation/final_ablation_20260507_210429/`：

| Seq | Variant | ATE RMSE m | ATE Mean m | ATE Max m | Enhanced Hz | Feature Hz | Odom Hz | Raw->Enh ms | Img->Feat ms | Img->Odom ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| normal | baseline | 0.1476 | 0.1356 | 0.3951 |  | 7.3613 | 7.3715 |  | 25.9849 | 57.5555 |
| normal | zero_dce_only | 0.1442 | 0.1315 | 0.3938 | 20.4944 | 6.8311 | 6.8413 | 12.8579 | 17.0156 | 89.2019 |
| normal | depth_to_map_only | 0.1405 | 0.1291 | 0.4268 |  | 7.3202 | 7.3330 |  | 26.2489 | 57.4532 |
| normal | full | 0.1582 | 0.1480 | 0.3727 | 20.4279 | 6.8091 | 6.8160 | 12.9601 | 17.1076 | 55.9022 |
| darkroom1 | baseline | 0.4305 | 0.3988 | 0.6983 |  | 10.4900 | 10.5205 |  | 8.7332 | 29.4437 |
| darkroom1 | zero_dce_only | 0.4176 | 0.3775 | 0.8990 | 14.9910 | 10.4903 | 10.5191 | 12.7395 | 8.5422 | 30.6502 |
| darkroom1 | depth_to_map_only | 0.4181 | 0.3853 | 0.7373 |  | 10.4903 | 10.5195 |  | 8.4712 | 31.4505 |
| darkroom1 | full | 0.4016 | 0.3638 | 0.8406 | 14.9914 | 10.4915 | 10.5209 | 12.7147 | 8.6047 | 33.2247 |

从这次结果看，Ground-Challenge 的 `darkroom1.bag` 上完整系统 ATE RMSE 最低；`Normal.bag` 上 Depth-to-map 单独开启最好，完整系统略差于 baseline。实时性方面，Zero-DCE++ ONNX C++ 增强节点在 CPU 上约 15-20 Hz，前端和里程计维持约 6.8-10.5 Hz。

针对 `darkroom1.bag` 这类低照、低纹理场景，`groundchallenge_config.yaml` 额外开启了 LK 前后向一致性检查：

- `lk_forward_backward_check: 1`
- `lk_max_fwd_bwd_error: 1.5`
- `lk_max_track_error: -1.0`

该检查会过滤无法从当前帧稳定反跟回上一帧的光流点，减少 Zero-DCE++ 增强后伪纹理或低纹理漂移点进入 VIO 和 Depth-to-map。开启后重跑 `darkroom1/full`：

| Seq | Variant | ATE RMSE m | ATE Mean m | ATE Max m | Enhanced Hz | Feature Hz | Odom Hz | Raw->Enh ms | Img->Feat ms | Img->Odom ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| darkroom1 | full + LK FB check | 0.3327 | 0.3026 | 0.6649 | 15.0004 | 10.4840 | 10.5134 | 13.2321 | 9.2473 | 33.9547 |

---


## RGBD-Inertial Trajectory Estimation and Mapping for Small Ground Rescue Robot
Based one open source SLAM framework [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono).

The approach contains
+ Depth-integrated visual-inertial initialization process.
+ Visual-inertial odometry by utilizing depth information while avoiding the limitation is working for 3D pose estimation.
+ Noise elimination map which is suitable for path planning and navigation.

However, the proposed approach can also be applied to other application like handheld and wheeled robot.

This dataset is part of the dataset collection of the [STAR Center](https://star-center.shanghaitech.edu.cn/), [ShanghaiTech University](http://www.shanghaitech.edu.cn/eng): https://star-datasets.github.io/

A video showing the data is available here: https://robotics.shanghaitech.edu.cn/datasets/VINS-RGBD

## Paper
Shan, Zeyong, Ruijian Li, and Sören Schwertfeger. "RGBD-inertial trajectory estimation and mapping for ground robots." Sensors 19.10 (2019): 2251.


    @article{shan2019rgbd,
      title={RGBD-inertial trajectory estimation and mapping for ground robots},
      author={Shan, Zeyong and Li, Ruijian and Schwertfeger, S{\"o}ren},
      journal={Sensors},
      volume={19},
      number={10},
      pages={2251},
      year={2019},
      publisher={Multidisciplinary Digital Publishing Institute}
    }


## 1. Prerequisites
1.1. **Ubuntu** 16.04 or 18.04.

1.2. **ROS** version Kinetic or Melodic fully installation

1.3. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html)


## 2. Datasets
Recording by RealSense D435i. Contain 9 bags in three different applicaions:
+ [Handheld](https://star-center.shanghaitech.edu.cn/seafile/d/0ea45d1878914077ade5/)
+ [Wheeled robot](https://star-center.shanghaitech.edu.cn/seafile/d/78c0375114854774b521/) ([Jackal](https://www.clearpathrobotics.com/jackal-small-unmanned-ground-vehicle/))
+ [Tracked robot](https://star-center.shanghaitech.edu.cn/seafile/d/f611fc44df0c4b3d936d/)

Note the rosbags are in compressed format. Use "rosbag decompress" to decompress.

Topics:
+ depth topic: /camera/aligned_depth_to_color/image_raw
+ color topic: /camera/color/image_raw
+ imu topic: /camera/imu




## 3. Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.
