2026.5.7

当前最终版本默认启用完整系统：Zero-DCE++ ONNX C++ 低照增强、Depth-to-map 约束、Voxblox 稠密建图、回环几何验证。不要用 `sudo` 进容器。

## 相比原版 VINS-RGBD 的主要改进

本分支是在原版 VINS-RGBD 的 RGB-D VIO、pose graph、回环和稀疏/半稠密点云基础上，面向黑暗、低纹理巷道和稠密重建需求做的工程增强。最终默认启用的是经过 darkroom 系列 bag 验证后保留的模块。

### 1. Zero-DCE++ ONNX C++ 低照增强

原版 VINS-RGBD 直接使用相机 RGB 图像做 KLT 光流和 BRIEF 回环描述子。低照场景下，图像对比度低、可跟踪角点少，前端容易退化。

本项目新增：

- `feature_tracker` 下的 Zero-DCE++ 模型权重和 ONNX 导出流程。
- C++ ROS 节点 `zero_dce_onnx_enhancer`，使用 ONNX Runtime CPU 推理，不依赖 PyTorch 运行时。
- Python/PyTorch Zero-DCE++ 节点作为 fallback 保留。
- `realsense_color.launch` 中通过 `use_zero_dce` 和 `zero_dce_use_onnx` 控制开关。
- RViz 对比配置 `config/zero_dce_compare.rviz`，可同时看原始图和增强图。

默认数据流变为：

```text
/camera/color/image_raw -> /zero_dce/image_enhanced -> feature_tracker / pose_graph
```

在 darkroom 测试中，ONNX C++ 节点 CPU 上约 15 Hz，前端和里程计保持约 10 Hz。

### 2. Depth-to-map 几何约束

原版 VINS-RGBD 主要使用特征重投影、IMU 预积分、深度辅助初始化和 pose graph 优化。深度图更多用于给特征点赋深度和生成点云，没有把当前帧深度与历史局部地图直接构成几何约束。

本项目新增：

- VIO 滑窗内的 Depth-to-map residual。
- Pose graph 后端中的 Depth-to-map residual。
- 基于历史关键帧深度云的局部平面拟合。
- 深度点到局部地图平面的残差。
- 参数化开关：
  - `use_depth_to_map`
  - `use_depth_to_map_pose_graph`
  - `depth_map_weight`
  - `depth_map_huber`
  - `depth_map_min_edges`
  - `depth_map_max_edges_per_frame`
  - `depth_cloud_sync_tol`

这部分对 darkroom 低纹理场景有实际帮助，因为 RGB 特征弱时，深度几何可以给位姿提供额外约束。

### 3. Voxblox TSDF/ESDF 稠密建图

原版 VINS-RGBD 的建图更接近点云/Octree 输出，不是面向高质量稠密表面重建的 TSDF pipeline。

本项目在 `pose_graph` 后端集成 Voxblox：

- 发布 TSDF/ESDF 点云和 mesh。
- 保存 `output/voxblox/map.vxblx`。
- 保存彩色 `output/voxblox/mesh.ply`。
- 保留 legacy PCD / Octree 输出。
- 支持正常 `Ctrl+C` 退出自动保存。
- 用关键帧位姿和深度图做 TSDF 融合。
- 从同步 RGB 图像采样颜色，生成彩色 mesh。

相关输出：

- `/pose_graph/voxblox/mesh`
- `/pose_graph/voxblox/surface_pointcloud`
- `/pose_graph/voxblox/tsdf_pointcloud`
- `/pose_graph/voxblox/esdf_pointcloud`
- `output/voxblox/map.vxblx`
- `output/voxblox/mesh.ply`

### 4. 回环几何验证

原版 VINS-RGBD 使用 BoW/BRIEF 和 PnP 流程做回环。巷道、墙面、重复结构场景中，单纯外观相似容易产生误回环。

本项目增强了回环验证：

- BoW 分数阈值参数化。
- 回环候选阈值参数化。
- Fundamental Matrix RANSAC 前置过滤。
- PnP inlier 数和 inlier ratio 门限参数化。
- yaw / translation 几何门限。
- 回环诊断日志输出匹配数、F-check、PnP inliers 等信息。

关键参数：

- `loop_geom_verify`
- `loop_enable_fundamental_check`
- `loop_fundamental_threshold_px`
- `loop_min_pnp_inliers`
- `loop_min_inlier_ratio`
- `loop_max_yaw_deg`
- `loop_max_translation_m`

darkroom1 上开启回环几何验证后，ATE RMSE 从约 `0.333 m` 降到约 `0.309 m`。

### 5. 前端与同步稳定性改进

除四个主模块外，还保留了一些 VINS 侧的小改动：

- LK forward-backward check：过滤不稳定光流点。
- `image_discontinue_threshold` 参数化：避免低速播放或深度慢时误判新序列。
- 图像/深度 ApproximateTime 同步队列加大。
- feature/depth cloud 同步容差 `depth_cloud_sync_tol`。
- 稠密深度中值滤波、边缘过滤、自适应采样。
- 统一输出目录到 `output/`。
- 自动化评测脚本记录 ATE、轨迹图、topic Hz、端到端延迟。
- 独立 ROS master 端口选项，避免交互运行和批量评测互相污染参数。

### 6. SubSurfaceGeoRobo / ZED2 支持

为 `/home/mhenwa/slam/bags/SubSurfaceGeoRobo/` 这类 ZED2 双目数据新增：

- `vins_estimator/launch/subsurface_georobo.launch`
- ZED2 stereo depth 节点 `feature_tracker/scripts/zed2_stereo_depth_node.py`
- 低分辨率配置 `config/subsurface_georobo/zed2_robot_imu_lowres_config.yaml`
- 低分辨率深度配置 `config/subsurface_georobo/zed2_lowres_depth_config.yaml`
- 针对跟踪断流的 `image_discontinue_threshold` 调整

该路径用于 ZED2 左右目图像在线生成深度，再进入 VINS-RGBD/Voxblox 流程。

### 7. 已验证后禁用的实验模块

下面两个方向已经实现过，但 darkroom1/2/3 完整测试没有稳定收益，因此最终版本强制禁用，只保留注释掉的参考代码：

- Depth-to-map 不确定性权重
- 轻量地面/墙面结构平面约束

当前即使通过 launch 传入 `depth_map_uncertainty_enable:=1` 或 `use_structural_planes:=1`，代码也会强制关闭，避免误用。

## Darkroom 评测结论

最终推荐系统是：

```text
Zero-DCE++ ONNX + Depth-to-map + Voxblox + 回环几何验证
```

在 darkroom 系列上，相比关闭 Zero-DCE、Depth-to-map、回环几何验证的 baseline，完整系统整体提升：

| Seq | baseline ATE RMSE m | full ATE RMSE m | 变化 |
| --- | ---: | ---: | ---: |
| darkroom1 | 0.4305 | 约 0.3431，历史最佳约 0.3089 | 提升 |
| darkroom2 | 0.8453 | 0.4549 | -46.2% |
| darkroom3 | 0.7502 | 0.5727 | -23.7% |

说明：

- Zero-DCE++、Depth-to-map 和回环几何验证直接影响轨迹精度。
- Voxblox 主要提升稠密建图和 mesh 输出能力，不应单独用 ATE 判断收益。
- Normal.bag 不是主要优化目标；Release/RealSense 配置保留可关闭各模块的参数。

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

自动化脚本会完整跑 `Normal.bag` 和 `darkroom1.bag`，默认每个 bag 跑 4 组：

- `baseline`：关闭 Zero-DCE++，关闭 Depth-to-map
- `zero_dce_only`：只开启 Zero-DCE++ ONNX C++
- `depth_to_map_only`：只开启 Depth-to-map
- `full`：Zero-DCE++ ONNX C++ 和 Depth-to-map 全部开启，新优化开关关闭，用作旧完整系统参考

额外可选的 darkroom 优化消融：

- `loop_geom_only`：在 `full` 基础上开启回环几何验证

Depth-to-map 不确定性权重和轻量地面/墙面平面约束已经退回为禁用实验代码，不再作为可运行消融 variant 暴露。完整 darkroom1/2/3 测试显示它们没有稳定收益。

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

只跑 darkroom 优化路线：

```bash
python3 tools/run_ablation_eval.py \
  --skip-build \
  --sequence darkroom1 \
  --variant full \
  --variant loop_geom_only
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

在此基础上，新增 Depth-to-map 不确定性权重、轻量平面约束、回环几何验证后，针对 `darkroom1.bag` 的测试输出目录为 `output/ablation/darkroom_opt_20260508_160538/`：

| Seq | Variant | ATE RMSE m | ATE Mean m | ATE Max m | Enhanced Hz | Feature Hz | Odom Hz | Raw->Enh ms | Img->Feat ms | Img->Odom ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| darkroom1 | full | 0.3329 | 0.3036 | 0.6949 | 14.9915 | 10.4846 | 10.5159 | 12.7307 | 8.9998 | 32.1090 |
| darkroom1 | uncertainty_only | 0.3952 | 0.3575 | 0.8249 | 14.9910 | 10.4905 | 10.5232 | 12.3937 | 8.8575 | 30.8096 |
| darkroom1 | planes_only | 0.3378 | 0.3070 | 0.7471 | 14.9908 | 10.4844 | 10.5137 | 12.5039 | 8.8621 | 271.9199 |
| darkroom1 | loop_geom_only | 0.3089 | 0.2896 | 0.5486 | 14.9911 | 10.4840 | 10.8662 | 12.4194 | 8.8515 | 110.3107 |
| darkroom1 | full_optimized | 0.3369 | 0.3048 | 0.6553 | 14.9900 | 10.4838 | 10.5127 | 12.8895 | 9.1779 | 31.9642 |

额外组合 `planes_loop` 输出目录为 `output/ablation/darkroom_planes_loop_20260508_161425/`，ATE RMSE 为 `0.3843 m`，说明当前轻量平面约束与回环几何验证叠加会退化。当前推荐默认配置只开启已验证有效的回环几何验证：

- `loop_geom_verify: 1`
- `depth_map_uncertainty_enable: 0`
- `use_structural_planes: 0`

不确定性权重和平面结构约束保留为注释掉的实验参考代码，当前最终版本不允许通过 launch 参数实际启用。

## SubSurfaceGeoRobo ZED2 数据运行

`/home/mhenwa/slam/bags/SubSurfaceGeoRobo/02_20m_zed2.bag` 包含 ZED2 左右目 rect 彩色图和 ZED2 IMU，没有直接录制 RGB-D depth。项目中新增了 `zed2_stereo_depth_node.py`，运行时用左右目图像生成对齐到左目的 `mono16` 深度图：

- left image: `/zed_node/left/image_rect_color`
- right image: `/zed_node/right/image_rect_color`
- imu: `/zed_node/imu/data`
- generated depth: `/subsurface_georobo/zed2/depth`

先启动容器：

```bash
cd /home/mhenwa/slam/VINS-RGBD
docker run --rm -it --network host \
  -v /home/mhenwa/slam/VINS-RGBD:/workspace/VINS-RGBD \
  -v /home/mhenwa/slam/VINS-RGBD/.docker_catkin_ws:/workspace/VINS-RGBD/.docker_catkin_ws \
  -v /home/mhenwa/slam/bags/SubSurfaceGeoRobo:/bags \
  -w /workspace/VINS-RGBD \
  --name vins-rgbd \
  vins-rgbd:melodic \
  bash
```

容器内编译并运行：

```bash
source /opt/ros/melodic/setup.bash
cd /workspace/VINS-RGBD/.docker_catkin_ws
catkin build feature_tracker vins_estimator pose_graph -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
cd /workspace/VINS-RGBD

roslaunch vins_estimator subsurface_georobo.launch
```

另开一个终端播放 bag：

```bash
docker exec -it vins-rgbd bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
rosbag play /bags/02_20m_zed2.bag
```

可视化：

```bash
docker exec -it vins-rgbd bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
rviz -d /workspace/VINS-RGBD/config/vins_rviz_config.rviz
```

烟测结果：短回放 35 秒时，`/subsurface_georobo/zed2/depth`、`/feature_tracker/feature`、`/vins_estimator/odometry` 均有输出；stereo depth 约 `6.7-8.9 Hz`，单帧约 `57-64 ms`，有效深度比例约 `67-79%`。由于这是 SGBM 伪 RGB-D，默认先关闭 `Depth-to-map`，确认轨迹稳定后再用 launch 参数开启：

```bash
roslaunch vins_estimator subsurface_georobo.launch \
  use_depth_to_map:=1 \
  use_depth_to_map_pose_graph:=1
```

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
