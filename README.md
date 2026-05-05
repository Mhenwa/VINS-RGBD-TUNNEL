2026.4.3

删除了了Shpous，把对应部分换成了OpenCV

不要用sudo进！！！

## docker

### 第一个终端
```bash
docker build -t vins-rgbd:melodic -f docker/Dockerfile .
./docker/run_container.sh
```

进容器后：
```bash
./docker/build_in_container.sh
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
roslaunch vins_estimator realsense_color.launch
```

运行输出统一写到仓库根目录的 `output/`。在容器里对应路径是 `/home/shanzy/output/`：

- `output/vins/`：VINS 运行结果 CSV 和外参标定结果
- `output/eval/`：和 GT 对齐后的评估结果、误差图
- `output/plots/`：只画运行轨迹的图
- `output/gt/`：单独画 GT 的图
- `output/pose_graph/`：pose graph 保存/加载目录
- `output/pcd/`：pose graph 按键导出的 PCD

换成ground challenge的配置文件
```bash
roslaunch vins_estimator realsense_color.launch \
  config_path:=/workspace/VINS-RGBD/config/ground_challenge/groundchallenge_config.yaml \
  depth_config_path:=/workspace/VINS-RGBD/config/ground_challenge/groundchallenge_depth_config.yaml

```
### 新建终端打开可视化：
```bash
docker exec -it vins-rgbd bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
roslaunch vins_estimator vins_rviz.launch
```

### 另一个终端播包：
```bash
docker exec -it vins-rgbd bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
rosbag play /data/Normal.bag
```

### 跑完后和 Ground-Challenge 的 `psudo_gt` 比较：
```bash
python3 tools/eval_ground_challenge.py \
  --est output/vins/vins_result_loop.csv \
  --seq darkroom1.bag \
  --gt-root /home/mhenwa/slam/Ground-Challenge/psudo_gt \
  --out-dir output/eval/darkroom1 \
  --name loop
```

会输出：

- `<name>_metrics.json`
- `<name>_aligned_est.txt`
- `<name>_gt_used.txt`
- `<name>_traj_xy.png`
- `<name>_traj_xyz_time.png`
- `<name>_trans_error_time.png`

### 直接读取指定 `psudo_gt` 文件并画真值轨迹：
```bash
python3 tools/plot_ground_challenge_gt.py \
  --gt /home/mhenwa/slam/Ground-Challenge/psudo_gt/darkroom1.txt \
  --out-dir output/gt/darkroom1 \
  --name darkroom1_gt
```

会输出：

- `<name>_trajectory.txt`
- `<name>_traj_xy.png`
- `<name>_traj_xyz_time.png`



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
