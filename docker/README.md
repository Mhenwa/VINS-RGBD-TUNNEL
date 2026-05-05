# Docker 运行说明

这套 Docker 文件按当前仓库实际情况整理，目标环境为 `Ubuntu 18.04 + ROS Melodic`。

仓库里的默认入口是 `realsense_color.launch`，默认 bag 使用你当前提供的：

```bash
/home/mhenwa/slam/bags/Normal.bag
```

容器内会自动把它映射为：

```bash
/data/Normal.bag
```

同时，项目配置文件里默认运行结果目录是 `/home/shanzy/output/vins/`，脚本会把宿主机 `output/` 挂载到 `/home/shanzy/output/` 并预创建常用子目录。

## 1. 文件说明

- `docker/Dockerfile`
  - 构建 ROS Melodic 镜像，并预装 Ceres、PCL、OpenCV、catkin_tools。
- `docker/run_container.sh`
  - 只负责启动容器和挂载目录，不做编译。
- `docker/build_in_container.sh`
  - 只负责在容器内创建 catkin workspace 并编译。

## 2. 第一步：构建镜像

在仓库根目录执行：

```bash
docker build -t vins-rgbd:melodic -f docker/Dockerfile .
```

只需要构建一次。后面改代码后，通常不需要重新 build 镜像。

## 3. 第二步：运行容器

在仓库根目录执行：

```bash
./docker/run_container.sh
```

如果你以后想挂载别的 bag，也可以把 bag 路径作为第一个参数传进去：

```bash
./docker/run_container.sh /home/mhenwa/slam/bags/Other.bag
```

这个脚本会做几件事：

- 挂载当前仓库到容器内 `/workspace/VINS-RGBD`
- 挂载 bag 所在目录到容器内 `/data`
- 挂载输出目录到容器内 `/home/shanzy/output`
- 挂载 `.docker_catkin_ws`，让编译结果保存在宿主机
- 打开 X11，这样容器里可以直接运行 RViz

启动后你会进入容器 shell。

## 4. 第三步：容器内编译

进入容器后执行：

```bash
./docker/build_in_container.sh
```

编译完成后加载工作区环境：

```bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
```

如果你修改了代码，后续重新编译也只需要再执行一次：

```bash
./docker/build_in_container.sh
```

## 5. 第四步：容器内运行程序

在容器的第一个终端里启动算法：

```bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
roslaunch vins_estimator realsense_color.launch
```

这个 launch 会同时启动：

- `feature_tracker`
- `vins_estimator`
- `pose_graph`

## 6. 第五步：回放 bag

新开一个宿主机终端，进入同一个容器：

```bash
docker exec -it vins-rgbd bash
```

在第二个终端里执行：

```bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
rosbag play /data/Normal.bag
```

如果你的 bag 是压缩过的，先在宿主机或容器内解压：

```bash
rosbag decompress /data/Normal.bag
```

## 7. 第六步：打开 RViz

再开一个终端进入容器：

```bash
docker exec -it vins-rgbd bash
```

执行：

```bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
roslaunch vins_estimator vins_rviz.launch
```

## 8. 常用目录

- 仓库源码：
```bash
/workspace/VINS-RGBD
```

- 容器内 bag：
```bash
/data/Normal.bag
```

- 容器内编译工作区：
```bash
/workspace/VINS-RGBD/.docker_catkin_ws
```

- 容器内输出结果：
```bash
/home/shanzy/output
```

- 宿主机输出结果：
```bash
<repo>/output
```

## 9. 最小使用流程

宿主机：

```bash
docker build -t vins-rgbd:melodic -f docker/Dockerfile .
./docker/run_container.sh
```

容器内：

```bash
./docker/build_in_container.sh
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
roslaunch vins_estimator realsense_color.launch
```

另一个宿主机终端：

```bash
docker exec -it vins-rgbd bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
rosbag play /data/Normal.bag
```

## 10. 常见问题

### 1. `rviz` 打不开

先在宿主机执行：

```bash
xhost +local:
```

然后重新运行 `./docker/run_container.sh`。

### 2. 容器里找不到 bag

确认宿主机文件存在：

```bash
ls -lh /home/mhenwa/slam/bags/Normal.bag
```

然后在容器里确认：

```bash
ls -lh /data/Normal.bag
```

### 3. 编译成功但 `roslaunch` 找不到包

通常是没有 source 工作区：

```bash
source /opt/ros/melodic/setup.bash
source /workspace/VINS-RGBD/.docker_catkin_ws/devel/setup.bash
```
