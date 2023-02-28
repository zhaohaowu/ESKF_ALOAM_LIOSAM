一、直接使用kitti_lidar_only_2011_10_03_drive_0042_synced.bag数据集建图和定位

百度网盘下载bag文件

链接：https://pan.baidu.com/s/197f1AybOI1a7wKvlLwf6gQ?pwd=slam 
提取码：slam

运行下面命令

```cmd
git clone https://github.com/zhaohaowu/ESKF_ALOAM_LIOSAM
#安装第三方库
#安装fmt
sudo apt-get install libfmt-dev
#安装protobuf
cd ESKF_ALOAM_LIOSAM/3rdparty/protobuf-3.14.0
./autogen.sh
./configure
make
sudo make install
sudo ldconfig
protoc --version
#安装g2o
cd ../g2o
mkdir build
cd build
cmake ..
make
sudo make install
#安装GeographicLib
cd ../../GeographicLib-1.48
mkdir build
cd build
cmake ..
make
sudo make install

#NDT融合GPS建图
cd ../../../ch3_
catkin_make
source devel/setup.bash
roslaunch lidar_localization mapping.launch
#ctrl+shift+t新终端
rosbag play ~/bag/duo/kitti_lidar_only_2011_10_03_drive_0042_synced.bag
#ctrl+shift+t新终端
source devel/setup.bash
rosservice call /optimize_map
rosservice call /save_map
rosservice call /save_scan_context
#地图保存至ESKF_ALOAM_LIOSAM/ch3_/src/lidar_localization/slam_data/map

#ESKF ALOAM LIOSAM定位
cd ../ch3
catkin_make
source devel/setup.bash
#ESKF定位
roslaunch lidar_localization kitti_localization.launch
#ctrl+shift+t新终端
rosbag play ~/bag/duo/kitti_lidar_only_2011_10_03_drive_0042_synced.bag
#ALOAM定位
roslaunch aloam aloam.launch
#ctrl+shift+t新终端
rosbag play bag/duo/kitti_lidar_only_2011_10_03_drive_0042_synced.bag
#LIO-SAM定位
roslaunch lio_sam run.launch
#ctrl+shift+t新终端
rosbag play bag/liosam/kitti_2011_10_03_drive_0042_synced.bag
```

二、详细使用其他kitti数据集

ch3_建图算法工作空间，ch3定位算法工作空间

保存点云地图

```
cd ch3_
catkin_make
source devel/setup.bash
roslaunch lidar_localization mapping.launch
rosbag play bag/duo/kitti_lidar_only_2011_10_03_drive_0027_synced.bag
```

将建图算法生成的的slam_data文件夹的map和scan_context文件夹复制到ESKF算法的slam_data文件夹下

记录终端的odom_init_pose，替换aloam中aloam_scan_map_registration_node.cpp的odom_init_pose为对应数据集的雷达坐标系到gps坐标系的变换矩阵

修改ESKF算法的kitti_filtering.yaml的map_path和scan_context_path为对应的数据集地图路径

```
cd ch3
catkin_make
source devel/setup.bash
roslaunch lidar_localization kitti_localization.launch
rosbag play bag/duo/kitti_lidar_only_2011_10_03_drive_0027_synced.bag
```

生成ESKF(ours).txt和Ground_Truth.txt，将其复制至实验结果的对应数据集文件夹中，防止被覆盖

```
roslaunch aloam aloam.launch 
rosbag play bag/duo/kitti_lidar_only_2011_10_03_drive_0027_synced.bag
```

生成ALOAM.txt，将其复制至实验结果的对应数据集文件夹中，防止被覆盖

```
roslaunch lio_sam run.launch
rosbag play bag/liosam/kitti_2011_10_03_drive_0027_synced.bag
```

生成LIO-SAM.txt，将其复制至实验结果的对应数据集文件夹中，防止被覆盖
