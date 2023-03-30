## 欧镭LR-16F雷达ROS驱动  ##

### Version:1.0.6 ###
### Update Time:2020-12-01 ###

### 更新内容 ###

变更点云数据维度为2维。共16线
1.launch 文件的这个参数表示是否用2维，
	<arg name="organize_cloud" default="true" />
	调用src/ole_pointcloud/src/conversions/organized_cloudXYZIR.cc
 
2.pointcloud2 中是列主秩，现改为行主秩
	在 src/ole_pointcloud/include/ole_pointcloud/datacontainerbase.h中修改

查看指令：rostopic echo olei_points | grep height 
#### 构建 ####
1. 在安装了ROS环境的ubuntu系统中创建工作区

    > mkdir -p ole3d_ws/src

2. 解压 'src' ROS驱动文件夹到ole2d_ws

    >cp src ole2d_ws

3. 安装 depend

    >rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
     
4. 编译
	
    >chmod -R 777 src
    >catkin_make
    注：编译前请chmod赋予src文件夹下可执行权限。

#### 运行 ####

1. 配置source源

    >source devel/setup.bash

2. 打开一个新的终端，运行roscore

    >roscore

3. 检查并连接激光雷达
	雷达默认出厂IP：192.168.1.100它将发送UDP数据包至192.168.1.10:2368
	因此，需配置本地静态IP：192.168.1.10子网掩码：255.255.255.0
    

4. 在当前配置source源的终端中，运行launch脚本

    >roslaunch ole_pointcloud LR16F_points.launch

#### rviz 可视化工具 ####
1. 打开新的终端，运行rviz
    >rosrun rviz rviz -f olei_lidar
    
2. 在rviz中add添加一个topic话题 PointCloud2
    
3. 如果有错误提示no fixed frame, 使用命令: 
    >rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map olei_lidar 10
