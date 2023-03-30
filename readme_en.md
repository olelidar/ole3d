## LR-16F ROS Drive and PointCloud2  ##

### Version:1.0.6 ###
### Update Time:2020-12-01 ###

#### build ####
1. make catkin workspace at your ros machine.

    > mkdir -p catkin_ws/src

2. get velodyne.tar.gz from vendor tech support

3. unzip  zipfile at catkin_ws/src

    >tar zxvf velodyne.tar.gz

4. install depend

    >rosdep install --from-paths src --ignore-src --rosdistro kinetic -y

5. build

    >catkin_make

#### run ####
1. source to path

    >source devel/setup.bash

2. new terminal,then run roscore

    >roscore

3. connect lidar and host,lidar ip default 192.168.1.100,port default 2368

    host ip default 192.168.1.10

    these ip and port can be modified by vendor config software

4. run lidar drive at first terminal

    >roslaunch ole_pointcloud LR16F_points.launch

#### rviz view ####
1. new terminal,then run rviz
    >rosrun rviz rviz -f olei_lidar
    
2. at rviz,add topic PointCloud2
    
3. If you have an error about no fixed frame, use: 
    >rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map olei_lidar 10
