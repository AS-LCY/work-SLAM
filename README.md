系统要求：ubuntu 20.04 ros noetic

配置网络
1 连接mid360网线，设置电脑ipv4为固定ip 192.168.1.5  255.255.255.0 192.168.1.254
2 配置lidar_slam/lib/config/MID360_config.json中lidar_configs下的"ip" : "192.168.1.100"中最后两位即00改为雷达机身编号的最后两位  
安装依赖库：
1 安装pcl  sudo apt-get install libpcl-dev
2 将lidar_slam 放在一个ros工作空间中
3 依次安装lidar_slam/lib/ThirdParty中的三方库(cmake ..  ；  sudo make install)。
  - yaml 
  - eigen  安装之前如果系统的/usr/include下如果有eigen要先删除，默认安装路径为/usr/local/include，安装完成之后把目录复制到/usr/include下，保证两个目录都是这个版本的eigen
  - fmt
  - Sophus
  - Pangolin 安装完成后运行 sudo ldconfig 刷新一下
  - Livox-SDK2
  - livox_ros_driver2 注意这是一个ros包，安装方式如下，source /opt/ros/noetic/setup.sh   ；     ./build.sh ROS1
  - gtsam
  - fast_gicp 注意这个不需要安装到系统中，也就是执行sudo make 即可，不需要install. You should set parameter "option(BUILD_PYTHON_BINDINGS "Build python bindings" OFF)" to "OFF"in file CmAKElist.txt firstly.
4 安装程序本体
  - 在lidar_slam/lib/build/ 下执行 cmake .. ; sudo make 
  - 在lidar_slam的上级ros空间中执行catkin_make
5 运行
  - 配置lidar_slam/lib/config/mid360.yaml中Lidar_In_Wheel这个外参，表示雷达到车体的坐标变换，其中wheel是车体坐标系，x是车的前进方向，z垂直向上。
  - 启动 roslaunch lidar_slam pc_mid360.launch ，其中唯一需要修改的参数是localization_mode ，当这个参数为0时就启动建图模式。为1时启动的是定位模式。
  - 建图模式下运行rostopic pub /command std_msgs/Int32 "data: 0"，会将地图保存在/lidar_slam/lib/map/目录下
  - 定位模式下运行会自动载入以上目录中地图,如果重定位成功，会打印globalLocalization success with score 然后会一直打印gicp success with score 表示定位正常
6 pangolin 显示
  - 如果在低端平台上运行，无法带动rviz，可以启动roslaunch lidar_slam car_mid360.launch ，使用pangolin来显示，没有rviz好用，凑合可以看


  
  
  ** $ source install/setup.bash
  ** $ roslaunch lidar_slam car_mid360.launch
