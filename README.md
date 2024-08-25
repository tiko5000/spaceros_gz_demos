# spaceros_gz_demos



https://github.com/user-attachments/assets/b5a11627-e0f3-451b-af0c-b0e76b30fb04



This is a ROS 2 package demonstrating how to use Gazebo for robotic simulations in the Space ROS environment. The simulated worlds include a submersible robot on Enceladus, the Perseverance rover and Ingenuity helicopter on Mars, a space capsule docking to the ISS, and two rovers on the Moon. Gravity of each world is set to their actual values, and the Moon uses the Selenographic Coordinate System (SCS). Perseverance and Ingenuity models are meant to be as close to their real-life counterparts as possible.

## Docker setup

If you don't have docker, install it with `sudo apt install docker.io`. If you get a permission denied error with the Docker daemon, you will have to add yourself to the docker user group with `sudo usermod -aG docker $USER`, then run `newgrp docker`. You may have also to run `xhost +local:docker` on your first time. You will need to log out and log back in for these changes to take effect on your system.

1. Navigate to the `docker` directory

2. Build the docker image locally with

    ```./docker_build.sh```

3. Start the container with

    ```./docker_start.sh```

4. Once you have a running container, to get another shell, run 

    ```./docker_shell.sh```

## Running code in the Docker container
1. Once you are in the Docker container, run `colcon build` (you should be in the `~/spaceros/ws` directory)

2. Run `source install/setup.bash`

3. Start one of the demos with `ros2 launch spaceros_gz_demos moon.launch.xml`, `mars.launch.xml`, or `enceladus.launch.xml`

## Moon
Launch the moon demo with the following command:

```ros2 launch spaceros_gz_demos moon.launch.xml```

This world contains an X1 rover holding a truss, an X2 rover, and a solar panel with one controllable joint.

| Topic Name | Topic Type | Description | 
| ---------- | ---------- | ----------- |
| /X1/camera_front/camera_info | sensor_msgs/msg/CameraInfo |                 | 
| /X1/camera_front/image | sensor_msgs/msg/Image |                 | 
| /X1/cmd_vel | geometry_msgs/msg/Twist |                 | 
| /X1/front_laser/scan | sensor_msgs/msg/LaserScan |                 | 
| /X1/front_laser/scan/points | sensor_msgs/msg/PointCloud2 |                 | 
| /X1/imu_sensor/imu | sensor_msgs/msg/Imu |                 | 
| /X1/odometry | nav_msgs/msg/Odometry |                 | 
| /X1/odometry_with_covariance | nav_msgs/msg/Odometry |                 | 
| /X1/truss/attach | std_msgs/msg/Empty |                 | 
| /X1/truss/detach | std_msgs/msg/Empty |                 | 
| /X2/camera_front/camera_info | sensor_msgs/msg/CameraInfo |                 | 
| /X2/camera_front/image | sensor_msgs/msg/Image |                 | 
| /X2/cmd_vel | geometry_msgs/msg/Twist |                 | 
| /X2/front_laser/scan | sensor_msgs/msg/LaserScan |                 | 
| /X2/front_laser/scan/points | sensor_msgs/msg/PointCloud2 |                 | 
| /X2/imu_sensor/imu | sensor_msgs/msg/Imu |                 | 
| /X2/odometry | nav_msgs/msg/Odometry |                 | 
| /X2/odometry_with_covariance | nav_msgs/msg/Odometry |                 | 
| /solar_panel/joint | std_msgs/msg/Float64 |                 | 
| /tf | tf2_msgs/msg/TFMessage |                 | 


## Mars
Launch the mars demo with the following command:

```ros2 launch spaceros_gz_demos moon.launch.xml```

This world contains models of the Perserverance rover and the Ingenuity helicopter.

| Topic Name | Topic Type | Description | 
| ---------- | ---------- | ----------- |
| /ingenuity/battery_recharge_start | std_msgs/msg/Bool |                 | 
| /ingenuity/battery_state | sensor_msgs/msg/BatteryState |                 | 
| /ingenuity/bottom_blades/thrust | std_msgs/msg/Float64 |                 | 
| /ingenuity/camera | sensor_msgs/msg/Image |                 | 
| /ingenuity/camera_info | sensor_msgs/msg/CameraInfo |                 | 
| /ingenuity/depth_camera | sensor_msgs/msg/Image |                 | 
| /ingenuity/depth_camera/points | sensor_msgs/msg/PointCloud2 |                 | 
| /ingenuity/odometry | nav_msgs/msg/Odometry |                 | 
| /ingenuity/swashplate_1/joint | std_msgs/msg/Float64 |                 | 
| /ingenuity/swashplate_2/joint | std_msgs/msg/Float64 |                 | 
| /ingenuity/top_blades/thrust | std_msgs/msg/Float64 |                 | 
| /perseverance/arm/joint_1 | std_msgs/msg/Float64 |                 | 
| /perseverance/arm/joint_2 | std_msgs/msg/Float64 |                 | 
| /perseverance/arm/joint_3 | std_msgs/msg/Float64 |                 | 
| /perseverance/arm/joint_4 | std_msgs/msg/Float64 |                 | 
| /perseverance/arm/joint_5 | std_msgs/msg/Float64 |                 | 
| /perseverance/camera | sensor_msgs/msg/Image |                 | 
| /perseverance/camera_info | sensor_msgs/msg/CameraInfo |                 | 
| /perseverance/camera_yaw | std_msgs/msg/Float64 |                 | 
| /perseverance/cmd_vel | geometry_msgs/msg/Twist |                 | 
| /perseverance/depth_camera | sensor_msgs/msg/Image |                 | 
| /perseverance/depth_camera/points | sensor_msgs/msg/PointCloud2 |                 | 
| /perseverance/odometry | nav_msgs/msg/Odometry |                 | 
| /tf | tf2_msgs/msg/TFMessage |                 | 


## Enceladus
Launch the Enceladus demo with the following command:

```ros2 launch spaceros_gz_demos moon.launch.xml```

This world contains a submarine model.

| Topic Name | Topic Type | Description | 
| ---------- | ---------- | ----------- |
| /submarine/buoyancy_engine | std_msgs/msg/Float64 |                 | 
| /submarine/left_thrust | std_msgs/msg/Float64 |                 | 
| /submarine/odometry | nav_msgs/msg/Odometry |                 | 
| /submarine/right_thrust | std_msgs/msg/Float64 |                 | 
| /submarine/sonar | sensor_msgs/msg/LaserScan |                 | 
| /submarine/sonar/points | sensor_msgs/msg/PointCloud2 |                 | 



## Orbit
Launch the orbiter demo with the following command:

```ros2 launch spaceros_gz_demos orbit.launch.xml```

This world contains a model of the International Space Station in orbit above the Earth.

| Topic Name | Topic Type | Description | 
| ---------- | ---------- | ----------- |
| /capsule/lidar | sensor_msgs/msg/LaserScan |                 | 
| /capsule/lidar/points | sensor_msgs/msg/PointCloud2 |                 | 
| /capsule/thrust/pitch | std_msgs/msg/Float64 |                 | 
| /capsule/thrust/push | std_msgs/msg/Float64 |                 | 
| /capsule/thrust/yaw | std_msgs/msg/Float64 |                 | 
