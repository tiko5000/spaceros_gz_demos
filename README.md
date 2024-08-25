# spaceros_gz_demos

This is a ROS 2 package demonstrating how to use Gazebo for robotic simulations in the Space ROS environment. The simulated worlds include a submersible robot on Enceladus, the Perseverance rover and Ingenuity helicopter on Mars, and swarm of construction rovers on the Moon. Gravity of each world is set to their actual values, and the Moon uses the Selenographic Coordinate System (SCS).

## Docker setup

If you don't have docker, install it with `sudo apt install docker.io`. If you get a permission denied error with the Docker daemon, you will have to add yourself to the docker user group with `sudo usermod -aG docker $USER`, then run `newgrp docker`. You may have also to run `xhost +local:docker` on your first time. You will need to log out and log back in for these changes to take effect on your system.

1. Navigate to the `docker` directory

2. Build the docker image locally with

    ```./docker_build.sh```

3. Start the container with

    ```./docker_start.sh```

4. Once you have a running container, to get another shell, run 

    ```./docker_shell.sh```

## Running code in the docker container
1. Once you are in the docker container, run `colcon build` (you should be in the `~/spaceros/ws` directory)

2. Run `source install/setup.bash`

3. Start one of the demos with `ros2 launch spaceros_gz_demos moon.launch.xml`, `mars.launch.xml`, or `enceladus.launch.xml`

## Moon
Launch the moon demo with the following command:

```ros2 launch spaceros_gz_demos moon.launch.xml```

This world contains an X1 rover holding a truss, an X2 rover, and a solar panel with one controllable joint.

### ROS Topics
* /X1/camera_front/camera_info
* /X1/camera_front/image
* /X1/cmd_vel
* /X1/front_laser/scan
* /X1/front_laser/scan/points
* /X1/imu_sensor/imu
* /X1/odometry
* /X1/odometry_with_covariance
* /X1/truss/attach
* /X1/truss/detach
* /X2/camera_front/camera_info
* /X2/camera_front/image
* /X2/cmd_vel
* /X2/front_laser/scan
* /X2/front_laser/scan/points
* /X2/imu_sensor/imu
* /X2/odometry
* /X2/odometry_with_covariance
* /solar_panel/joint
* /tf

## Mars
Launch the mars demo with the following command:

```ros2 launch spaceros_gz_demos moon.launch.xml```

This world contains models of the Perserverance rover and the Ingenuity helicopter.


### ROS Topics
* /ingenuity/battery_recharge_start
* /ingenuity/battery_state
* /ingenuity/bottom_blades/thrust
* /ingenuity/camera
* /ingenuity/camera_info
* /ingenuity/depth_camera
* /ingenuity/depth_camera/points
* /ingenuity/odometry
* /ingenuity/swashplate_1/joint
* /ingenuity/swashplate_2/joint
* /ingenuity/top_blades/thrust
* /perseverance/arm/joint_1
* /perseverance/arm/joint_2
* /perseverance/arm/joint_3
* /perseverance/arm/joint_4
* /perseverance/arm/joint_5
* /perseverance/camera
* /perseverance/camera_info
* /perseverance/camera_yaw
* /perseverance/cmd_vel
* /perseverance/depth_camera
* /perseverance/depth_camera/points
* /perseverance/odometry
* /tf


## Enceladus
Launch the Enceladus demo with the following command:

```ros2 launch spaceros_gz_demos moon.launch.xml```

This world contains a submarine model.

### ROS Topics
* /submarine/buoyancy_engine
* /submarine/left_thrust
* /submarine/odometry
* /submarine/right_thrust
* /submarine/sonar
* /submarine/sonar/points


## Orbit
Launch the orbiter demo with the following command:

```ros2 launch spaceros_gz_demos orbit.launch.xml```

This world contains a model of the International Space Station in orbit above the Earth

### ROS Topics
* /capsule/lidar
* /capsule/lidar/points
* /capsule/thrust/pitch
* /capsule/thrust/push
* /capsule/thrust/yaw
