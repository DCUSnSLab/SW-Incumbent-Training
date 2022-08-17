source devel/setup.bash
roslaunch racecar teleop.launch &
roslaunch rplidar_ros rplidar.launch &
roslaunch hector_slam_launch tutorial_tif.launch &
