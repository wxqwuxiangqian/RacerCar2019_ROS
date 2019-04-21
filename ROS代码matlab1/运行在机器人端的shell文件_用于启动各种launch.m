
#!/bin/bash
roslaunch rikirobot bringup.launch &
roslaunch robot_pose_ekf myekf.launch &
roslaunch rikirobot bringup.launch &
sleep 10
roslaunch rikirobot lidar_slam.launch