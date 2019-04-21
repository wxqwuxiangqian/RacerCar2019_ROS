% roslaunch rikirobot teleop_twist_keyboard.py
% rostopic pub -r 0.1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.05, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
% https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
%rostopic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
% rosinit('192.168.1.112')
% rostopic info /odom
%roslaunch ros_pose_ekf ros_pose_ekf.launch
rostopic list
mypose = rossubscriber('/robot_pose_ekf/odom_combined')
figure(1)
hold on
for i=1:1000
posemdata = receive(mypose,100);
% showdetails( posemdata.Pose.Pose.Position);
x = posemdata.Pose.Pose.Position.X-0.5667
y = posemdata.Pose.Pose.Position.Y

plot(x,y,'+')
pause(0.1)
end

hold off