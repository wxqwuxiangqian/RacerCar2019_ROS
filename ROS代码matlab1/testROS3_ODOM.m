% roslaunch rikirobot teleop_twist_keyboard.py
% rostopic pub -r 0.3 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
% https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py
%rostopic pub -1 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}'
% rosinit('192.168.1.112')
% rostopic info /odom
myodom = rossubscriber('/odom')
figure(2)
hold on
for i=1:1000
odomdata = receive(myodom,10);
showdetails(odomdata.Pose.Pose.Position);
x = odomdata.Pose.Pose.Position.X;
y = odomdata.Pose.Pose.Position.Y;

plot(x,y,'+')
view(90,-90)
axis equal
pause(0.1)
end

hold off