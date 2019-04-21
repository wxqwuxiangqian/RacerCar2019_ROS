clear all;
close all;
% Connect to a ROS Network,http://www.mathworks.com/help/robotics/examples/connect-to-a-ros-network.html
% rosinit('http://192.168.1.113:12000')
% setenv('ROS_MASTER_URI','http://192.168.1.113:11311')
% setenv('ROS_IP','192.168.1.113')
% rosinit
%roslaunch rikirobot bringup.launch
rosinit('192.168.1.112')
rosnode list
rostopic list
%roslaunch rikirobot bringup.launch
%roslaunch rikirobot lidar_slam.launch
%roslaunch rikirobot navigate.launch
%%%%%%%%%%%%%%%%EXAMPLE 1
rostopic info /imu/data 
% imudata = rosmessage('riki_msgs/Imu')
% msgInfo = rosmsg('show','/imu/data')
% imudata = rosmessage('sensor_msgs/Imu');
rawimu = rossubscriber('/imu/data')

imudata = receive(rawimu,10)
showdetails(imudata)


odom = rossubscriber('odom')
odomdata = receive(rawimu,10)
showdetails(odomdata)
odom  


rosservice list

rostopic echo /imu/data


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%EXAMPLE 2

maxRange = 2; % meters
resolution = 10; % cells per meter
slamObj = robotics.LidarSLAM(resolution,maxRange);


laser = rossubscriber('/scan')

%%%%不停的获取数据
%%
scandata = receive(laser,10);
figure,
plot(scandata,'MaximumRange',2)
set(gca,'YDir','reverse')
cart = readCartesian(scandata);
scandata1 = lidarScan(cart);
addScan(slamObj,scandata1)

%%
% close all
[scansSLAM,poses] = scansAndPoses(slamObj);
occGrid = buildMap(scansSLAM,poses,resolution,maxRange);

figure
show(occGrid)
view(-90, 90)
set(gca,'YDir','reverse')
title('Occupancy Map of Garage')
%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%EXAMPLES 3
% roslaunch rikirobot teleop_twist_keyboard.py
% rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0, z: 0}, angular: {x: 0, y: 0, z: -0.5}}'
% https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

cmdvelpub = rospublisher('/cmd_vel');
msg = rosmessage(cmdvelpub);

%%%车动一下
%%
msg.Linear.X=0.1;
msg.Angular.Z = -0.5;
send(cmdvelpub,msg);

%%%%%%%%%%%%%%%%%%%%%%%%%
rostopic info /odom
myodom = rossubscriber('/odom')
odomdata = receive(myodom,10);
showdetails(odomdata)