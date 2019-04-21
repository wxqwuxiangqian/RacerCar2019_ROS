clear all;
close all;
%roslaunch rikirobot bringup.launch
% $ roslaunch robot_pose_ekf.launch
% https://blog.csdn.net/wilylcyu/article/details/51891299
% https://ww2.mathworks.cn/matlabcentral/answers/196911-use-matlab-robotics-system-toolbox-to-receive-ros-message
%不要设定HOSTNAME
% setenv('ROS_MASTER_URI','http://192.168.1.112:11311')
% setenv('ROS_IP','192.168.1.113')
% rosinit('http://192.168.1.112:11311')
% rosnode list
% rostopic list

%%获取IMU和odom
%%注意1 基于RIKIBOBOT的IMU+ODOM有错误，因为车子轮子和本身动力学模型误差，理论上的直线行驶不存在
% rawimu = rossubscriber('/imu/data');
% imudata = receive(rawimu,10)
% showdetails(imudata)
% 
% 
% odom = rossubscriber('odom')
% odomdata = receive(rawimu,10)
% showdetails(odomdata)

% %%
% %%获取数据1/imu/data
% bag = rosbag('2019-01-27-21-33-29.bag');
% bagSelection = select(bag,'Topic','/imu/data');
% ts = timeseries(bagSelection);
% 
% tsdata = getdatasamples(ts,1:ts.length);
% dataimu = tsdata(:,4:13);
% seq = tsdata(:,4:13);
% sec = tsdata(:,2);
% nsec = tsdata(:,3);
% time2 = double(sec)+double(nsec)*10^-9;
% 
% %%
% %%基于Iomega计算位移，速度
% oriData = dataimu(:,1:4);
% anSData = dataimu(:,5:7);
% accData = dataimu(:,8:10);
% data2 = [time2 anSData accData ];
% init2 = [0 0 0; 0 0 0; 0 0 0];
% output=find_position(data2,init2)

%%
%%获取数据3 /robot_pose_ekf/odom_combined  得出结论，基于IMU积分+EKF，1分钟误差在分米级别，而且有扩大趋势
% rosbag record /imu/data_raw /imu/data /odom /robot_pose_ekf/odom_combined
% rawimu = rossubscriber('/robot_pose_ekf/odom_combined');
% imudata = receive(rawimu,10)
% showdetails(imudata)
if 0
bag = rosbag('2019-01-29-02-29-45.bag');
bagSelection = select(bag,'Topic','/robot_pose_ekf/odom_combined');
ts = timeseries(bagSelection);

tsdata = getdatasamples(ts,1:ts.length);
dataimu = tsdata(:,4:end);
dataPos = tsdata(:,[4 5 6 ]);
sec = tsdata(:,2);
nsec = tsdata(:,3);
time2 = double(sec)+double(nsec)*10^-9;

figure
hold on
plot(dataPos(:,1),dataPos(:,2),'.')
plot(dataPos(end,1),dataPos(end,2),'s')
plot(dataPos(1,1),dataPos(1,2),'o')
hold off

end

%%
%%获取数据2 /odo
% rosbag record /imu/data_raw /imu/data /odom /robot_pose_ekf/odom_combined
% odom = rossubscriber('odom')
% odomdata = receive(odom,10)
% showdetails(odomdata)
if 1
bag = rosbag('2019-01-29-02-29-45.bag');
bagSelection = select(bag,'Topic','/odom');
ts = timeseries(bagSelection);

tsdata = getdatasamples(ts,1:ts.length);

dataPos = tsdata(:,[4 5 6 ]);
sec = tsdata(:,2);
nsec = tsdata(:,3);
time2 = double(sec)+double(nsec)*10^-9;
oridata = tsdata(:,[7 8 9 10]);
linearSpeed = tsdata(:,[11 12 13]);
angularSpeed = tsdata(:,[14 15 16]);
figure
hold on
plot(dataPos(:,1),dataPos(:,2),'.')
plot(dataPos(end,1),dataPos(end,2),'s')
plot(dataPos(1,1),dataPos(1,2),'o')
hold off

end


