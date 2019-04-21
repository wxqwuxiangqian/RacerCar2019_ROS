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
rosshutdown;
rosinit('http://192.168.1.112:11311')
% rawimu = rossubscriber('/imu/data');
% imudata = receive(rawimu,10)
% showdetails(imudata)

path = [0 0;
    0.5 0.25;
    1 0.5;
    1.5 0.75;
    1.7 1 ];
controller = robotics.PurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.1;
controller.MaxAngularVelocity = 20*pi/180;
controller.LookaheadDistance = 0.5;


odom = rossubscriber('odom')
% odom = rossubscriber('/robot_pose_ekf/odom_combined')

odomdata = receive(odom,100)
showdetails(odomdata)

laser = rossubscriber('/scan');
scan = receive(laser,3)
figure(1001)
plot(scan);
cmdvelpub = rospublisher('/cmd_vel');

vfh = robotics.VectorFieldHistogram('UseLidarScan',true);
vfh.DistanceLimits = [0.01 0.5];
vfh.RobotRadius = 0.3;
vfh.MinTurningRadius = 0.2;
vfh.SafetyDistance = 0.1;
distanceThreshold =0.5;
targetDir = 0;
%%


for i=1:1000
    scan = receive(laser);
    figure(1)
    plot(scan);
    
    scandata = readCartesian(scan);
    x = scandata(:,1);
    y = scandata(:,2);
    
    
	
    %%%%
    
    odomdata = receive(odom,100);
    X = odomdata.Pose.Pose.Position.X;
    Y = odomdata.Pose.Pose.Position.Y;
    ori1= odomdata.Pose.Pose.Orientation;
    [roll,pitch,yaw] = quat2angle([ori1.X ori1.Y ori1.Z ori1.W]);
    tp = [X Y yaw];
    poseposiT = [X Y yaw*180/pi];
    

     % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(tp);

    %%%%%%%%%%%
    % Call VFH object to computer steering direction
    % Compute distance of the closest obstacle
%      UseLidarScan: false
%           NumAngularSectors: 180
%              DistanceLimits: [0.0100 0.5000]
%                 RobotRadius: 0.3000
%              SafetyDistance: 0.5000
%            MinTurningRadius: 0.5000
%       TargetDirectionWeight: 5
%      CurrentDirectionWeight: 2
%     PreviousDirectionWeight: 2
%         HistogramThresholds: [3 10]


   dist = sqrt(x.^2 + y.^2);
   [minDist,index] = min(dist);

    if minDist < distanceThreshold
%         ranges = double(scan.Ranges);
%         angles = double(scan.readScanAngles);
%         steerDir = vfh(ranges, angles,  omega);%Target direction for the robot, specified as a scalar in radians. The forward direction of the robot is considered zero radians, with positive angles measured counterclockwise.

% scan2 = lidarScan( scandata);       
% steerDir = vfh(scan2,  omega);%Target direction for the robot, specified as a scalar in radians. The forward direction of the robot is considered zero radians, with positive angles measured counterclockwise.
%           figure(2);
%           show(vfh)
[steerAngle,pks] = simpleRangeHist(scan);

%   [T,loc] = min(abs(steerAngle-steerDir));
    [T,loc] = max(pks);
  steerAngle1 = steerAngle(loc);

          
          fprintf(' [X Y YAW]=[%.2f,%.2f,%2f],[V,omega] =[%.2f,%.2f],steerDir=[%.2f]\n',X,Y,yaw*180/pi,v,omega*180/pi, steerAngle1*180/pi);
          
    pause(1)
        
        if ~isnan(steerAngle1) % If steering direction is valid
            v = 0.1;
             omega =  steerAngle1;
        else
             v = -1;
             omega = 0;
        end
    end
      
      if minDist < 0.1
          v = -1;
             omega = 0;
      end
    cmdvelpub = rospublisher('/cmd_vel');
    msg = rosmessage(cmdvelpub);
    msg.Linear.X=v;
    msg.Angular.Z = omega;
    send(cmdvelpub,msg);
  
    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    
    dist = norm([X Y] - path(end,:));
    if dist<0.2
        break;
    end
    
end


function [steerAngle,pks] = simpleRangeHist(scan)
ranges = double(scan.Ranges);
angles = double(scan.readScanAngles);
ind = find(angles<90*pi/180 & angles>-90*pi/180 );
angles1 = angles(ind);
ranges1 = ranges(ind);
ranges1 = ranges1>0.01 & ranges1<1;
figure(1001)
pax = polaraxes;

polarplot(angles(ind),ranges(ind),'.');
pax.ThetaZeroLocation = 'top';

ranges1(1) = 1;
ranges1(end) =1;

D = bwdist(ranges1);
[pks,locs] = findpeaks(D);
figure(1002)
subplot(3,1,1)
bar(angles(ind)*180/pi,ranges(ind))
subplot(3,1,2)
bar(angles(ind)*180/pi,ranges1)
subplot(3,1,3)
plot(angles(ind)*180/pi,D)
if  sum(ranges1) == length(ind)
    steerAngle = NaN;
    pks = NaN;
else
    steerAngle = angles1(locs);
end
end
