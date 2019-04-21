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


odom = rossubscriber('odom')
% odom = rossubscriber('/robot_pose_ekf/odom_combined')

odomdata = receive(odom,100)
showdetails(odomdata)

laser = rossubscriber('/scan');
scan = receive(laser,3)
figure(1001)
plot(scan);
cmdvelpub = rospublisher('/cmd_vel');
%%
path = [0 0;
    0.2 0
    0.3 0 
    0.4 0
    0.8   0.25;
    1 0.5
    1.6 1];
path = [0 0;
    0.5 0.25;
    1 0.5;
    1.5 0.75;
    1.7 1 ];

% path = [0 0;
%     
%     2 1 ];
controller = robotics.PurePursuit;
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.1;
controller.MaxAngularVelocity = 10*pi/180;
controller.LookaheadDistance = 0.5;

spinVelocity = 135*pi/180;       % Angular velocity (rad/s)
forwardVelocity = 0.1;    % Linear velocity (m/s)
backwardVelocity = -0.5; % Linear velocity (reverse) (m/s)
distanceThreshold = 0.2;  % Distance threshold (m) for turning

for i=1:1000
     scan = receive(laser);
      plot(scan);

      data = readCartesian(scan);
      x = data(:,1);
      y = data(:,2);
      
     
      
    odomdata = receive(odom,100);
    X = odomdata.Pose.Pose.Position.X;
    Y = odomdata.Pose.Pose.Position.Y;
    ori1= odomdata.Pose.Pose.Orientation;
    [roll,pitch,yaw] = quat2angle([ori1.X ori1.Y ori1.Z ori1.W]);
    tp = [X Y yaw];
    poseposiT = [X Y yaw*180/pi]
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(tp);
    
    
    
    %%%
     % Compute distance of the closest obstacle
      dist = sqrt(x.^2 + y.^2);
      minDist = min(dist);   
      
       if minDist < distanceThreshold
          % If close to obstacle, back up slightly and spin
          omega = spinVelocity;
          v = backwardVelocity;
            msg = rosmessage(cmdvelpub);
            msg.Linear.X=v;
            msg.Angular.Z = omega;
            send(cmdvelpub,msg);
             pause(1)
             msg = rosmessage(cmdvelpub);
            msg.Linear.X=-1.5*v;
            msg.Angular.Z = 0;
            send(cmdvelpub,msg);
           
            pause(0.01)
            continue;
            
       else  
            [v, omega] = controller(tp);
    
       end   
       
    % drive the robot using the controller outputs.
    %%%车动一下
    
    
    
    msg = rosmessage(cmdvelpub);
    msg.Linear.X=v;
    msg.Angular.Z = omega;
    send(cmdvelpub,msg);
    fprintf('[X Y YAW]=[%.2f,%.2f,%2f],[V,omega] =[%.2f,%.2f]',X,Y,yaw*180/pi,v,omega*180/pi)
    pause(0.01)
    % Extract current location information ([X,Y]) from the current pose of the
    % robot
    
    dist = norm([X Y] - path(end,:));
    if dist<0.2
        break;
    end
    
end
