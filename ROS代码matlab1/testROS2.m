clear all;
close all;
% Connect to a ROS Network,http://www.mathworks.com/help/robotics/examples/connect-to-a-ros-network.html
% rosinit('http://192.168.1.113:12000')
% setenv('ROS_MASTER_URI','http://192.168.1.113:11311')
% setenv('ROS_IP','192.168.1.113')
% rosinit
%roslaunch rikirobot bringup.launch
% rosshutdown
clear all
rosshutdown;
rosinit('192.168.1.112')
rosnode list
rostopic list
rosaction list
%roslaunch rikirobot bringup.launch
%roslaunch rikirobot lidar_slam.launch
%roslaunch rikirobot navigate.launch

%step 1
maxRange = 2; % meters
resolution = 10; % cells per meter
slamObj = robotics.LidarSLAM(resolution,maxRange);
laser = rossubscriber('/scan');
%step2
cmdvelpub = rospublisher('/cmd_vel');
cmdvelpub = rospublisher('/cmd_vel');
msg = rosmessage(cmdvelpub);

%%
%%%%%开始前建立地图初始地图,以及规划路径（不做）
    if 1
   
        for i = 1:10
        scandata = receive(laser,10);
        figure(1),
        plot(scandata,'MaximumRange',2)
%         set(gca,'YDir','reverse')
        cart = readCartesian(scandata);
        scandata1 = lidarScan(cart);
        addScan(slamObj,scandata1);

        [scansSLAM,poses] = scansAndPoses(slamObj);
        occGrid = buildMap(scansSLAM,poses,resolution,maxRange);

        figure(2)
        show(occGrid)
        view(-90, 90)
        set(gca,'YDir','reverse')
        title('Occupancy Map of Garage');


        %%%车动一下
        cmdvelpub = rospublisher('/cmd_vel');
        msg = rosmessage(cmdvelpub);
        msg.Linear.X=1*0.1;
        msg.Angular.Z = 0;
        send(cmdvelpub,msg);
        pause()
        end
        mat = occupancyMatrix(occGrid);
        mat2 = mat>0.5;
        a= occGrid.GridLocationInWorld;
        map2 =robotics.BinaryOccupancyGrid(mat2,10);
        map2.GridLocationInWorld =a;
        show(map2)
        prm = robotics.PRM(map2,100);
        show(prm, 'Map', 'off', 'Roadmap', 'off');
        path = findpath(prm,[0 0],[1 0.3]);

    end

   
 return;
%%
path =[0.00    0.00; 0.1 0;0.2 0.1;1 0.3]
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotCurrentLocation initialOrientation];

goalRadius = 0.1;
distanceToGoal = norm(robotCurrentLocation - robotGoal);

controlRate = robotics.Rate(10);
controller = robotics.PurePursuit
controller.Waypoints = path;
controller.DesiredLinearVelocity = 0.1;
controller.MaxAngularVelocity = 1;
controller.LookaheadDistance = 0.5;
odom = rossubscriber('/odom');
pose = receive(odom,10);

while( distanceToGoal > goalRadius )
%     scandata = receive(laser,10);
    pose = receive(odom,10);
    x= pose.Pose.Pose.Position.X;
    y= pose.Pose.Pose.Position.Y;
    theta = pose.Pose.Pose.Orientation.Z;
    [v, omega] = controller([x y theta]);
    msg.Linear.X=v;
    msg.Angular.Z = omega;
    send(cmdvelpub,msg);
    % Re-compute the distance to the goal
    pose = receive(odom,10);
    x= pose.Pose.Pose.Position.X;
    y= pose.Pose.Pose.Position.Y;
    distanceToGoal = norm([x y] - robotGoal);
    pause()

    
    %一边走一边建立地图
    scandata = receive(laser,10);
    figure(1),
    subplot(2,1,1)
    plot(scandata,'MaximumRange',2)
    set(gca,'YDir','reverse')
    cart = readCartesian(scandata);
    scandata1 = lidarScan(cart);
    addScan(slamObj,scandata1);

    [scansSLAM,poses] = scansAndPoses(slamObj);
    occGrid = buildMap(scansSLAM,poses,resolution,maxRange);
    
    %一边走一边规划路径
end

%%
% [actClient,goalMsg] = rosactionclient('/move_base');
% goalMsg.TargetPose.Header.FrameId ='base_link'
%  goalMsg.TargetPose.Header.Stamp = rostime('now','system');
% goalMsg.TargetPose.Pose.Position.X = 1;
% goalMsg.TargetPose.Pose.Orientation.W =1;
% rvlmsg = sendGoalAndWait(actClient,goalMsg)