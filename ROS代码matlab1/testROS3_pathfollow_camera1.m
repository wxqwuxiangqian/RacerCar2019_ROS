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




% camera1 = rossubscriber('/camera/rgb/image_raw');
% camera2 = rossubscriber('/camera/rgb/image_raw/compressed');

camera1 = rossubscriber('/usb_cam/image_raw');
cmdvelpub = rospublisher('/cmd_vel');

% camera2 = rossubscriber('/usb_cam/image_raw/compressed');
startline = 180;
for i=1:1000
    img1 = receive(camera1);
%     img2 = receive(camera2);
%     showdetails(img1)
    image = readImage(img1);
    lim1= stretchlim(image,[0.02 0.98]);
    image = imadjust(image,lim1);
    [h,w,d] = size(image);
%     [BW,maskedRGBImage] = createMask1(image );
 [BW,maskedRGBImage] = createMask2(image );
    roiBW = BW(startline:end,:);
    se = strel('disk',5);
    BW2 = imclose(roiBW, se);
     se = strel('disk',1);
     BW2 = imopen(BW2, se);
    figure(1)
    subplot(2,2,1)
    imshow(image)
     subplot(2,2,2)
    imshow(roiBW)
    subplot(2,2,3)
    imshow(BW)
    subplot(2,2,4)
    imshow(BW2)
    
    [row,lin]=find(BW2==1);
    
    p = polyfit(row+startline,lin,1);
    row2 = [startline:240]';
    lin2 = polyval(p,row2);
    figure(2)
    hold on
    imagesc(image)
    plot(lin2,row2)
    hold off
    axis ij
 
    diva = lin2(1) - w/2
    if diva <-5
    msg = rosmessage(cmdvelpub);
    msg.Linear.X=0.1;
    msg.Angular.Z = min(45*pi/180,5*pi/180*(1+abs(diva)/20));
    send(cmdvelpub,msg);
    
    elseif diva >5
        msg = rosmessage(cmdvelpub);
    msg.Linear.X=0.1;
    msg.Angular.Z = max(-45*pi/180,-5*pi/180*(1+abs(diva)/20));
    send(cmdvelpub,msg);
    else
         msg = rosmessage(cmdvelpub);
    msg.Linear.X=0.1;
    msg.Angular.Z = 0;
    send(cmdvelpub,msg);
    pause(2)
    end
end