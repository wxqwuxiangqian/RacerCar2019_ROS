
close all;


load lidarScen1.mat


scandata = readCartesian(scan);
x = scandata(:,1);
y = scandata(:,2);




X = odomdata.Pose.Pose.Position.X;
Y = odomdata.Pose.Pose.Position.Y;
ori1= odomdata.Pose.Pose.Orientation;
[roll,pitch,yaw] = quat2angle([ori1.X ori1.Y ori1.Z ori1.W]);
tp = [X Y yaw];
poseposiT = [X Y yaw*180/pi]

[steerAngle,pks] = simpleRangeHist(scan)


function [steerAngle,pks] = simpleRangeHist(scan)
ranges = double(scan.Ranges);
angles = double(scan.readScanAngles);
ind = find(angles<90*pi/180 & angles>-90*pi/180 );
angles1 = angles(ind);
ranges1 = ranges(ind);
ranges1 = ranges1>0.01 & ranges1<0.5;
figure(1001)
pax = polaraxes;

polarplot(angles(ind),ranges(ind),'.');
pax.ThetaZeroLocation = 'top';
figure
subplot(2,1,1)
bar(angles(ind)*180/pi,ranges(ind))
subplot(2,1,2)
bar(angles(ind)*180/pi,ranges1)

ranges1(1) = 1;
ranges1(end) =1;

D = bwdist(ranges1);
[pks,locs] = findpeaks(D);

if  sum(ranges1) == length(ind)
    steerAngle = NaN;
    pks = NaN;
else
    steerAngle = angles1(locs);
end
end

