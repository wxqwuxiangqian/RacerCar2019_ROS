function plot_trajectory(data,t)
x=[];
y=[];
z=[];
phi=[];
theta=[];
psi=[];
w=[];
u=[];
v=[];
for i=1:size(data,3)
    x=[x data(3,1,i)];
    y=[y data(3,2,i)];
    z=[z data(3,3,i)];
    u=[u data(2,1,i)];
    v=[v data(2,2,i)];
    w=[w data(2,3,i)];
    phi=[phi data(1,1,i)];
    theta=[theta data(1,2,i)];
    psi=[psi data(1,3,i)];
end
phi=(180/pi)*phi;
theta=(180/pi)*theta;
psi=(180/pi)*psi;

figure,plot3(x,y,z);
grid on
axis on
xlabel('x');
ylabel('y');
zlabel('z');
title('trajectory of the body');
figure,plot(t,phi);
grid on
xlabel('t');
ylabel('phi');
title('phi vs time');
figure,plot(t,theta)
grid on
xlabel('t');
ylabel('theta');
title('theta vs time');
figure,plot(t,psi);
grid on
xlabel('t');
ylabel('psi');
title('psi vs time');
figure,plot(t,u);
grid on
xlabel('t');
ylabel('u');
title('u vs time');
figure,plot(t,v);
grid on
xlabel('t');
ylabel('v');
title('v vs time');
figure,plot(t,w);
grid on
xlabel('t');
ylabel('w');
title('w vs time');