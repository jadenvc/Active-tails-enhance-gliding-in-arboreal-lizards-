%Jaden Clark
%Stanford University
%jvclark@stanford.edu
%7 May 2021

%this file runs the simulation for d maculatus or d fimbriatus

%initial parameters
x=0; %distance
y=5; %height
theta=0*pi/180; %pitch angle
w=0; %angular acceleration
%initial velocity
vx=2.5;
vy=0;
dt=0.001; %timestep
density=1.225; %air density
aa= [0]; %angle of attack (tracked over time)

%angle of attack error
last_error = 0;
error_integral = 0;
tt = 0;
torque = 0;

species = "maculatus"

if species=="maculatus"
    %surface area of d maculatus
    area=0.045;
    %dist from body center of mass to tail center of mass
    tailradius=0.097;
else
    %surface area of d fimbriatus
    area=0.083
    %dist from body center of mass to tail center of mass
    tailradius=0.142
end

%tail angle relative to body
tail_angle=pi/2;
%tail angular acceleration relative to body
tail_w=0;

t=0;
i=1;

%simulate until lizard reaches height of 0
while y(i)>0
    %update time
    i=i+1;
    t = t+dt;
    %update forces/torques
    [drag,lift,aa(i)] = dynamics(vx,vy,density,area,theta(i-1)); 
    if species=="maculatus"
        [vx,vy,w,last_error,aa_desired,error_integral,tt(i),torque(i),tailradius,tail_angle,tail_w] = calcVelocities_maculatus(vx,vy,w,theta(i-1),drag,lift,dt,aa(i),last_error,error_integral,tailradius,tail_angle,tail_w);
    else
        [vx,vy,w,last_error,aa_desired,error_integral,tt(i),torque(i),tailradius,tail_angle,tail_w] = calcVelocities_fimbriatus(vx,vy,w,theta(i-1),drag,lift,dt,aa(i),last_error,error_integral,tailradius,tail_angle,tail_w);
    end
    [x(i),y(i),theta(i)] = dracoPos(x(i-1),y(i-1),theta(i-1),vx,vy,w,dt);
end


figure(1);

%lizard trajectory

plot(x,y, 'Color', 'black', 'LineWidth', 2.5);

line([0,10],[0,0], 'Color','black','LineStyle','--', 'LineWidth',1.5);
hold on;
r=0.5;

%plot arrows to indicate pitch angle
for i=1:length(x)
    if mod(i, 80) == 0
        h1 = quiver(x(i), y(i), r*cos(theta(i)), r*sin(theta(i)), 'Color', 'blue', 'LineWidth', 1);
        set(h1,'AutoScale','on', 'AutoScaleFactor', 2, 'MaxHeadSize', 1)
    end
end


set(gca,'FontSize',17);
set(gcf,'Color','white');
set(gca, 'FontName', 'Helvetica');
xlabel('Horizontal Distance (meters)', 'FontSize', 20);
ylabel('Height (meters)', 'FontSize', 20);

figure(2);

%plot angle of attack and pitch angle over time

plot(theta, 'blue', 'LineWidth', 1.5);
hold on;
plot(aa, 'Color','red', 'LineWidth',1.5);
y2 = aa_desired*180/pi;
%plot desired angle of attack
line([0,length(x)],[.2,.2], 'Color','black','LineStyle','--', 'LineWidth',1.5);

set(gca,'FontSize',12);
set(gcf,'Color','white');
set(gca, 'FontName', 'Helvetica');
legend({'\Theta','\alpha','\alpha desired'}, 'Location', 'northeast', 'FontSize',10);
xlabel('Time (s^{-3})', 'FontSize', 20);
ylabel('Angle (radians)', 'FontSize', 20);



    