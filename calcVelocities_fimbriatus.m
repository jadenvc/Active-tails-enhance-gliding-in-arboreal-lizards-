function [vx,vy,w,error,aa_desired,error_integral,tt,torque,tailradius_cur,tail_angle_cur,tail_w_cur] = calcVelocities_fimbriatus(vxprev,vyprev,wprev,thetaprev,drag,lift,dt,aa,last_error,error_integral,tailradius_prev,tail_angle_prev,tail_w_prev)
    %Jaden Clark
    %Stanford University
    %jvclark@stanford.edu
    %this file contains code to compute x/y velocity and angular velocity/acceleration
    %of Draco fimbriatus
    %7 May 2021
    
    %lizard faces right for angle measurements
    aa_desired = 12*pi/180;
    gamma = angleDiff(thetaprev - aa);

    % draco fimbriatus
    l=0.1050; %svl of lizard
    m=0.0173; %mass of lizard
    tailm=0.0012025; %tail mass
    tail_length=0.18; %tail length

    %set control gains
    kp=0.0;
    kd=0.005;
    ki=0.001;
    %calculate errors
    error = aa-aa_desired;
    error_dot = (error - last_error)/dt;
    error_integral = error_integral+error;
    %bound error_integral
    if error_integral > 6.28
        error_integral = 6.28;
    end
    if error_integral < -6.28
        error_integral = -6.28;
    end

    %set new tail angle
    tail_angle_cur = tail_angle_prev + kp*angleDiff(error) + kd*error_dot + ki*(error_integral);

    %set max tail speed (bounded at speed derived from (Jusufi et al., 2010))
    max_tail_disp = .05;
    if tail_angle_cur-tail_angle_prev>max_tail_disp
        tail_angle_cur=tail_angle_prev+max_tail_disp;
    end
    if tail_angle_cur-tail_angle_prev< -max_tail_disp
        tail_angle_cur=tail_angle_prev-max_tail_disp;
    end

    %bound tail angle at  perpendicular to body
    if tail_angle_cur > pi/2
        tail_angle_cur = pi/2;
    end
    if tail_angle_cur < -pi/2
        tail_angle_cur=-pi/2;
    end

    I = 1/12*(m*l*l);%inertia of lizard (modeled as a rod)

    %calculate distance from tail cm to body cm
    cm_to_tail=0.46; %dist from cm to base of tail
    tailradius_cur = (tail_length/2) * cos(tail_angle_cur-thetaprev) + cm_to_tail*l*cos(thetaprev);

    %torque from tail weight force
    tt = (tailradius_cur * tailm * 9.81);

    %calc torque from tail movement
    tail_w_cur = (angleDiff(tail_angle_cur-tail_angle_prev))/dt;
    tail_angacc = (tail_w_cur-tail_w_prev)/dt;
    tail_torque = tail_angacc*(1/12)*tailm*tail_length*tail_length;

    %incorporate tail angle into center of mass
    radius = .0225;%distance from center of mass to center of pressure (tail at 180degrees)
    max_disp=.0096; % displacement of center of mass w/perpendicular tail
    radius = radius - max_disp*cos(tail_angle_cur-thetaprev);

    %total torque on body
    torque = (radius * cos(aa) * (-lift)) + (radius * sin(aa)*-drag) + (tt) - tail_torque;

    %sum forces
    ax = (-drag*cos(gamma)/(tailm+m))-(lift*sin(gamma)/(tailm+m));
    yforces = -9.81 * (m + tailm) + cos(gamma)*lift - sin(gamma)*drag;
    ay = yforces / (m+tailm);
    angacc = torque/I;

    vx = vxprev + ax*dt;
    vy = vyprev + ay*dt;
    w = wprev + angacc*dt;
end


