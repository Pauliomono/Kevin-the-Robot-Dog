clear
clc
close all


L = 124;

% static gait - feet moving up & down w/out x & y offsets, fixed speed
% gait split into 8 chunks (fixed chunk time)
% to move forward/back/strafeLR/yawLR add x & y multiplier
% this gives open loop control
% all of the above is calculated once per chunk
% the below is repeated until chunk is completed

% offset acceleration to control motion? uses IMU to drive
% forward/back/strafeLR/yawLR (later implementation much smarter)

% IMU gives xy accel data for COM - use to 
%- adds offsets based on PID loop (needs to play nice with open loop
%offsets) (doesn't affect foot when its off the ground)
% this control loop gives xyz desired values - feed this into inverse kinematics to get servo angles
% heres the tricky part - want accurate pos/velocity of foot
% interpolation! - method below works, but only if the foot stops @ every
% point, no via_points



rx = -20;
ry = 0;
rz = 180;
%inverse kinematics
t_s1f = atan2d(ry,rz) 
r = sqrt(rz^2 + rx^2);
t_kf = acosd((L^2 + L^2 - r^2)/(2*L*L))
t_s2f = acosd((L^2 + r^2 - L^2)/(2*r*L)) - atan2d(rx,rz)


%trajectory calc
t_s10 = 0;
t_s20 = 0;
t_kf = 0;


%deltat = delta_pos/rate; informs tf for a desired rate
% theta_dot = (d_theta)/dt; %per loop

% function [theta] = evaluateCubicSpline(theta0, thetaf, theta_dot0, theta_dotf t0, tf, t)
%     tf = tf-t0; % modify tf and t so you can treat everything as starting at t0=0 s
%     t  = t-t0;
%     
%     %compute a0-a3
%     a0 =  theta0;
%     a1 =  0;
%     a2 =  3*(thetaf - theta0)/tf^2;
%     a3 = -2/tf^3*(thetaf - theta0);
%     
%     theta = a0 + a1*t + a2*t^2 + a3*t^3;
% end










