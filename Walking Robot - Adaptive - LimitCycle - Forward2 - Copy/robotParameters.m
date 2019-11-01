% function robotParameters
% Walking Robot Parameters
% Copyright 2017 The MathWorks, Inc.

%% General parameters
density = 1000;
foot_density = 2000;
world_damping = 0.25;
world_rot_damping = 0.25;
if ~exist('actuatorType','var')
    actuatorType = 1;
end

%% Inputs
% gaitPeriod = 0.8;
% time = linspace(0,gaitPeriod,7)';
% ankle_motion = deg2rad([-6 15 15 5 0 -15 -6]');
% knee_motion = deg2rad([25, 15, 5, -10, 2.5, -10, -15, -20, 15, 25]');
% hip_motion = deg2rad([-40, -27.5, -15, -5, 15, 20, 40, 15, -5, -27.5, -40]');
% curveData = createSmoothTrajectory(ankle_motion,knee_motion,hip_motion,gaitPeriod);

% [Traj, time] = learn_rcp_batch(1,ones(1,6));

% %% Contact/friction parameters
% contact_stiffness = 2500;
% contact_damping = 100;
% mu_k = 0.6;
% mu_s = 0.8;
% mu_vth = 0.1;
% height_plane = 0.025;
% plane_x = 25;
% plane_y = 3;
% contact_point_radius = .005;

%% Contact/friction parameters
contact_stiffness = 300;  % 2000   400      70
contact_damping = 2200;   % 10000  4000    2400  
mu_k = 0.5;
mu_s = .6;
mu_vth = 0.1;
height_plane = 0.1;
plane_x = 3;
plane_y = .3;
contact_point_radius = .005;

k=900;   %1000  500
b=2000;
kf=.6;
ks=7;  % 10  7
r=.005;

%% Foot parameters
foot_x = 8;
foot_y = 6;
foot_z = 1;
foot_offset = [-4 0 0];

%% Leg parameters
leg_radius = 3.5;
lower_leg_length = 38;
upper_leg_length = 38;

%% Torso parameters
torso_y = 10;
torso_x = 5;
torso_z = 8;
torso_offset_z = -2;
torso_offset_x = -0.5;
% init_height = 3 + 38 + 38 + ...
%               25 - 12.5 ;
% init_height = 3 + 38 + 38 + .5;
          
%% Joint parameters
joint_damping = 0;
joint_stiffness = 0;
motion_time_constant = 0.01; %0.025;

%% Joint controller parameters
hip_servo_kp = 60;
hip_servo_ki = 10;
hip_servo_kd = 20;
knee_servo_kp = 60;
knee_servo_ki = 5;
knee_servo_kd = 10;
ankle_servo_kp = 20;
ankle_servo_ki = 4;
ankle_servo_kd = 8;
deriv_filter_coeff = 100;
max_torque = 20;

%% Electric motor parameters
motor_resistance = 1;
motor_constant = 0.02;
motor_inertia = 0;
motor_damping = 0;
motor_inductance = 1.2e-6;
gear_ratio = 50;



