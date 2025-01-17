% function robotParameters
% Walking Robot Parameters
% Copyright 2017 The MathWorks, Inc.

%% General parameters
% -------------------------------------------------------------------
%  Generated by MATLAB on 25-Nov-2019 11:55:25
%  MATLAB version: 9.4.0.949201 (R2018a) Update 6
% -------------------------------------------------------------------
                                          

density = 1000;           % kg/m^3
foot_density = 2000;      % kg/m^3
world_damping = 0.25;     % N.s/m
world_rot_damping = 0.25; % N.s/rad
    actuatorType = 1;

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
contact_stiffness = 300;     % N/m   % 2000   400      70
contact_damping = 2200;      % N.s/m % 10000  4000    2400  
mu_k = 0.5;                  % Dynamic Coeff of Friction
mu_s = .6;                   % Static Coeff of Friction
mu_vth = 0.1;                % Speed Threshold for Friction
height_plane = 0.1;          % Height of the treadmill (m)
plane_x = 3;                 % Length of the treadmill (m)
plane_y = .3;                % Width of the treadmill (m)
contact_point_radius = .005; % m

k=900;   % N/m   %1000  500
b=2000;  % N.s/m 
kf=.6;   % Dynamic Coeff of Friction
ks=7;    % Static Coeff of Friction  10  7
r=.005;  % m


%% Leg parameters
lower_leg_radius = 3.1;       % cm
lower_leg_length = 38;  % cm
lower_leg_mass = 2.87;  % kg
lower_leg_CM_offset=2.55; % cm
lower_leg_inertia = [.0717 .0717 .0001];  

upper_leg_radius = 3.5;       % cm
upper_leg_length = 38;  % cm
upper_leg_mass = 4.7;  % kg
upper_leg_CM_offset=2.55; % cm
upper_leg_inertia = [.0708 .0708 .0001]; 

%% Torso parameters
torso_dim = [6 30 5];   % cm
torso_mass= 27.17;      % kg
torso_offset_z = -2;    % cm
torso_offset_x = -0.5;  % cm
torso_to_hip   = 11;    % cm
% init_height = 3 + 38 + 38 + ...
%               25 - 12.5 ;
% init_height = 3 + 38 + 38 + .5;

foot_dim = [17 7 4];
foot_CoM = [-2 0 0];
foot_mass = 1;
foot_offset = [-4 0 0];

ankle_to_midLow = 19;
midLow_to_knee  = 19;
knee_to_midUpp  = 19;
midUpp_to_hip   = 19;

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


%% Belt parameters
YBT_Par.BeltIn.length = 3;  % m
% YBT_Par.BeltIn.width  = 1;  % m
YBT_Par.BeltIn.width  = .4;  % m
YBT_Par.BeltIn.height = 0.2; % m
YBT_Par.BeltIn.depth  = 0.1; % m
YBT_Par.BeltIn.spd    = 0.1;  % m/s
YBT_Par.BeltIn.dist_from_robot  = 0.305; % m
YBT_Par.BeltIn.offset = [0 YBT_Par.BeltIn.dist_from_robot+YBT_Par.BeltIn.length/2 YBT_Par.BeltIn.height-YBT_Par.BeltIn.depth/2]; % m
YBT_Par.BeltIn.Control.Delay_on = 0.2;  % sec - perhaps move to logic
YBT_Par.BeltIn.Roller.color = [1 1 1];  % RGB

YBT_Par.BeltOut = YBT_Par.BeltIn;

YBT_Par.Cube.d = 6e-2;                         % m
YBT_Par.Cube.Con.rSph = YBT_Par.Cube.d*5e-2;   % m

%% Electric motor parameters
motor_resistance = 1;
motor_constant = 0.02;
motor_inertia = 0;
motor_damping = 0;
motor_inductance = 1.2e-6;
gear_ratio = 50;


initAnglesR = Simulink.Parameter;
initAnglesR.Value = [];
initAnglesR.CoderInfo.StorageClass = 'Auto';
initAnglesR.Description = '';
initAnglesR.DataType = 'auto';
initAnglesR.Min = [];
initAnglesR.Max = [];
initAnglesR.DocUnits = '';

