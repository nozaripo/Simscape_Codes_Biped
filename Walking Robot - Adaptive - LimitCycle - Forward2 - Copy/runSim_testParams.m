% This file is to generate arbitrary values for the variables that might
% change during the simulation. This is used to give the Simscape
% simulation only a run.

theta0      = 0;    % initial torso pitch
init_height = 77;   % initial torso height
ts0         = 38;   % initial config timepoint
[DMP, W, Am, Ym, tau, dt, time, Traj, F, init_pos, init_vel, DMP_traj] = learn_rcp_batch(ts0);  % run DMP
Am          = ones(1,6);
V_treadmill = 1;    % treadmill Velocity
actuatorType = 1;  % actuator type


