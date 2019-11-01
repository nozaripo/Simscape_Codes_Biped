youBot_PARAM;
robotParameters;

ts0 = 31;   % initial time point (indicates starting phase of the trajectories)

[DMP, W, Am, Ym, tau_nom, dt, time, Traj, F, init_pos, init_vel] = learn_rcp_batch(ts0);
% % % [DMP, W1, Am, Ym1, tau_nom, dt, time, Traj, F, init_pos1, init_vel1] = learn_rcp_batch(ts0);
% % % [DMP, W2, Am, Ym2, tau_nom, dt, time, Traj, F, init_pos2, init_vel2] = learn_rcp_batch(ts0);
% DMP: a structure containing DMP info of all DOFs
% W  : a weight matrix each column of which indicates the weight vector
%      corresponding to each DOF
% 

% Traj = Traj*pi/180; % initial configurations from the demonstrations
Traj = init_pos'*pi/180;

tau = tau_nom;      % gait period to be manipulated (here set the same as of demonstrations)

theta0 = .02;       % initial torso pitch in rad (CCW+ / CW-)

tSim = (.57/tau_nom)*tau; % simulation time



options = optimoptions('fsolve','Display','off','FunctionTolerance',1e-10,'MaxFunctionEvaluations',1000);

q0 = 0;

fun = @(q)torso_pitch(q, Traj);
    [q, fval] = fsolve(fun,q0,options)

theta0 = q;




% init_height = 15*sin(Traj(1,3)*pi/180) + ...
%     38*( cos(Traj(1,1)*pi/180) + cos(Traj(1,2)*pi/180) ) + 2.5+...
%     25 - 12.5-12.5 ;      % initial height of the torso (doesn't matter here)
init_height = max(12.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3)) , -4.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3))) + ...
    38*( cos(-Traj(1,1)-theta0) + cos(-Traj(1,1)-theta0+Traj(1,2)) ) + 3*cos(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3));

% init_height = max(12.5*sin(-Traj(1,1)+Traj(1,2)-Traj(1,3)) , -4.5*sin(-Traj(1,1)+Traj(1,2)-Traj(1,3))) + ...
%     38*( cos(-Traj(1,1)) + cos(-Traj(1,1)+Traj(1,2)) ) + 3*cos(-Traj(1,1)+Traj(1,2)-Traj(1,3));


function F = torso_pitch(q, Traj)

theta0 = q;


Z_L = max(12.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3)) , -4.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3)))+...
    38*( cos(-Traj(1,1)-theta0) + cos(-Traj(1,1)-theta0+Traj(1,2)) ) + 3*cos(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3));

Z_R = max(12.5*sin(Traj(1,4)+theta0 +Traj(1,5)-Traj(1,6)) , -4.5*sin(Traj(1,4)+theta0 +Traj(1,5)-Traj(1,6)))+...
    38*( cos(Traj(1,4)+theta0) + cos(Traj(1,4)+theta0+Traj(1,5)) ) + 3*cos(Traj(1,4)+theta0+Traj(1,5)-Traj(1,6));

F(1) = Z_L-Z_R;

end