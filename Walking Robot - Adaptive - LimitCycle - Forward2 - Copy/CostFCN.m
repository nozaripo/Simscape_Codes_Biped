function [penalty, CoT, pitch_err, Constraint_viol, Constraint_viol_indiv] = CostFCN(params,mdlName, V_treadmill)

% Cost function for robot walking optimization


ind0 = 35;
params(1,1) = ind0+7*params(1,1);

%% Decision Variables (They could be defined here and unused but they will be used by Simscape)
ts0 = ceil(params(1,1));
tau = params(1,2);
Am = params(1,3:end);
Am = repmat(Am,1,6/length(Am));

%% DMP and config based on initial config variable "ts0"
[DMP, W, ~, Ym, tau_nom, dt, time, Tra, F, init_pos, init_vel] = learn_rcp_batch(ts0);

Traj = Am.*init_pos'*pi/180;
Ym = Ym.*Am;
init_pos = Am'.*init_pos;
init_vel = Am'.*init_vel;



%% find the correct inital torso pitch and height such that the initial config is right toe-off
options = optimoptions('fsolve','Display','off','FunctionTolerance',1e-10,'MaxFunctionEvaluations',1000);

q0 = 0;

fun = @(q)torso_pitch(q, Traj);
    [q, fval] = fsolve(fun,q0,options);

theta0 = q;


[~, init_height] = torso_pitch(theta0,Traj);



%% run the simulation with the indivduals variables and the dependent parameters
simout = sim(mdlName,'SrcWorkspace','current');


%% evaluate the outputs of the simulation
x = simout.x0;
z = simout.z0;
torso_theta = simout.torso_theta;
l_ank = simout.left_ankle;
l_kn  = simout.left_knee;
l_hip = simout.left_hip;
r_ank = simout.right_ankle;
r_kn  = simout.right_knee;
r_hip = simout.right_hip;

WorkEnd = simout.Work(end);

transport = simout.pos_rel(end);


%% the term is used to penalize the conditions where the transport is negative (falling etc)
% this is used in CoT definition
transport_penalty = 1e-3 + max(0,transport);

% Cost of Transport normalized by distance travelled
CoT = WorkEnd/transport_penalty;



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Penalty (the main output of this file that is used for ga optimization)
penalty = CoT + 10*norm(torso_theta(1)-torso_theta(end));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%






%% additional outputs of this file used to evaluate the constraint violations later
pitch_err = torso_theta(end)-torso_theta(1);

Constraint_viol = sqrt( (100*(x(1)-x(end)))^2+(l_hip(1)-r_hip(end))^2);

Constraint_viol_indiv = -[x(1)-x(end);
    z(1)-z(end);
    torso_theta(1)-torso_theta(end);
    l_ank(1)-r_ank(end);
    l_kn(1)-r_kn(end);
    l_hip(1)-r_hip(end);
    r_ank(1)-l_ank(end);
    r_kn(1)-l_kn(end);
    r_hip(1)-l_hip(end)];


clear simout
% Simulink.sdi.clear

end



function [F, Z_L, Z_R] = torso_pitch(q, Traj)

theta0 = q;


Z_L = max(-12.5*sin(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3)) , +4.5*sin(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3))) + ...
    38*( cos(Traj(1,1)+theta0) + cos(Traj(1,1)+theta0-Traj(1,2)) ) + 4*cos(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3));

Z_R = max(12.5*sin(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6)) , -4.5*sin(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6)))+...
    38*( cos(-Traj(1,4)-theta0) + cos(-Traj(1,4)-theta0+Traj(1,5)) ) + 4*cos(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6));


% Z_L = 4.5*sin(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3)) + ...
%     38*( cos(Traj(1,1)+theta0) + cos(Traj(1,1)+theta0-Traj(1,2)) ) + 4*cos(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3));
% 
% Z_R = 12.5*sin(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6))+...
%     38*( cos(-Traj(1,4)-theta0) + cos(-Traj(1,4)-theta0+Traj(1,5)) ) + 4*cos(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6));


F(1) = Z_L-Z_R;

end
