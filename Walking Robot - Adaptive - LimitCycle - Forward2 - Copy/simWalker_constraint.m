function [C, Ceq] = simWalker_constraint(params,mdlName,V_treadmill)
% Cost function for robot walking optimization
% Copyright 2017 The MathWorks, Inc.

% Load parameters into function workspace
% params(1,1) = 31+56*params(1,1);
% params(1,1) = 25+10*params(1,1);
ind0 = 35;
% ind0 = 55;
params(1,1) = ind0+7*params(1,1);
ts0 = ceil(params(1,1));
% ts0 = params(1,1);

youBot_PARAM
robotParameters;

[DMP, W, Am, Ym, tau_nom, dt, time, Tra, F, init_pos, init_vel] = learn_rcp_batch(ts0);
% Apply variable scaling
%     Traj = [-30 0 -2 30 0 0]*pi/180;
%     params = scaleFactor*params;

% % Traj = init_pos'*pi/180;

tau = params(1,2);


Am = params(1,3:end);
Am = repmat(Am,1,6/length(Am));

% theta0 = params(1,4);


tSim = (.57/tau_nom)*tau;


Traj = Am.*init_pos'*pi/180;

Ym = Ym.*Am;

init_pos = Am'.*init_pos;
init_vel = Am'.*init_vel;


options = optimoptions('fsolve','Display','off','FunctionTolerance',1e-10,'MaxFunctionEvaluations',1000);

q0 = 0;

fun = @(q)torso_pitch(q, Traj);
    [q, fval] = fsolve(fun,q0,options);

theta0 = q;


[~, init_height] = torso_pitch(theta0,Traj);

% init_height = 15*sin(Traj(1,3)*pi/180) + ...
%     38*( cos(Traj(1,1)*pi/180) + cos(Traj(1,2)*pi/180) ) + 2.5+...
%     25 - 12.5-12.5 ;      % initial height of the torso (doesn't matter here)
% init_height = max(12.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3)) , -4.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3))) + ...
%     38*( cos(-Traj(1,1)-theta0) + cos(-Traj(1,1)-theta0+Traj(1,2)) ) + 3*cos(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3));


% theta0 = -.04;

% .2
% % % % init_height = max(12.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3)) , -4.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3))) + ...
% % % %     38*( cos(-Traj(1,1)-theta0) + cos(-Traj(1,1)-theta0+Traj(1,2)) ) + 3*cos(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3)) ;

% init_height = max(12.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3)) , -4.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3))) + ...
%     38*( cos(-Traj(1,1)-theta0) + cos(-Traj(1,1)-theta0+Traj(1,2)) ) + 3*cos(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3));


%     % Simulate the model
% simout = sim(mdlName,'SrcWorkspace','current','FastRestart','on');
simout = sim(mdlName,'SrcWorkspace','current');

%
%     % Unpack logged data
%     h = min(simout.yout{3}.Values.Data);


% C = [(x(1)-x(end))^2+(z(1)-z(end))^2];

%     C = [-(h - 80)];


x = simout.x0;
z = simout.z0;
torso_theta = simout.torso_theta;
l_ank = simout.left_ankle;
l_kn  = simout.left_knee;
l_hip = simout.left_hip;
r_ank = simout.right_ankle;
r_kn  = simout.right_knee;
r_hip = simout.right_hip;


transport = simout.pos_rel(end);

% C = [(x(1)-x(end))^2-.1;
%     (z(1)-z(end))^2-1;
%     (torso_theta(1)-torso_theta(end))^2-1;
%     (l_ank(1)-r_ank(end))^2-1;
%     (l_kn(1)-r_kn(end))^2-1;
%     (l_hip(1)-r_hip(end))^2-1;
%     (r_ank(1)-l_ank(end))^2-1;
%     (r_kn(1)-l_kn(end))^2-1;
%     (r_hip(1)-l_hip(end))^2-1];

C = [];
% C = (x(1)-x(end))^2-.1+...
%     (z(1)-z(end))^2+...
%     (torso_theta(1)-torso_theta(end))^2+...
%     (l_ank(1)-r_ank(end))^2+...
%     (l_kn(1)-r_kn(end))^2+...
%     (l_hip(1)-r_hip(end))^2+...
%     (r_ank(1)-l_ank(end))^2+...
%     (r_kn(1)-l_kn(end))^2+...
%     (r_hip(1)-l_hip(end))^2-30;

% C = ConstraintViol-50;

%    c = [1.5 + x(1)*x(2) + x(1) - x(2);
%    -x(1)*x(2) + 10];
%     Ceq = [x(1)-x(end);z(1)-z(end)];

% Ceq = [];
% % % Ceq = 5*(x(1)-x(end))^2+...
% % %     (z(1)-z(end))^2+...
% % %     (torso_theta(1)-torso_theta(end))^2+...
% % %     (l_ank(1)-r_ank(end))^2+...
% % %     (l_kn(1)-r_kn(end))^2+...
% % %     (l_hip(1)-r_hip(end))^2+...
% % %     (r_ank(1)-l_ank(end))^2+...
% % %     (r_kn(1)-l_kn(end))^2+...
% % %     (l_hip(1)-r_hip(end))^2;


Ceq = [100*(x(1)-x(end));
    l_hip(1)-r_hip(end);
    l_kn(1)-r_kn(end);
    z(1)-z(end);
    (torso_theta(1)-torso_theta(end))];

% Ceq = [100*norm(x(1)-x(end))-1 + norm(l_hip(1)-r_hip(end))];

% C = [norm(100*(x(1)-x(end)))+...
%     norm(l_hip(1)-r_hip(end))+...
%     100*norm(torso_theta(1)-torso_theta(end))+...
%     norm(z(1)-z(end))-5];

clear simout
% Simulink.sdi.clear

end


function [F, Z_L, Z_R] = torso_pitch(q, Traj)

theta0 = q;

% 
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
