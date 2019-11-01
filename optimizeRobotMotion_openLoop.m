% Main script for optimizing the gait of a walking robot model
% Copyright 2017 The MathWorks, Inc.

%% Set initial parameters
clear all

% Choose model name
mdlName = 'walkingRobot_Forward2'; % Main model

% Flags to speed up simulation
accelFlag = false;
parallelFlag = true;

% Joint actuator type for optimization
% 1 = motion | 2 = torque | 3 = motor
actuatorType = 1;

% To reduce the search space, scale the angle waypoints and solve the
% optimization algorithm with integer parameters
scalingFactor = 1;        % Scaling from degrees to integer parameters

% Number of decision variables
% nDOF  = 6;
% nGrid = 10;
% numPoints = nDOF*nGrid;              % Number of joint angle points




%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% % % % LimitCycle_Amp_split_tri-amp
%%
% upperBnd = [1.5,20,.5     1.5,20,.5   1.5,20,.5];
% lowerBnd = [.001, 0, 0    .001, 0, 0   .001, 0, 0];
%

% % % upperBnd = [0,20*ones(1,nGrid-2),-2    0,45*ones(1,nGrid-2),0   20,40*ones(1,nGrid-2),-30  ...
% % %             -2,20*ones(1,nGrid-2),0    0,45*ones(1,nGrid-2),0   -20,40*ones(1,nGrid-2),30] *pi/180;
% % %         
% % % lowerBnd = [0,-20*ones(1,nGrid-2),-2   0,zeros(1,nGrid-2),0     20,-40*ones(1,nGrid-2),-30  ...
% % %             -2,-20*ones(1,nGrid-2),0   0,zeros(1,nGrid-2),0   -20,-40*ones(1,nGrid-2),30] *pi/180;

% lowerBnd = [0     0.3   0.5   -pi/15];
% upperBnd = [1  2.5   2.0   +pi/15];

Amp_no = 3;

lowerBnd = [0     0.3    0.6 0.6 0.60];
upperBnd = [1     1.50   1.40 1.40 1.30];

IntCon = [1];
numPoints = size(lowerBnd,2);

% p0 = lowerBnd+(upperBnd-lowerBnd).*rand(1,numPoints);  % Create zero motion initial conditions
p0 = [.4 .45 1 1 1];
p01= [.7  .7 .8 .8 .7];

%% Set optimization options
opts = optimoptions('ga');
opts.Display = 'iter';
opts.OutputFcn= @outputFCN;
% opts.FitnessLimit = 1;
opts.ConstraintTolerance = 1e-3;
opts.MaxGenerations = 7;
opts.PopulationSize = 25;
% opts.MutationRate = 1/numPoints;
% opts.Crossover = .8;

% opts.InitialPopulationMatrix = p0 + .5*(upperBnd-p0).*rand(opts.PopulationSize,numPoints) + .2*(p0-lowerBnd).*rand(opts.PopulationSize,numPoints);
r = randi([2 3],1);
r1 = randi([2 3],1);
opts.InitialPopulationMatrix = [lowerBnd+(upperBnd-lowerBnd).*rand(opts.PopulationSize-r1-r,numPoints); repmat(p0,[r1 1]); repmat(p01,[r 1])];% Add copies of initial gait

opts.PlotFcn = {@gaplotbestf, @gaplotmaxconstr @outputFCN}; % Add progress plot of fitness function
% opts.PlotFcn = {@gaplotbestf}; % Add progress plot of fitness function
opts.UseParallel = parallelFlag;
opts.UseVectorized = false;
        
output = zeros(opts.MaxGenerations,numPoints);





%% fmincon options
% % % opts = optimoptions('fmincon');
% % % opts.Display = 'iter';
% % % opts.Algorithm = 'sqp';
% % % opts.UseParallel = parallelFlag;
% % % opts.PlotFcn = {@optimplotx, @optimplotfval, @optimplotconstrviolation};
% guess.states = lowerBnd+(upperBnd-lowerBnd).*rand(1,numPoints);


% 
for k = 4:6

V_treadmill = 0.3 + (1.7/5)*(k-1);

%% Run commands to set up parallel/accelerated simulation
doSpeedupTasks;

%% Run optimization
costFcn = @(p)CostFCN(p,mdlName,V_treadmill);
% nonlcon = @(p)GAconstraint(p,mdlName,scalingFactor,gaitPeriod,actuatorType); % Constraint for height

nonlcon = @(p)simWalker_constraint(p,mdlName,V_treadmill); % Constraint for height

tic



%% solve by ga
disp(['Running optimization! Population Size: ' num2str(opts.PopulationSize) ...
      ', Fitness Limit: ' num2str(opts.FitnessLimit) ...
      ', Generations No: ' num2str(opts.MaxGenerations)])
   
  
[pFinal,penalty,exitflag] = ga(costFcn,numPoints,[],[],[],[], ... 
                     lowerBnd,upperBnd, ...
                     nonlcon,opts);
                 

%  [x,fval,exitflag,output,population,scores]                
% % % [pFinal,penalty,exitflag] = ga(costFcn,numPoints,[],[],[],[], ... 
% % %                      lowerBnd,upperBnd, ...
% % %                      [],[],opts);                 
                 % IntCon []
                 
% [pFinal,penalty,exitflag,output,population,scores] = gamultiobj(costFcn,numPoints,[],[],[],[], ... 
%                      lowerBnd,upperBnd, ...
%                      nonlcon,opts);                 

%% solve by fmincon
% % % [pFinal,penalty,exitflag] = fmincon(costFcn,p0,[],[],[],[], ...
% % %                         lowerBnd,upperBnd, ...
% % %                         nonlcon,opts);                 
                 
disp(['Final penalty function value: ' num2str(penalty)])

% pFinal(1,1) = 25+10*pFinal(1,1);
pFinal(1,1) = 38+7*pFinal(1,1);
% pFinal(1,1) = 31+56*pFinal(1,1);
%%
    ts0 = ceil(pFinal(1,1));    
%     ts0 = pFinal(1,1); 
    tau = pFinal(1,2);
    
    Am = pFinal(1,3:end);
    
    
    Am = repmat(Am,1,6/length(Am));
%     theta0 = pFinal(1,4);
    
    tau_nom=1.13;
    tSim = (.57/tau_nom)*tau;

[DMP, W, ~, Ym, tau_nom, dt, time, Tra, F, init_pos, init_vel] = learn_rcp_batch(ts0);

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

% init_height = max(12.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3)) , -4.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3))) + ...
%     38*( cos(-Traj(1,1)-theta0) + cos(-Traj(1,1)-theta0+Traj(1,2)) ) + 3*cos(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3));
% 

for j = 1:size(GenBest,1)
    GenBest_vel(:,:,k) = GenBest'
    j
    [penalty_bestGen(j,k), Work_bestGen(j,k), pitch_err, Constraint_viol, Constraint_viol_indiv] = CostFCN(GenBest(j,:),mdlName,V_treadmill);
    Constraint_viol_bestGen(j,k)=Constraint_viol;
    pitch_err_bestGen(j,k)=pitch_err;
    Constraint_viol_indiv_bestGen(j,:,k)=Constraint_viol_indiv;
%     init_ht_sym(j,1) = z_lh(1,1)-z_rt(1,1);
%     final_stop(j,1)  = z_rh(1,end)-5.01;
    clear z_lh z_rt z_lt z_rh
end








% init_height = (15*sin(Traj(1,3)*pi/180) + ...
%     38*( cos(Traj(1,1)*pi/180) + cos(Traj(1,2)*pi/180) ) + 3 )*cos(theta0);
%%

el_time = toc/3600;
el_time = [num2str(floor(el_time)), ' hrs & ', num2str(floor(60*(el_time-floor(el_time)))), ' mins']

end
%% Cleanup
% bdclose(mdlName);

% [Traj, time, ta, init_height] = run_rcp(pFinal(1,1),pFinal(1,2:end));

% Data=[time, Traj];
% save('Dataaaa.mat','Data','ta','init_height')

% if parallelFlag
%    delete(gcp('nocreate')); 
% end

% save('pFinal.mat','K_h', 'Ki_h', 'Kd_h','K_k', 'Ki_k', 'Kd_k', 'K_a', 'Ki_a', 'Kd_a')

robotParameters;

youBot_PARAM;





function [F, Z_L, Z_R] = torso_pitch(q, Traj)

theta0 = q;


Z_L = max(-12.5*sin(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3)) , +4.5*sin(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3))) + ...
    38*( cos(Traj(1,1)+theta0) + cos(Traj(1,1)+theta0-Traj(1,2)) ) + 3*cos(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3));

Z_R = max(12.5*sin(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6)) , -4.5*sin(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6)))+...
    38*( cos(-Traj(1,4)-theta0) + cos(-Traj(1,4)-theta0+Traj(1,5)) ) + 3*cos(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6));

F(1) = Z_L-Z_R;

end


% function F = torso_pitch(q, Traj)
% 
% theta0 = q;
% 
% 
% Z_L = max(12.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3)) , -4.5*sin(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3)))+...
%     38*( cos(-Traj(1,1)-theta0) + cos(-Traj(1,1)-theta0+Traj(1,2)) ) + 3*cos(-Traj(1,1)-theta0+Traj(1,2)-Traj(1,3));
% 
% Z_R = max(12.5*sin(Traj(1,4)+theta0 +Traj(1,5)-Traj(1,6)) , -4.5*sin(Traj(1,4)+theta0 +Traj(1,5)-Traj(1,6)))+...
%     38*( cos(Traj(1,4)+theta0) + cos(Traj(1,4)+theta0+Traj(1,5)) ) + 3*cos(Traj(1,4)+theta0+Traj(1,5)-Traj(1,6));
% 
% F(1) = Z_L-Z_R;
% 
% end