% Main script for optimizing the gait of a walking robot model

%% Run the model start-up file and libraries
startupDemo

%% Set initial parameters

% Choose model name
mdlName = 'walkingRobot_Forward2'; % Main model

% Flags to speed up simulation
accelFlag = false;
parallelFlag = true;

% Joint actuator type for optimization
% 1 = motion | 2 = torque | 3 = motor
actuatorType = 1;


%% Bounds and number of variables

Amp_no = 3;     % 3 amplitudes each for each of the DOFs (symmetric gait)

lowerBnd = [0     0.3    0.6 0.6 0.60];
upperBnd = [1     1.50   1.40 1.40 1.30];

numPoints = size(lowerBnd,2);
% IntCon = [1];



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

opts.PlotFcn = {@gaplotbestf, @gaplotmaxconstr @outputFCN}; % Add progress plot of fitness function
opts.UseParallel = parallelFlag;
opts.UseVectorized = false;


%% optimization initial guess individuals
% opts.InitialPopulationMatrix = p0 + .5*(upperBnd-p0).*rand(opts.PopulationSize,numPoints) + .2*(p0-lowerBnd).*rand(opts.PopulationSize,numPoints);
r = randi([2 3],1);
r1 = randi([2 3],1);
p0 = [.4 .45 1 1 1];
p01= [.7  .7 .8 .8 .7];
opts.InitialPopulationMatrix = [lowerBnd+(upperBnd-lowerBnd).*rand(opts.PopulationSize-r1-r,numPoints); repmat(p0,[r1 1]); repmat(p01,[r 1])];% Add copies of initial gait

% output matrix in which the optimal individuals of each generations is
% stored and shown during the optimization
output = zeros(opts.MaxGenerations,numPoints);



%% one can run the optimization simulation for different treadmill speeds
for k = 4

V_treadmill = 0.3 + (1.7/5)*(k-1);  % V_treadmill ~ 1 m/s

%% Run commands to set up parallel/accelerated simulation
doSpeedupTasks;

%% Optimization cost and constraint functions
costFcn = @(p)CostFCN(p,mdlName,V_treadmill);
nonlcon = @(p)simWalker_constraint(p,mdlName,V_treadmill); % Constraint for height

tic



%% solve the problem by ga
disp(['Running optimization! Population Size: ' num2str(opts.PopulationSize) ...
      ', Fitness Limit: ' num2str(opts.FitnessLimit) ...
      ', Generations No: ' num2str(opts.MaxGenerations)])
   
  
[pFinal,penalty,exitflag] = ga(costFcn,numPoints,[],[],[],[], ... 
                     lowerBnd,upperBnd, ...
                     nonlcon,opts);
                        

%% solve by fmincon
% % % [pFinal,penalty,exitflag] = fmincon(costFcn,p0,[],[],[],[], ...
% % %                         lowerBnd,upperBnd, ...
% % %                         nonlcon,opts);                 
                 
disp(['Final penalty function value: ' num2str(penalty)])


%% Return the optimal decision variables
    pFinal(1,1) = 35+7*pFinal(1,1);
    ts0 = ceil(pFinal(1,1));    
    tau = pFinal(1,2);
    Am = pFinal(1,3:end);
    Am = repmat(Am,1,6/length(Am));
    tau_nom=1.13;
    tSim = (.57/tau_nom)*tau;

%% 
% Obtain DMPs and other simulation variables dependent on the optimization
% decision variables - This part can be used for running the simulation
% with the final optimal variables and parameters
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


%% Evaluate the individuals cost penalties and constraint violations in each generation
% It does the same thing as the two last parts above but for all the 
% generations and retunrs the cost penalties and constraints violations
for j = 1:size(GenBest,1)
    GenBest_vel(:,:,k) = GenBest'
    j
    [penalty_bestGen(j,k), Work_bestGen(j,k), pitch_err, Constraint_viol, Constraint_viol_indiv] = CostFCN(GenBest(j,:),mdlName,V_treadmill);
    Constraint_viol_bestGen(j,k)=Constraint_viol;
    pitch_err_bestGen(j,k)=pitch_err;
    Constraint_viol_indiv_bestGen(j,:,k)=Constraint_viol_indiv;
%     init_ht_sym(j,1) = z_lh(1,1)-z_rt(1,1);
%     final_stop(j,1)  = z_rh(1,end)-5.01;
%     clear z_lh z_rt z_lt z_rh
end



el_time = toc/3600;
el_time = [num2str(floor(el_time)), ' hrs & ', num2str(floor(60*(el_time-floor(el_time)))), ' mins']

end



%% The function used to find the initial torso pitch and height 
% torso height is a function of torso pitch and is defined by the
% assumption that in the beginning, the right leg toe-off is taking place
function [F, Z_L, Z_R] = torso_pitch(q, Traj)

theta0 = q;


Z_L = max(-12.5*sin(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3)) , +4.5*sin(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3))) + ...
    38*( cos(Traj(1,1)+theta0) + cos(Traj(1,1)+theta0-Traj(1,2)) ) + 3*cos(Traj(1,1)+theta0-Traj(1,2)+Traj(1,3));

Z_R = max(12.5*sin(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6)) , -4.5*sin(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6)))+...
    38*( cos(-Traj(1,4)-theta0) + cos(-Traj(1,4)-theta0+Traj(1,5)) ) + 3*cos(-Traj(1,4)-theta0 +Traj(1,5)-Traj(1,6));

F(1) = Z_L-Z_R;

end
