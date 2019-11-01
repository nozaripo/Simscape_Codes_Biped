% Main script for optimizing the gait of a walking robot model
% Copyright 2017 The MathWorks, Inc.

%% Set initial parameters
clear all
% Choose model name
mdlName = 'walkingRobot_Amp'; % Main model
% mdlName = 'walkingRobot_Peasgood_Tau_SplitBelt'; % Main model
% mdlName = 'walkingRobot_Peasgood_Amp_SplitBelt'; % Main model
% Last Run % mdlName = 'walkingRobot_Peasgood_Amp_SplitBelt_MultiAmp'; % Main model

% mdlName = 'RobotWalkerModified'; % CAD imported model

[DMP, W, Am, Ym, tau, dt, time, Traj, F, init_pos, init_vel] = learn_rcp_batch;

% Flags to speed up simulation
accelFlag = true;
parallelFlag = true;

% Joint actuator type for optimization
% 1 = motion | 2 = torque | 3 = motor
actuatorType = 1;

% To reduce the search space, scale the angle waypoints and solve the
% optimization algorithm with integer parameters
scalingFactor = 1;        % Scaling from degrees to integer parameters

% Uncomment to create initial conditions from scratch
numPoints = 9;              % Number of joint angle points
% numPoints = 21;
% gaitPeriod = 0.8;           % Period of walking gait [s]
%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% [K_t, Ki_t, Kd_t      K_v, Ki_v, Kd_v]
% t: relate desired pitch to desired time period
% v: relate velocity/relative position to desired pitch

% % % % LimitCycle_Amp_split_tri-amp
%%
upperBnd = [1.5,20,.5     1.5,20,.5   1.5,20,.5];
lowerBnd = [.001, 0, 0    .001, 0, 0   .001, 0, 0];
%

% % % % Peasgood_Amp_split_multi-amp
%%
% % % % % % upperBnd = [1,5,.2      .015,.1,.005     .015,.1,.005   .015,.1,.005    .015,.1,.005    .015,.1,.005    .015,.1,.005];
% % % % % % lowerBnd = [0,0,0      .001, 0, 0    .001, 0, 0   .001, 0, 0    .001, 0, 0  .001, 0, 0  .001, 0, 0]; % Limit for tau
% % % % % %%
%%
% upperBnd = [.4, .1, 1];

% [K_t, Ki_t, Kd_t      K_v, Ki_v, Kd_v]
% t: relate desired pitch to desired time period
% v: relate velocity to desired pitch\

%%%%%%% % % % % Peasgood_Amp
% upperBnd = [.01,.05,.001    1,5,.2];     %Amplitude update law
% lowerBnd = [0, 0, 0     0,0,0]; % Limit for tau



% upperBnd = [.5, 2];

% [3, ... % Limit for tau
%             2*ones(1,numPoints-1)] ... % Limit for amplitude r
%             /scalingFactor;
% lowerBnd = zeros(1,numPoints);

%             .3*ones(1,numPoints-1)] ... % Limit for amplitude r
%             /scalingFactor;
%%
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p0 = lowerBnd+(upperBnd-lowerBnd).*rand(1,numPoints);  % Create zero motion initial conditions
% p0 = [.25 .047 .72];  % Create zero motion initial conditions
% p0 = [.25 .1];  % Create zero motion initial conditions
% p0 = .25;  % Create zero motion initial conditions

% Uncomment to load initial gait from previous optimization results
% startFile = 'optimizedData_17Jul17_1021';
% [p0,gaitPeriod,numPoints] = createInitialConditions(startFile,scalingFactor);

%% Set optimization options
opts = optimoptions('ga');
opts.Display = 'iter';
% opts.FitnessLimit = -9e+4;
opts.FitnessLimit = 1;
opts.MaxGenerations = 45;
opts.PopulationSize = 400;
opts.InitialPopulationMatrix = repmat(p0,[opts.PopulationSize 1]); % Add copies of initial gait
opts.PlotFcn = @gaplotbestf; % Add progress plot of fitness function
opts.UseParallel = parallelFlag;
opts.UseVectorized = false;

%% Set bounds and constraints
% Upper and lower angle bounds
% upperBnd = [3, ... % Limit for tau
%             5*ones(1,numPoints-1)] ... % Limit for amplitude r
%             /scalingFactor;
% lowerBnd = [.5, ... % Limit for tau
%             .1*ones(1,numPoints-1)] ... % Limit for amplitude r
%             /scalingFactor;
        

%% Run commands to set up parallel/accelerated simulation
doSpeedupTasks;

%% Run optimization
costFcn = @(p)simulateWalkingRobot(p,mdlName,scalingFactor);
% nonlcon = @(p)GAconstraint(p,mdlName,scalingFactor,gaitPeriod,actuatorType); % Constraint for height
tic
disp(['Running optimization! Population Size: ' num2str(opts.PopulationSize) ...
      ', Fitness Limit: ' num2str(opts.FitnessLimit) ...
      ', Generations No: ' num2str(opts.MaxGenerations)])
   
% ', max Generations No.: ' num2str(opts.maxGenerations) ...  
  
[pFinal,reward] = ga(costFcn,numPoints,[],[],[],[], ... 
                     lowerBnd,upperBnd, ...
                     [],[],opts);
disp(['Final reward function value: ' num2str(-reward)])

K_h   = pFinal(1);
Ki_h  = pFinal(2);% Ktau = pFinal(2);
Kd_h  = pFinal(3); % Kd = pFinal(2);
K_k   = pFinal(4);
Ki_k  = pFinal(5);% Ktau = pFinal(2);
Kd_k  = pFinal(6); % Kd = pFinal(2);
K_a   = pFinal(7);
Ki_a  = pFinal(8);% Ktau = pFinal(2);
Kd_a  = pFinal(9); % Kd = pFinal(2);
%%
% K_v   = pFinal(1);
% Ki_v  = pFinal(2);% Ktau = pFinal(2);
% Kd_v  = pFinal(3); % Kd = pFinal(2);
% K_t   = pFinal(4);
% Ki_t  = pFinal(5);% Ktau = pFinal(2);
% Kd_t  = pFinal(6); % Kd = pFinal(2);
% K_t1  = pFinal(7);
% Ki_t1 = pFinal(8);% Ktau = pFinal(2);
% Kd_t1 = pFinal(9); % Kd = pFinal(2);
% K_t2  = pFinal(10);
% Ki_t2 = pFinal(11);% Ktau = pFinal(2);
% Kd_t2 = pFinal(12); % Kd = pFinal(2);
% K_t3  = pFinal(13);
% Ki_t3 = pFinal(14);% Ktau = pFinal(2);
% Kd_t3 = pFinal(15); % Kd = pFinal(2);
% K_t4  = pFinal(16);
% Ki_t4 = pFinal(17);% Ktau = pFinal(2);
% Kd_t4 = pFinal(18); % Kd = pFinal(2);
% K_t5  = pFinal(19);
% Ki_t5 = pFinal(20);% Ktau = pFinal(2);
% Kd_t5 = pFinal(21); % Kd = pFinal(2);
%%

el_time = toc/3600;
el_time = [num2str(floor(el_time)), ' hrs & ', num2str(floor(60*(el_time-floor(el_time)))), ' mins']
%% Save results to MAT-file
% Convert from optimization integer search space to trajectories in radians
% pScaled = scalingFactor*pFinal;
% time = linspace(0,gaitPeriod,numPoints+1)';
% ankle_motion = deg2rad([pScaled(1:numPoints) pScaled(1)]');
% knee_motion = deg2rad([pScaled(numPoints+1:2*numPoints) pScaled(numPoints+1)]');
% hip_motion = deg2rad([pScaled(2*numPoints+1:3*numPoints) pScaled(2*numPoints+1)]');
% curveData = createSmoothTrajectory(ankle_motion,knee_motion,hip_motion,gaitPeriod);
% outFileName = ['optimizedData_' datestr(now,'ddmmmyy_HHMM')];
% save(outFileName,'curveData','reward','gaitPeriod','time', ... 
%                  'hip_motion','knee_motion','ankle_motion');
% plotSmoothTrajectory;
%% Cleanup
% bdclose(mdlName);

% [Traj, time, ta, init_height] = run_rcp(pFinal(1,1),pFinal(1,2:end));

% Data=[time, Traj];
% save('Dataaaa.mat','Data','ta','init_height')

if parallelFlag
   delete(gcp('nocreate')); 
end

save('pFinal.mat','K_h', 'Ki_h', 'Kd_h','K_k', 'Ki_k', 'Kd_k', 'K_a', 'Ki_a', 'Kd_a')

robotParameters;

youBot_PARAM;