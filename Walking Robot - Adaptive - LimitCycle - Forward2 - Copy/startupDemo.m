%% Clear everything

close all
% Copyright 2016-2018 The MathWorks(TM), Inc.

YBT_HomeDir = pwd;

% Code to use copy within this repository
addpath([YBT_HomeDir filesep 'Libraries' filesep 'CFL_Libs']);
cd([YBT_HomeDir filesep 'Libraries' filesep 'CFL_Libs']);
startup_Contact_Forces

cd(YBT_HomeDir)
addpath(YBT_HomeDir);
% addpath([YBT_HomeDir filesep 'CAD']);
% addpath([YBT_HomeDir filesep 'Scripts_Data']);
addpath([YBT_HomeDir filesep 'Libraries']);
% addpath([YBT_HomeDir filesep 'Libraries' filesep 'Pace']);
addpath([YBT_HomeDir filesep 'Images']);
% addpath([YBT_HomeDir filesep 'Optim']);
% addpath([YBT_HomeDir filesep 'Resources']);
% addpath([YBT_HomeDir filesep 'URDF']);

%% Add folders to the path
% addpath('Optim','SavedResults',genpath('Robot'),genpath('Libraries'))

%% Open the main model and load parameters
% youBot_PARAM
% robotParameters
walkingRobot_Forward2
