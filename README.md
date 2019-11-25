# Simscape_DMP_Biped
### Give the simulation a single run
1. First, run the 'startupDemo.m' to load the biped Simscape model 'walkingRobot_Forward2'. 'startupDemo.m' automatically calls all the libraries needed for the model to run. These libraries are placed in 'Libraries' as well as 'walkingRobotUtils'. 

2. Then run the Matlab file 'runSim_testParams.m' to load arbitrary values for decision variables and their dependent parameters (torso pitch and height). Note that you can modify values in this file and see the changes. This step is just to make sure that we are providing the Simscape model with enough and correct decision variables and parameters. 

3. Now you can run the simulation for the variables and parameters loaded.

Note: Once the model file opens up it has the parameters of the file 'robotParameters.m' embedded into its workspace so there's no need to run 'robotParameters.m' before running the simulation. For checking if the parameters are loaded correctly (in case you get error loading model parameters) you can go into the Model Explorer tab (on the right of the 'settings' tab) in the Simulink model 'walkingRobot_Forward2' and go the 'Model Workspace' under the model name on the left list 'Model Hierarchy'. If the parameters are already in the center list you are good. But if not, then on the right you can load the matlab file 'robotParameters.m' and reinitilize its from there.

### Give the GA optimization for bipedal walking simulation a run 
The files you need to run for simulating a forward optimization are as follows:
1. 'optimizeRobotMotion_openLoop.m' is the main file where the GA optimization is done for given number of generations and value for population size. This file passes the cost and the constraint functions to the GA optimization. 

2. The cost function is defined in 'CostFCN.m'.

3. The constraints are defined in 'simWalker_constraint.m'.

4. 'outputFCN.m' is used to pass all individuals in the generations.
