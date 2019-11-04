# Simscape_DMP_Biped
The files you need to run for simulating a forward optimization are as follows:
1. First, run the 'startupDemo.m' to load the biped Simscape model 'walkingRobot_Forward2'. 'startupDemo.m' automatically calls all the libraries needed for the model to run. These libraries are placed in 'Libraries' as well as 'walkingRobotUtils'.

2. 'optimizeRobotMotion_openLoop.m' is the main file where the GA optimization is done for given number of generations and value for population size. This file passes the cost and the constraint functions to the GA optimization. 

3. The cost function is defined in 'CostFCN.m'.

4. The constraints are defined in 'simWalker_constraint.m'.

5. 'outputFCN.m' is used to pass all individuals in the generations.
