EE599 - Spring 2023 - Instructor: Prof. Somil Bansal
Student Code Template - HW 2 - Problem 3
Written by: Kaustav Chakraborty (kaustavc@usc.edu) 
*******************************************************************************************

TASK 1: The students are required to compute the BRT, optimal controller and the optimal disturbance
for a vehicle model. Specifially, you are required to fill in the files:

1. @DubinsCar/optCtrl.m
   This will require you to compute the optimal controller that maximizes the value function
2. @DubinsCar/optDstb.m
   This will require you to compute the optimal disturbance that minimizes the value function
3. BRT_computation.m 
   This will require you to define the grid and the obstacle function for the which you want to compute
   the unsafe set

Note: 
1. Please add the helperOC and ToolboxLS libraries to your matlab path for the BRT computation to happen
2. If implemented successfully, you can visualize the growing BRT by running the following line:
     
    BRT_computation(get_params())


TASK 2: The students are required to compute the filters (least restrictive, QP-based), that 
will safely take a vehicle from a start state to a goal state while avoiding obstacles in the environment.
Specifially, you are required to fill in the files:
    1. get_qpfilter_controller.m
    This will require you to complete the function 
    "get_qpfilter_controller(current_state, u_nom, params)"
    which takes in the nominal controller "u_nom", the "curent_state" of the robot
    and the packed parameter struct "param", to return the QP-based safety filtered 
    controller at that state.

    2. get_safety_controller.m
    This will require you to complete the function 
    "get_safety_controller(current_state, u_nom, params)"
    which also takes in the nominal controller "u_nom", the "curent_state" of the robot
    and the packed parameter struct "param", to return the least restrictive safety 
    controller at that state. 


The "main.m" file is the main driver for the code.

The organization is as follows:
    - parameter declaration
    - BRT computation
    - robot trajectory initialization 
    - while the robot does not reach the goal do the following,
        - get the nominal controller 
        - filter the controller according to the controller choice
        - simulate the next state with noise and add it to the trajectory
        - plot the environment with the trajectory uptil current time
    - plot the controller profiles

Parameters: The parameter for the code can be set in the "get_params.m" file. This file 
    is called at the beginning of the main file and returns the struct called params. Please
    see Note 1 (below) for usage.

Testing: You are provided with test cases to test your work. Please choose the tests numbered
    0 to 5, and verify if you are able to safely navigate your vehicle to the goal. Each of the 
    tests applies a different magnitude of disturbance to the vehicle trajectory. 
    The disturbance magnitudes may be constant or fixed. To choose a test, simply 
	pass the test number to the "params.test_choice" field in the "get_params.m" file.
	Example: To choose a test 3, set the following in "get_params.m",
			params.test_choice = 3;

	Note: test 4, and test 5 requires you to have the toolboxLS and helperOC libraries in your matlab 
	path. These are provided to you.

Chosing controllers: 
     The nominal planner is already provied to you. To generate the nominal trajectory,
     set "params.controller_choice = 0;" in the "get_params.m" file then execute
     the "main.m" file. This will display the environment with the evolving trajectory
     under the nominal controller. The nominal controller profile will then be plotted at the end.
	
     To chose the least restrictive safety filtering, set "params.controller_choice = 1;" in 
     the "get_params.m" file then execute the "main.m" file. Make sure that you have imple-
     mented the "get_safety_controller.m" file before doing so.

     To chose the qp based safety filtering, set "params.controller_choice = 2;" in 
     the "get_params.m" file then execute the "main.m" file. Make sure that you have imple-
     mented the "get_qpfilter_controller.m" file before doing so.

This allow you to check your controllers with the 6 test cases to verify the performance of your
method.

Additional files (no need for modification):

    * dynConst.m - defines the nonlinear contraints for the dynamics
    * get_nominal_controller.m - returns the nominal controller at a given state 
    * mpc.m - returns the MPC solution for the optimal control problem of reaching the goal
    * plot_env.m - plots the environment - obstacles, goal, states, trajectory, etc.
    * simulate.m - simulates the next state given the current state, controls, disturbance, 
                    noise, step time
    * simulate_trajectory.m - simulates the MPC trajectory by taking the optimization solution 
                   and initial state
    * stopping_criteria.m - takes the current state and goal conditions and returns true 
                    if the state has reached the goal else returns false
    * plot_controller.m - plots the choice of the controller over the nominal controller,
                    over the entire trajectory.
    * get_optDst_precom.m - returns the values from the precomputed optimal disturbance files.
    * test.m - returns the noise drift for testing performance of controller.

NOTE: 
    1. You are allowed to add additional parameters to the param struct in the 
    "get_params.m" file. As an example, if you want to add the parameter "time", 
    you can do so by "param.time = 0".

    2. We recommend keeping the function signatures same, as it will help in uniform 
    evaluation of your work.

    3. In order to visualize the environment in general you can run 
    
        "plot_env([], get_params())"

    4. Saving plots is not enabled by default, so you have to save them manually from the
    popup windows or write appropriate code.

    5. You may add additional files to your solution, eg. compuation of a look up table with precomputed 
    safety controllers, learned neural networks, etc. But your code must be reproducible under any 
    general setup.
