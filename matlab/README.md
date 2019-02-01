PID 101 -- simulator

### About

This a simulation toolbox 

### Contents


- car_kinematics: defines the kinematics of the car using a bicycle model
- car_move: solves the car kinematics between two sampling times
- car_runner: main runner (script); performs simulations & produces plots
- car_simulate: performs simulations using a given PID controller
- make_car: constructs a structure with all necessary vehicle parameters
- make_pid_controller: constructs a PID controller out of given parameters
- plot_controlled_car: plots closed-loop trajectories