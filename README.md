## An Experimentally Validated Hybrid Control Strategy for Robust Contact Detection and Force Regulation

This repository contains simulation and experimental code for the paper "An Experimentally Validated Hybrid Control Strategy for Robust Contact Detection and Force Regulation" by India Spott, Jan De Priester, and Ricardo G. Sanfelice. 

To see the code in action, please look at the [#Experiments](#experiments) section below, which contains videos of the experiments conducted in the paper.

## Simulation Code
The simulation code can be found in the `simulation` folder. It includes the implementation of the hybrid control strategy as well as the baseline controllers for comparison written in MATLAB. 

To run the simulations, simply navigate to the `simulation` folder and execute the `main.m` file. 

`Figure 1` will produce the simulation results for `Figure 2` in the paper, while `Figure 2` will produce the simulation results for `Figure 3` in the paper. 

## Experimental Code
The experimental code can be found in the `experimental_src` folder. It includes the implementation of the hybrid control strategy as well as the ROS SDKs for the manipulator and end-effector in C++ and Python. The experiments are run using an Ubuntu 22.04 system with ROS Humble Hawksbill on an Elfin e03 robotic manipulator and an OnRobot RG2-FT end-effector.

The code is designed for and tested on the specific hardware and software setup used in the experiments, so it may not be directly applicable to other setups without modification. The experimental code is modified slightly between each experiment. The nominal force regulation experiment uses just the hybrid controller, the changing center of gravity experiment uses the hybrid controller and a manual rotation of the end-effector, the changing stiffness uses the hybrid controller and a manual change of the stiffness of the environment, and the long term stability experiment uses just the hybrid controller with an extended time period. Each experiment follows the descriptions in the paper, so please refer to the paper for more details on the experimental setup and procedure.

## Experiments
*Note: Videos files are too large to view in GitHub's website and must be downloaded.*

### Changing Center of Gravity (CoG)
![Rotating CoG Experiment 1](videos/rotate.MP4)

![Changing CoG Experiment 2](videos/rotate_2.MP4)

### Changing Stiffness
![Changing Stiffness Experiment 1](videos/squeeze.MP4)

![Changing Stiffness Experiment 2](videos/squeeze_2.MP4)
