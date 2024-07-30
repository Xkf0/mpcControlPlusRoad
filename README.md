# MPC-Based Control and Obstacle Avoidance Project

## Introduction

This is an MPC (Model Predictive Control)-based control and obstacle avoidance project that utilizes the Eigen and NLopt libraries for matrix calculations and nonlinear optimization. The project includes a source file named `main.cpp`, which demonstrates the application of the MPC algorithm in robot path planning and obstacle avoidance.

## Installing Dependencies and Running the Project

Before running this project, make sure to install the Eigen and NLopt libraries. Below are the installation steps and instructions on how to compile and run the project.

### Installing Eigen and NLopt Libraries and Running the Project

For Ubuntu or Debian systems:

```
sudo apt update
sudo apt install libeigen3-dev libnlopt-dev
g++ -I /usr/include/eigen3 -I /usr/include -o main main.cpp -lnlopt
./main
```

## Results Display
Below is an image showcasing the results of running the code:

<img height="469" src="figure/trajectory_plot.png" width="781.5"/>