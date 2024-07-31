# MPC-Based Control and Obstacle Avoidance Project

## 📌 Introduction

This is an MPC (Model Predictive Control)-based control and obstacle avoidance project that utilizes the Eigen and NLopt libraries for matrix calculations and nonlinear optimization. The project includes source files, which demonstrates the application of the MPC algorithm in robot control and obstacle avoidance.

## 🖼️ Kinematic Modeling Diagram

<img height="300" src="figure/kinematic_modeling_diagram.jpg" width="400"/>

## 🚀 Installing Dependencies and Running the Project

Before running this project, make sure to install the Eigen and NLopt libraries. Below are the installation steps and instructions on how to compile and run the project.

### Installing Eigen and NLopt Libraries and Running the Project

For Ubuntu or Debian systems:

```
sudo apt update
sudo apt install libeigen3-dev libnlopt-dev
g++ -I /usr/include/eigen3 -I /usr/include -o main main.cpp -lnlopt
./main
```

## 🎉 Results Display
Below is an image showcasing the results of running the code.

### Simulation Image

<img height="469" src="figure/trajectory_plot.png" width="781.5"/>

### Actual Landing Image

The actual landed historical data is stored in the root directory in a file named `historicalDataActualLanding.txt`.

<img height="240" src="figure/controlTruth.jpg" width="400"/>  <img height="240" src="figure/controlTruth0.jpg" width="400"/>