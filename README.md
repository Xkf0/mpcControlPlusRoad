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

### Vehicle Control Simulation Image

The following are control simulation images for vehicle speeds:

- **1 m/s**:

<img height="469" src="figure/trajectory_plot_speed1.png" width="781.5"/>

- **2 m/s**:

<img height="469" src="figure/trajectory_plot_speed2.png" width="781.5"/>

- **3 m/s**:

<img height="469" src="figure/trajectory_plot_speed3.png" width="781.5"/>

The following are error over time images

|              Yaw Angle Error                                 |               Distance Error                                 |
|:------------------------------------------------------------:|:------------------------------------------------------------:|
| <img height="230" src="figure/theta_error.jpg" width="390"/> | <img height="230" src="figure/theta_error.jpg" width="390"/> |


### Vehicle Control and Obstacle Avoidance Simulation Image

The following are control and obstacle avoidance simulation image:

<img height="469" src="figure/trajectory_plot.png" width="781.5"/>

### Actual Landing Image

The actual landed historical data is stored in the root directory in a file named `historicalDataActualLanding.txt`.

<img height="240" src="figure/controlTruth.jpg" width="400"/>  <img height="240" src="figure/controlTruth0.jpg" width="400"/>

## 📢 Acknowledgments

I would like to express my sincere gratitude to Xinke Liu for her invaluable support and contributions to this project.

## 🛡️ Cite

If you find this project useful in your research, please consider citing:

```bibtex
@misc{mpcControlPlusRoad,
  title={MPC-Based Control and Obstacle Avoidance},
  author={Fan Xie, Xinke Liu},
  howpublished = {\url{https://github.com/Xkf0/mpcControlPlusRoad}},
  year={2024}
}
```