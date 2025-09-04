# UAV Demonstrator

This project demonstrates the use of both traditional (PID) and intelligent (RL) control methods for the Crazyflie nano-quadcopter.  
We use:
- **MATLAB** for PID-based control and for the user interface (UI)
- **Python** (`gym-pybullet-drones`) for RL-based control
The MATLAB app allows users to visualise and compare PID and RL behaviours on a variety of trajectories.
You don't need Matlab to run the RL simulation. The RL part is based of gym-pybullet-drones and the Waypoints problem is implemented after PyFlyt's Waypoints.

## Requirements
You need to have conda and Matlab installed to run this demonstartor. 
RL simulations could run without Matlab.

## Installation
```sh
git clone https://github.com/norahatem/rl-crazyflie
cd rl-crazyflie

conda create -n drones python=3.10
conda activate drones

pip3 install -e .

```