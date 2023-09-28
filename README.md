# Suplementary material for contributed paper 875 submitted to ICRA 2024
## A Policy Hierarchy via Probabilistic Logic Factored Markov Decision Processes for Behavior Selection in Self-driving Cars

This repository contains the required software to reproduce the results presented in the contributed paper "A Policy Hierarchy via Probabilistic Logic Factored Markov Decision Processes for Behavior Selection in Self-driving Cars". Such results can be summarized as follows:

* Experiment 1a: Test in an environment with 5 static cars
* Experiment 1b: Test in an environment with 10 static cars
* Experiment 2: Test in an environment with 5 moving cars

Methodology details are described in the contributed paper. 

## Requirements:

* Ubuntu 20.04
* ROS Noetic (http://wiki.ros.org/noetic/Installation/Ubuntu)
* Webots 2022a (https://github.com/cyberbotics/webots/releases/download/R2022a/webots_2022a_amd64.deb)

## Installation

Note: It is assumed that Ubuntu OS, ROS platform and Webots simulator are already installed. 

* $ cd
* $ git clone https://github.com/hector-aviles/ICRA2024
* $ cd ICRA2024/catkin_ws
* $ catkin_make
* $ echo "source ~/ICRA2024/catkin_ws/devel/setup.bash" >> ~/.bashrc
* $ source ~/.bashrc

## Testing

To test each experiment:

* $ roslaunch icra2024 experiment1_5_cars.launch
* $ roslaunch icra2024 experiment1_10_cars.launch
* $ roslaunch icra2024 experiment2.launch

In each experiment you should see a simulator like this:


## Contact

Héctor Avilés<br>
havilesa@upv.edu.mx <br>
Marco Negrete<br>
marco.negrete@ingenieria.unam.edu

