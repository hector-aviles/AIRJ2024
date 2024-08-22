## Requirements:

* Ubuntu 20.04
* ROS Noetic (http://wiki.ros.org/noetic/Installation/Ubuntu)
* Webots 2022a (https://github.com/cyberbotics/webots/releases/download/R2022a/webots_2022a_amd64.deb)

## Installation

Note: It is assumed that Ubuntu OS, ROS platform and Webots simulator are already installed. 

* $ cd
* $ git clone https://github.com/hector-aviles/JIFS2024
* $ cd AIR2024/catkin_ws
* $ catkin_make
* $ sudo pip3 install beepy
* $ sudo apt install ros-noetic-webots-ros
* $ sudo apt install ros-noetic-ros-numpy
* $ echo "source ~/AIR2024/catkin_ws/devel/setup.bash" >> ~/.bashrc
* $ source ~/.bashrc

## Two Way experiment:

* $ roslaunch get_samples jifs.launch world:=two_ways
* $ rosrun utils logger.py

* En la GUI presionar botón de Start antes de 50 segundos de iniciado el mundo
* Con los botones de la GUI se activan los comportamientos

## Videos
* A vehicle approaching in the opposite direction
  
https://github.com/hector-aviles/JIFS2024/assets/67079858/0860044b-ea15-4925-a62f-529eb55a9948

* A vehicle travelling in the transverse direction

https://github.com/hector-aviles/JIFS2024/assets/67079858/03caaf6b-ab8c-444d-a79c-f0df0bf894c9

## Contact

Héctor Avilés<br>
havilesa@upv.edu.mx <br>
Marco Negrete<br>
marco.negrete@ingenieria.unam.edu

