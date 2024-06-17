## Requirements:

* Ubuntu 20.04
* ROS Noetic (http://wiki.ros.org/noetic/Installation/Ubuntu)
* Webots 2022a (https://github.com/cyberbotics/webots/releases/download/R2022a/webots_2022a_amd64.deb)

## Installation

Note: It is assumed that Ubuntu OS, ROS platform and Webots simulator are already installed. 

* $ cd
* $ git clone https://github.com/hector-aviles/JIFS2024
* $ cd JIFS2024/catkin_ws
* $ catkin_make
* $ sudo pip3 install beepy
* $ sudo apt install ros-noetic-webots-ros
* $ sudo apt install ros-noetic-ros-numpy
* $ echo "source ~/JIFS2024/catkin_ws/devel/setup.bash" >> ~/.bashrc
* $ source ~/.bashrc

## Two Way experiment:

* $ roslaunch get_samples jifs.launch world:=two_ways
* $ rosrun utils logger.py

* En la GUI presionar botón de Start antes de 50 segundos de iniciado el mundo
* Con los botones de la GUI se activan los comportamientos

## Videos
* A vehicle approaching in the opposite direction

https://github.com/hector-aviles/JIFS2024/assets/67079858/3a7ea50d-20ee-4a78-81ad-f38a0a62c281

* A vehicle travelling in the transverse direction

https://github.com/hector-aviles/JIFS2024/assets/67079858/1e4b4664-0559-4ee1-98ab-f971068c3fa2



## Contact

Héctor Avilés<br>
havilesa@upv.edu.mx <br>
Marco Negrete<br>
marco.negrete@ingenieria.unam.edu

