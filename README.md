# turtlebot_with_burger_catkin

### Pre requesites
- ROS melodic
- Ubuntu 18

Código criado no âmbito da dissertação "Sensor Fusion and  Mapping of a wheeled Mobile Robot using Vision and LiDAR", de Carolina  Marques

Sensor fusion SLAM algorithm between an LDS-01 and a Intel Realsense D435 RGB-D sensor, with support for the Turtlebot3 simulation environments.

How to run the algorithm

1. Clone the repository on https://github.com/carolina-819/ORB-SLAM2-sensor-fusion and follow the instructions on that README.md file

2. Create a new catkin workspace http://wiki.ros.org/catkin/Tutorials/create_a_workspace

3. Delete the existing src folder created and clone this repository into the root of your catkin workspace, take the folders from inside the cloned repository into the root folder. Your catkin workspace should look like this

![image](https://user-images.githubusercontent.com/61470279/197336448-29bceffb-c048-410c-9276-61f4c4d902bb.png)

4. Build the workspace by running catkin_make

5. On the root of your catkin workspace, run 

   `chmod +x run_sim_gaz`



### To run the whole algorithm

###### Open three terminal windows at the root of your catkin workspace (it is recommended the usage of terminator)

- ./run_sim_gaz on the root catkin workspace makes the turtlebot3 simulation run. You can run other environments by uncomenting line 4 and commenting line 5

- roslaunch slam_test teste.launch -> this makes the noise node run in order to add gaussian noise to the outputs produced by the simulation (the terminal should output a sequence of numbers, that's the time in miliseconds it is taking the algorithm to add noise to each measurement from the sensors)

- export TURTLEBOT3_MODEL=burger && roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch -> this is to be able to control the turtlebot

###### Open a new terminal window at the root of your ORB_SLAM2 folder and run the following command

- rosrun --prefix 'gdb -ex run --args' ORB_SLAM2 RGBD ~/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/ORB_SLAM2/Examples/RGB-D/d435i.yaml -> this launches the ORB SLAM sensor fusion algorithm. You need to have the previous two nodes (turtlebot simulation and noise node) running in order for this to launch. The ORB vocabulary can take some time to load up, please be patient.


#### If you ran all of the commands correctly, you should now be seeing something similarly to this on your screen (you might need to adjust the windows)
![image](https://user-images.githubusercontent.com/61470279/197337171-bf44ca5b-8ad3-4262-b25d-6c85aa4c972d.png)

