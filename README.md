# turtlebot_with_burger_catkin

### Pre requesites
- ROS melodic
- Ubuntu 18

Código criado no âmbito da dissertação "Sensor Fusion and  Mapping of a wheeled Mobile Robot using Vision and LiDAR", de Carolina  Marques

Sensor fusion SLAM algorithm between an LDS-01 and a Intel Realsense D435 RGB-D sensor, with support for the Turtlebot3 simulation environments.

How to run the algorithm

1. Clone the repository on https://github.com/carolina-819/ORB-SLAM2-sensor-fusion and follow the instructions on that README.md file

2. Create a new catkin workspace http://wiki.ros.org/catkin/Tutorials/create_a_workspace

3. Delete the existing src folder created and clone this repository into the root of your catkin workspace

4. Build the workspace

5. On the root of your catkin workspace, run

   `chmod +x run_sim_gaz`

6. run catkin_make to build the catkin workspace



### To run the whole algorithm

###### Open three terminal windows at the root of your catkin workspace

- ./run_sim_gaz on the root catkin workspace makes the turtlebot3 simulation run. You can run other environments by uncomenting line 4 and commenting line 5
- roslaunch slam_test teste.launch -> this makes the noise node run in order to add gaussian noise to the outputs produced by the simulation
- export TURTLEBOT3_MODEL=burger && roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch -> this is to be able to control the turtlebot

###### Open a new terminal window at the root of your ORB_SLAM2 folder and run the following command

- rosrun --prefix 'gdb -ex run --args' ORB_SLAM2 RGBD ~/ORB_SLAM2/Vocabulary/ORBvoc.txt ~/ORB_SLAM2/Examples/RGB-D/d435i.yaml -> this launches the ORB SLAM sensor fusion algorithm. You need to have the previous two nodes (turtlebot simulation and noise node) running in order for this to launch
