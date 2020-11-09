# HSR Navigation Framework

Navigation framework for the HSR mobile base robot by Toyota.

You can watch the video demonstrating the package here: https://www.youtube.com/watch?v=IkanTA_ikvA&ab_channel=HumanoidsBonn

## Compilation

Before being able to run the package, it has to be compiled. To do so start by cloning the project into your directory. This is going to be a two step compilation, where:

1) First we compile the messages, actions, etc.
2) We compile the C++ executables.

### Message/Actions compilation

To compile the messages we have to disable the executable compilation as this makes use of the former, hence encouring in compilation problems. Therefore, you should **comment out** the four lines in the **CMakeLists.txt** file, and run:

- **catkin build** from the catkin_ws directory

The compilation should have been successful. Now **uncomment** the same four lines you commented before and run again the same command from the catkin_ws directory to compile the executables:

- **catkin build**

The package is now compiled and ready to be used.

## How To Run

To run the pipeline a sequence of commands have to be launched:

1) Make map available: **rosrun map_server map_server <path_to_the_map_yaml_file>**

2) Launch the planner: **rosrun hsr_navigation planner**

3) Launch navigation: **roslaunch hsr_navigation launch**

## How Does It Work

The aim of the package is to allow the Toyota HSR to move from point A to point B while freeing up the path by manipulating obstacles. The package has four main modules:

1) control
2) planner
3) navigation
4) perception

### Control

The control class handles the controls of the robot, this means the **DWA** navigation, **replanning** while the robot is navigating in case a new obstacle is detected, and **manipulation** by calling a general **Python** script that used the off-the-shelf Toyota library to grasp, kick, and push.

### Planner

The planner plans in the given static map (made available through the map_server command) and returns the path to the goal, as well as the obstacles on the path.

### Navigation

The navigation handles all that is high-level to this process, meaning calling the perception to get which obstacles are visible in the RGB camera, as well as the planner with the obstacle's positions as arguments in order to receive the most efficient path and start the navigation process.

**Please note that you have to set the goal you want the robot to reach in the navigation.cpp file, from line 186-194**.

### Perception

The perception class uses the incoming RGB and Depth data from the robot to detect and compute the 3D pose of the robot in the world space (necessary for the planner). A neural network is used to get strict bounding boxes around the obstacles so as to make the 3D pose computation as precise as possible.

The obstacle detection is serviced through a **Python** script. All the scripts used can be found in the **scripts** folder.
