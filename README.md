# Robot Wall Follower
## Introduction
This is a ROS reactive two-wheeled robot which goal is to follow walls.

## Development environment
* Ubuntu 18.04.3 LTS
* ROS melodic
* Python 2.7.15
* Gazebo 9.0.0

## The wall-following algorithm
In a nutshell, the algorithm to follow walls can be described by the tables:
![state machine table](https://github.com/filipaivars/robot_wall_follower/blob/master/images/state_machine.png)
![actions table](https://github.com/filipaivars/robot_wall_follower/blob/master/images/actions.png)

To see detailed information about this, please read the report

## Directory structure
```bash
.
|- catkin_ws
|   |- build                        # build folder
|   |- devel                        # compilation folder
|   |   |- ...
|   |   |- setup.bash               # file to source for catkin_ws
|   |   |- ...
|   |- src
|   |   |- motion_plan
|   |   |   |- scripts
|   |   |   |   |- follow_wall.py   # script to follow the wall
|   |   |   |   |- ...
|- simulation_ws
|   |- build                        # build folder
|   |- devel                        # compilation folder
|   |   |- ...
|   |   |- setup.bash               # file to source for catkin_ws
|   |   |- ...
|   |- src
|   |   |- two_wheeled_robot
|   |   |   |- m2wr_description
|   |   |   |   |- launch
|   |   |   |   |   |- rviz.launch  # script to launch RViz
|   |   |   |   |   |- spawn.launch # script to spawn the robot into Gazebo
|   |   |   |   |- urdf
|   |   |   |   |   |- m2wr.gazebo
|   |   |   |   |   |- m2wr.xacro
|   |   |   |   |   |- macros.xacro
|   |   |   |   |   |- materials.xacro
|   |   |   |- my_worlds
|   |   |   |   |- launch
|   |   |   |   |   |- launch_world.launch
|   |   |   |   |- worlds
|   |   |   |   |   |- i_world.world
|   |   |   |   |   |- l_world.world
|   |   |   |   |   |- t_world.world
|   |   |   |   |   |- w_world.world
|   |   |   |   |   |- x_world.world
|   |   |   |   |   |- x_big_world.world
```

## How to run

### Setup the environment
First things first! We need to first setup the environment by building each one of the workspaces : `catkin_ws` and `simulation_ws` as follows:

```bash
cd catkin_ws
catkin_make
cd ../simulation_ws
catkin_make
```

### Run Gazebo
Gazebo is our simulation tool. Launch a new terminal.

First we need to launch it with a world of our choice.

You can check the available worlds in the directory structure section.

```bash
cd simulation_ws
source devel/setup.bash
```
You can launch a world using this command:

`roslaunch my_worlds launch_world.launch <world_name>`

For example:
```bash
roslaunch my_worlds launch_world.launch w_world
```

### Place your robot in Gazebo world
Now we need to place our robot inside the world we just launched above.

Launch a new terminal and run the following commands.

```bash
cd simulation_ws
source devel/setup.bash
```
The command to spawn the robot as the form:

`roslaunch m2wr_description spawn.launch [<coord_to_spawn>]`

Default run:
```bash
roslaunch m2wr_description spawn.launch
```

For example:
```bash
roslaunch m2wr_description spawn.launch y:=7
```

### Run Motion Planner
Our wall-following algorithm is in the `follow_wall.py` file at the `catkin_ws` workspace and `motion_plan` package.

After you run Gazebo and spawn the robot in it (see above sections), you're ready to run the script:

```bash
cd catkin_ws
source devel/setup.bash
rosrun motion_plan follow_wall.py
```

### Run RViz - optional
RViz will be a good tool to check the laser scan readings. Even though it's not a simulation tool, it is useful to make sure that everything is working as expected.

To add the robot, add the model `RobotModel` and set the Fixed Frame in Global Options to `link_chassis`.

To add the laser scan in order to track it, add the model `LaserScan` and select the topic `m2wr/laser/scan`.

This is not required to run the project.

```bash
cd simulation_ws
source devel/setup.bash
export LC_NUMERIC="en_US.UTF-8"
roslaunch m2wr_description rviz.launch
```
