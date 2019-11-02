# robot_wall_follower
a ROS robot that follows walls

:warning: **develoment environment: Ubuntu 18.04.3 LTS**

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
|   |   |   |   |   |- world01.world
|   |   |   |   |   |- world02.world
|   |   |   |   |   |- world03.world
|   |   |   |   |   |- i_world.world
|   |   |   |   |   |- l_world.world
|   |   |   |   |   |- t_world.world
|   |   |   |   |   |- x_world.world
|   |   |   |   |   |- w_world.world
|   |   |   |   |   |- z_world.world
```

## How to run

### Run RViz
```bash
cd simulation_ws
source devel/setup.bash
export LC_NUMERIC="en_US.UTF-8"
roslaunch m2wr_description rviz.launch
```

### Run Gazebo
```bash
cd simulation_ws
source devel/setup.bash
roslaunch my_worlds launch_world.launch
```

### Place your robot in Gazebo world
```bash
cd simulation_ws
source devel/setup.bash
roslaunch m2wr_description spawn.launch y:=8 
```

### Run Motion Planner
```bash

```


