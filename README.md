# robot_wall_follower
a ROS robot that follows walls

:warning: **develoment environment: Ubuntu 18.04.3 LTS**

## Run RViz
```bash
cd simulation_ws
source devel/setup.bash
export LC_NUMERIC="en_US.UTF-8"
roslaunch m2wr_description rviz.launch
```

## Run Gazebo
```bash
cd simulation_ws
source devel/setup.bash
roslaunch my_worlds launch_world.launch
```

## Place your robot in Gazebo world
```bash
cd simulation_ws
source devel/setup.bash
roslaunch m2wr_description spawn.launch y:=8 
```

## Run Motion Planner
```bash

```


