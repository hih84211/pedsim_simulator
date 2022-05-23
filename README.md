# Pedestrian Simulator
<img src=https://github.com/hih84211/pedsim_simulator/blob/master/pedsim_simulator/images/pedsim_gazebo.png width=600/> 
The implementation is based on [Pedsim_ros](https://github.com/srl-freiburg/pedsim_ros), ROS packages that wrap a crowd simulator based on Christian Gloor's [libpedsim](http://pedsim.silmaril.org/) library.
The pedestrian model is based on the social force model of [Helbing et. al](http://arxiv.org/pdf/cond-mat/9805244.pdf).

### Features
- Individual walking using social force model for very large crowds in real time
- Group walking using the extended social force model
- Social activities simulation
- Sensors simulation (point clouds in robot frame for people and walls)
- XML based scene design
- Extensive visualization using Rviz
- (Modified) Option to connect with gazebo for physics reasoning 

### Requirements
- ROS (melodic) with the visualization stack 
- C++11 compiler
- [Turtlebot3 packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/), [turtlebot3_simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation) package

### Installation

The default version is now `melodic`. 

```
cd [workspace]/src
git clone https://github.com/hih84211/pedsim_simulator.git  
cd pedsim_simulator
cd ../..
catkin build -c  # or catkin_make
```

### Sample usage
```
roslaunch pedsim_simulator tb3_pedsim_gazebo.launch
```
## Scenarios

     <scenario>
        <waypoint id="wu" x="0" y="-20" r="2" b="2"/>
        <waypoint id="wd" x="0" y=" 20" r="2" />
        <obstacle x1="10" y1="30" x2="10" y2="-30"/>
        <agent x="0" y="-30" n="12" dx="10" dy="10" type="0" purpose="1">
            <addwaypoint id="wd" />
            <addwaypoint id="wu" />
        </agent>
    </scenario>

### waypoint
| Parameter  | Use | Possible values
| ------------- | ------------- | ------------- |
| id | Identifiant  | string
| x  | x position  | double
| y  | y position  | double
| r (opt, default 0) | radius  | double
| b (opt, default 0) | behaviour  | 0: simple / 1: source / 2: sink

### obstacle
| Parameter  | Use | Possible values
| ------------- | ------------- | ------------- |
| x1  | x1 position: obstacle start  | double
| y1  | y1 position: obstacle start  | double
| x2  | x2 position: obstacle end  | double
| y2  | y2 position: obstacle end  | double

### agent
| Parameter  | Use | Possible values
| ------------- | ------------- | ------------- |
| x  | x initial position | double
| y  | y initial position | double
| n  |  Number of agents, to change crowd density | int
| dx (opt if n=1) | Dispersion of agents along x axis  | double
| dy (opt if n=1) | Dispersion of agents along y axis  | double
| type (opt, default 0) | Agent type (affects the max speed and force desired for elderly, affects the max speed/size/max rotation angle/force social and force obstacle for robot)  | 0: adult / 1: child (unused) / 2: robot (=AV) / 3: elder / 4: immob (stationary)
| purpose (opt, default 0) | Trip purpose of agent (affects the max speed except for robot and elderly)  | 0: unknown / 1: work / 2: leisure
| addwaypoint | id  | id of a waypoint
### Licence
The core `libpedsim` is licensed under LGPL. The ROS integration and extensions are licensed under BSD.

