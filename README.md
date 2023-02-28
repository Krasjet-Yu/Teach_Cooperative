# Teach_Cooperative

>Preparation and Visualization:
```
cd Teach_Cooperative
catkin_make
source devel/setup.zsh
roslaunch simulator rviz_sim.launch
```

>Load Robot Model and Perceptual Map
```
source devel/setup.bash
roslaunch simulator  cooperative_simulator.launch
```
   

>Struct
```
├── mapping
│   ├── config
│   ├── include
│   ├── launch
│   └── src
├── planning
│   ├── TODO: UGV_Planning
│   ├── TODO: UAV_Planning
├── quadrotor_msgs
│   ├── msg
└── simulator
    ├── local_sensing
    ├── mockamap
    ├── odom_vis
    ├── simulator
    ├── sitl
    ├── so3_controller
    └── so3_quadrotor
```