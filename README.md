# Teach_Cooperative

>Preparation and Visualization:
```
git clone https://github.com/Krasjet-Yu/Teach_Cooperative.git
cd Teach_Cooperative
catkin_make
source devel/setup.bash
roslaunch simulator rviz_sim.launch
```

>Load Robot Model and Perceptual Map
```
source devel/setup.bash
roslaunch simulator cooperative_simulator.launch
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