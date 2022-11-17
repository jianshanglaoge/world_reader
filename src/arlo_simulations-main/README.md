# arlo_simulations
This meta package simulates the Parallax Arlo Robot with a 6-DOF Interbotix WX250S arm in Gazebo. The inertial values for simulation were obtained using SolidWorks.  dfg

# Install

Please use `Gazebo Version> 9.19.x+`

Install the interbotix packages by running the [install script](https://github.com/Interbotix/interbotix_ros_manipulators/tree/main/interbotix_ros_xsarms/install/amd64). Its easiest to put all the packages into one workspace like this:

```
src
├── arlo_simulations
├── aws-robomaker-small-house-world
├── interbotix_ros_core
├── interbotix_ros_manipulators
└── interbotix_ros_toolboxes
```
Clone/install the repos
|Required Packages    |  Link                                                                                   |
|--------------       | ----------                                                                              |
| aws-house           | [repository](https://github.com/aws-robotics/aws-robomaker-small-house-world)           |
| interbotix_ros_*    | [repository](https://github.com/Interbotix/interbotix_ros_manipulators)                 |

run simulation of ArloRobot (no external dependencies)
```bash
roslaunch arlo_gazebo arlo_sim.launch paused:=True
```
Launch the [xsarm_moveit.launch](https://github.com/Interbotix/interbotix_ros_manipulators/blob/main/interbotix_ros_xsarms/interbotix_xsarm_moveit/launch/xsarm_moveit.launch) file like this ([video](https://youtu.be/k3zkgN7TYTE?t=455)) to verify the interbotix packages are correctly installed.  
<br/>

# Simulating Arlo & wx250s 

* Simulate Arlo robot with mounted wx250s arm:
```bash
roslaunch arlo_gazebo arlo_wx250s_sim.launch rosbridge:=False world_modifier:=EMPTY
```
* Simulate Arlo robot with mounted arm in [small-house](https://github.com/aws-robotics/aws-robomaker-small-house-world):
```bash
roslaunch arlo_gazebo arlo_wx250s_sim.launch rosbridge:=False
```

### ROS Launch args

Please visit the [launch folder](/launch/README.md) for more information.

|Argument               | description                           | default                   |
|-----------------------|:-------------------------------------:|--------------------------:|
|x_pos, y_pos, z_pos    | Set robot spawn location              | (-1.33, 0.55, 0.02)       |
|use_rtabmapviz         | Enable rtabmap GUI visualization      | false                     |
|ns                     | Namespace for the entire robot        | wx250s                    |
|arlo_rviz              | Visualize the Arlo Robot in RViz      | true                      |
|isolate                | Minimal startup                       | false                     |
|teleop                 | Launch teleop_keyboard node           | false                     |




## [img] Arlo Robot running in RViz.
![This is an image](/.github/resources/images/rviz_gazebo.png)

## [img] rtabmap/MapData in RViz.
![This is an image](/.github/resources/images/MapData_after_mapping.png)

## [img] Rtabmap GUI.
Set the `use_rtabmapviz` argument to `true` in the launch file.

In Rtabmap-GUI, enable mapping to populate the rtabmap/MapData as shown in the image below.

![This is an image](/.github/resources/images/rtabmapviz_enable_mapping.jpg)



# Contact_graspnet Service Call
### contact_graspnet_rosbridge
- Rosbridge [implementation](https://github.com/danialdunson/contact_graspnet_ROSBridge) of:
[contact_graspnet](https://github.com/NVlabs/contact_graspnet)
```
@article{sundermeyer2021contact,
  title={Contact-GraspNet: Efficient 6-DoF Grasp Generation in Cluttered Scenes},
  author={Sundermeyer, Martin and Mousavian, Arsalan and Triebel, Rudolph and Fox, Dieter},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2021}
}
```
Start the service call provider with:
```bash
conda activate contact_graspnet_env
python contact_graspnet/ros_ServiceProvider.py
```
<br/>We can launch a [service caller](https://github.com/danialdunson/arlo_simulations/blob/main/arlo_gazebo/scripts/contact_grspnet2movegroup.py), node from the catkin_workspace environment: 
```bash
roslaunch arlo_gazebo pick_place.launch robot_name:=wx250s robot_model:=wx250s tester:=Beta
```
From here, we can request 6-DOF gripper poses by calling the `/contact_graspnet/request_inference` service.
![This is an image](/.github/resources/images/rosbridge_working.png)
This implementation can be further improved by training the contact_graspnet with updated gripper dimensions. The collision mesh files have been aligned with the original panda implementation similiar to [this implementation](https://github.com/NVlabs/contact_graspnet/issues/8#issuecomment-1039480945).
<br/>See these two issue threads:
  - [gripper_finger_length](https://github.com/NVlabs/contact_graspnet/issues/25#issuecomment-1213238791)
  - [train using diff collistion mesh](https://github.com/NVlabs/contact_graspnet/issues/8#issuecomment-932755773)
