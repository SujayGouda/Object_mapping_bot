# semantic_object_mapping_bot

Description:
ROS implementation of Semantic SLAM based Object detection and Object Mapping Bot, using the combination of sensors (Kinect, Camera), Tensorflow single shot detection, with a two-wheeled differential drive robot

## Video preview
[![Watch the video](https://img.youtube.com/vi/-H25q_Vcol8/default.jpg)](https://www.youtube.com/watch?v=uv7T1tmgn-U)


## Dependencies
The following python packges are required:
* python 2.*
* numpy
* sklearn
* sciPy
* openCV
* TensorFlow 1.1* (GPU version)

## ROS-Packages
* hector_mapping (http://wiki.ros.org/hector_mapping)
* depthimage_to_laserscan (http://wiki.ros.org/depthimage_to_laserscan)
* currently tested in ros melodic in ubuntu 18.04

## Setup
1. Download repository to your catkin workspace:
```bash
git clone https://github.com/or-tal-robotics/object_map.git
```
2. Build:
```bash
catkin_make
```
3. Install SSD image detector for ROS:
```bash
pip install -e object_detector_ssd_tf_ros
```
4. Unzip SSD weights in `object_map/object_detector_ssd_tf_ros/ssd/model/ssd_300_vgg.ckpt.zip`

## Running
* For a demo of slam, object detection, object mapping use:
```bash
roslaunch gazebo_demo demo.launch
```
* To save the map use:
```bash
rosrun map_server map_saver -f ~/my_map
```
* To recall the map saved with semantic objects use:
```bash
roslaunch pub_obj default.launch
```
## rqt-graph
![alt text](https://github.com/SujayGouda/Object_mapping_bot/blob/main/rosgraph.png)

## Nodes list
/Publish_object_to_rviz_node\
/gazebo\
/gazebo_gui\
/hector_mapping\
/object_detection_node\
/object_detector_ssd_node\
/object_mapper_node\
/publisher_state_pub\
/rosout\
/world_map_static_broadcaster\

## Topics list
/OB_Points\
/Objects\
/Theta_List\
/camera/camera_info\
/camera/image_raw\
/camera/image_raw/compressed\
/camera/image_raw/compressed/parameter_descriptions\
/camera/image_raw/compressed/parameter_updates\
/camera/image_raw/compressedDepth\
/camera/image_raw/compressedDepth/parameter_descriptions\
/camera/image_raw/compressedDepth/parameter_updates\
/camera/image_raw/theora\
/camera/image_raw/theora/parameter_descriptions\
/camera/image_raw/theora/parameter_updates\
/camera/parameter_descriptions\
/camera/parameter_updates\
/camera_up/camera_info_up\
/camera_up/image_raw_up\
/camera_up/image_raw_up/compressed\
/camera_up/image_raw_up/compressed/parameter_descriptions\
/camera_up/image_raw_up/compressed/parameter_updates\
/camera_up/image_raw_up/compressedDepth\
/camera_up/image_raw_up/compressedDepth/parameter_descriptions\
/camera_up/image_raw_up/compressedDepth/parameter_updates\
/camera_up/image_raw_up/theora\
/camera_up/image_raw_up/theora/parameter_descriptions\
/camera_up/image_raw_up/theora/parameter_updates\
/camera_up/parameter_descriptions\
/camera_up/parameter_updates\
/clock\
/cmd_vel\
/gazebo/link_states\
/gazebo/model_states\
/gazebo/parameter_descriptions\
/gazebo/parameter_updates\
/gazebo/set_link_state\
/gazebo/set_model_state\
/im_info\
/initialpose\
/joint_states\
/kinect/color/camera_info\
/kinect/depth/camera_info\
/kinect/depth/image_raw\
/kinect/depth/points\
/kinect/parameter_descriptions\
/kinect/parameter_updates\
/map\
/map_metadata\
/object_mapped_values\
/odom\
/poseupdate\
/rosout\
/rosout_agg\
/scan\
/slam_cloud\
/slam_out_pose\
/ssd_image_output\
/syscommand\
/tf\
/tf_static\

## Simulation

### Bot:
<img src="https://github.com/SujayGouda/Object_mapping_bot/blob/main/images/robot.png" width="450" height="300">

#### Bot_Description:
* Two-wheeled Differential drive
* RGB-D Camera (Kinect)
* Two mono cameras


### Gazebo world
![alt text](https://github.com/SujayGouda/Object_mapping_bot/blob/main/images/gazebo_world_side_view.png)

### Rviz visualization
#### * Tensorflow SSD object detection: 

#### * Semantic objects mapping:
![alt text](https://github.com/SujayGouda/Object_mapping_bot/blob/main/images/rviz_top_view.png)
![alt text](https://github.com/SujayGouda/Object_mapping_bot/blob/main/images/rviz_side_view1.png)
![alt text](https://github.com/SujayGouda/Object_mapping_bot/blob/main/images/rviz_side_view2.png)
