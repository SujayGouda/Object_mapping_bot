# semantic_slam_mapping_bot

Description:
ROS implementation of semantic SLAM based Object detection and Object Mapping Bot, using the combination of sensors (Kinect), Tensorflow single shot detection, with a two-wheeled differential drive robot



## Dependencies
The following python packges are required:
* python 2.*
* numpy
* sklearn
* sciPy
* openCV
* TensorFlow 1.1* (GPU version)
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
* For a demo of hector_slam, object detection, object mapping use:
```bash
roslaunch gazebo_demo demo.launch
```
* To save the map into pgm and yaml use:
```bash
rosrun map_server map_saver -f ~/my_map
```
* To recall the map saved with semantic objects use:
```bash
roslaunch pub_obj default.launch
```

