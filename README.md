# cam_publisher

## Overview

The `cam_publisher` package is a ROS node developed to capture images from a camera and publish them on a ROS topic. It uses OpenCV for image capture and `cv_bridge` to convert OpenCV images to ROS messages. The camera calibration parameters are loaded from a YAML file.

**Keywords:** ROS, OpenCV, Camera, Image Publisher

### Building from Source

#### Dependencies

- Robot Operating System (ROS)
- OpenCV
- YAML-CPP

```sh
sudo rosdep install --from-paths src
```

## Config Files

### Camera Configuration

Camera configuration files, such as (`config/front_camera.yaml`):

```yaml
image_width: 1920
image_height: 1080
camera_matrix:
  rows: 3
  cols: 3
  data: [1332.2905012603742, 0.0, 1001.619419558725, 0.0, 1313.963171950812, 491.9312293181554, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.5176939966889225, 0.17338115766870202, 0.023434654253362403, -0.008230427613718875, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [862.3757981998015, 0.0, 958.1924244615784, 0.0, 0.0, 1187.5335378689128, 503.67812178921946, 0.0, 0.0, 0.0, 1.0, 0.0]
```

## Launch Files

### Main Launch File

Main launch file (`launch/cam_publisher.launch`):

```xml
<launch>
  <node name="image_publisher" pkg="cam_publisher" type="camera_node" output="screen">
    <param name="camera_info_yaml" value="$(find cam_publisher)/config/front_camera.yaml"/>
    <param name="device" value="2"/>
    <param name="topic_name" value="camera"/>
  </node>
</launch>
```

## Nodes

### image_publisher

The `image_publisher` node reads images from a camera and publishes the images on a ROS topic.

#### Published Topics

- `/camera/image` (`sensor_msgs/Image`): Images captured by the camera.

#### Parameters

- `camera_info_yaml` (string): Path to the YAML file containing the camera calibration parameters.
- `device` (int): Camera device number.
- `topic_name` (string): Name of the topic where images will be published. Default: "camera".
