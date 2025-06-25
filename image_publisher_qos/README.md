# Image Publisher - ROS 2 Package

## Overview

This ROS 2 package provides a configurable image publisher node that reads an image from the filesystem and publishes it periodically on a specified topic using the `compressed` image transport. It uses a RELIABLE QoS profile, making it suitable for applications requiring guaranteed image delivery.

## Features

* Publishes a static image repeatedly at a configurable rate
* Uses RELIABLE QoS for robust communication
* Easy-to-configure parameters for topic name, image path, and publish rate

## Installation

Make sure you have a ROS 2 workspace set up. Clone this package into the `src` folder of your workspace:

```bash
git clone https://github.com/RizviRaza/ros2_test_tools.git src/ros2_test_tools
cd ~/your_ros2_ws
colcon build --packages-select image_publisher_qos
source install/setup.bash
```

## Usage

Run the image publisher with default or customized parameters:

```bash
ros2 run image_publisher_qos image_publisher_node --ros-args \
  -p topic_name:=image_topic/compressed \
  -p image_path:=image.png \
  -p publish_rate:=1.0
```

## Parameters

| Parameter      | Type   | Description                               | Default Value            |
| -------------- | ------ | ----------------------------------------- | ------------------------ |
| `topic_name`   | string | Topic to publish the image on             | `image_topic/compressed` |
| `image_path`   | string | Path to the image file                    | `image.png`              |
| `publish_rate` | double | Rate (Hz) at which the image is published | `1.0`                    |

## Example

Publishing an image at 5 Hz on a custom topic:

```bash
ros2 run image_publisher_qos image_publisher_node --ros-args \
  -p topic_name:=/camera/static/compressed \
  -p image_path:=/home/user/Pictures/static_image.jpg \
  -p publish_rate:=5.0
```

## QoS Settings

This node uses RELIABLE QoS with default history and depth settings to ensure image delivery even under unreliable network conditions.

## Dependencies

* `rclcpp`
* `sensor_msgs`
* `image_transport`
* `cv_bridge`
* OpenCV
* ROS 2 (Foxy, Galactic, Humble, or newer)

## Contributing

Feel free to contribute by submitting issues or pull requests to improve functionality, documentation, or add new features.

## Maintainer

* Raza Rizvi ([mrazarizvi96@gmail.com](mailto:mrazarizvi96@gmail.com))
