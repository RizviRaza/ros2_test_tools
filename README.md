# ROS 2 Test Tools

## Overview

**ros2\_test\_tools** is a meta-package that provides tools for testing image transport and QoS configurations in ROS 2 environments. It includes both an image publisher and an image subscriber node, each of which supports customizable parameters for flexible testing setups.

This package is especially useful for developers and researchers working with camera data streams, QoS tuning, and performance evaluation in ROS 2 systems.

## Included Packages

### 1. `image_publisher_qos`

Publishes a static image to a configurable ROS 2 topic using a RELIABLE QoS profile.

* **Parameters**:

  * `topic_name` (string): Topic to publish to (default: `image_topic/compressed`)
  * `image_path` (string): Path to the image file (default: `image.png`)
  * `publish_rate` (double): Rate in Hz to publish the image (default: `1.0`)

### 2. `image_subscriber_qos`

Subscribes to one or more image topics with fully configurable QoS settings.

* **Parameters**:

  * `topic1`, `topic2` (string): Topics to subscribe to
  * `qos_reliability` (string): `reliable` or `best_effort`
  * `qos_history` (string): `keep_last` or `keep_all`
  * `qos_depth` (int): Queue depth when using `keep_last`

## Installation

Clone this repository into your ROS 2 workspace and build:

```bash
cd ~/your_ros2_ws/src
git clone https://github.com/RizviRaza/ros2_test_tools.git ros2_test_tools
cd ~/your_ros2_ws
colcon build
source install/setup.bash
```

## Example Usage

Start the publisher:

```bash
ros2 run image_publisher_qos image_publisher_node --ros-args \
  -p topic_name:=/test/image \
  -p image_path:=/path/to/image.png \
  -p publish_rate:=5.0
```

Start the subscriber:

```bash
ros2 run image_subscriber_qos compressed_image_subscriber --ros-args \
  -p topic1:=/test/image \
  -p qos_reliability:=reliable \
  -p qos_history:=keep_last \
  -p qos_depth:=10
```

## Use Cases

* Evaluating QoS impact on image data delivery
* Testing image\_transport configurations
* Benchmarking image streaming performance
* Simulating camera feeds in testing environments

## Dependencies

* ROS 2 (Foxy, Galactic, Humble, or newer)
* `rclcpp`
* `sensor_msgs`
* `image_transport`
* `cv_bridge`
* OpenCV


## Maintainer

* Raza Rizvi ([smrazarizvi96@gmail.com](mailto:smrazarizvi96@gmail.com))
