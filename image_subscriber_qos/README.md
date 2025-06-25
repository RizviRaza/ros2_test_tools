# Image Subscriber QoS - ROS 2 Package

## Overview

This ROS 2 package provides a configurable image subscriber node that allows subscribing to two image topics with customizable Quality of Service (QoS) settings. It is useful for scenarios requiring fine-grained control over QoS behavior such as reliability, history policy, and queue depth.

## Features

* Subscribe to two image topics
* Configurable QoS settings:

  * Reliability: `reliable` or `best_effort`
  * History: `keep_last` or `keep_all`
  * Depth: Queue size for `keep_last` policy
* Lightweight and easy to configure

## Installation

Make sure you have a ROS 2 workspace set up. Clone this package into the `src` folder of your workspace:

```bash
git clone https://github.com/RizviRaza/ros2_test_tools.git src/ros2_test_tools
cd ~/your_ros2_ws
colcon build --packages-select image_subscriber_qos
source install/setup.bash
```

## Usage

To run the subscriber node with your desired parameters:

```bash
ros2 run image_subscriber_qos compressed_image_subscriber --ros-args \
  -p topic1:=/topic1 \
  -p topic2:=/topic2 \
  -p qos_reliability:=reliable \
  -p qos_history:=keep_last \
  -p qos_depth:=30
```

## Parameters

| Parameter         | Type   | Description                                      | Example          |
| ----------------- | ------ | ------------------------------------------------ | ---------------- |
| `topic1`          | string | First topic to subscribe to                      | `/camera/image1` |
| `topic2`          | string | Second topic to subscribe to                     | `/camera/image2` |
| `qos_reliability` | string | QoS reliability: `reliable` or `best_effort`     | `reliable`       |
| `qos_history`     | string | QoS history: `keep_last` or `keep_all`           | `keep_last`      |
| `qos_depth`       | int    | Queue depth (used only with `keep_last` history) | `30`             |

## Example

Running with specific image topics:

```bash
ros2 run image_subscriber_qos compressed_image_subscriber --ros-args \
  -p topic1:=/camera/front/image_raw/compressed \
  -p topic2:=/camera/rear/image_raw/compressed \
  -p qos_reliability:=best_effort \
  -p qos_history:=keep_last \
  -p qos_depth:=10
```

## Dependencies

* `rclcpp`
* `sensor_msgs`
* `image_transport`
* ROS 2 (Foxy, Galactic, Humble, or newer)

## Contributing

Contributions are welcome! Please open an issue or pull request to suggest improvements or report bugs.


## Maintainer

* Raza Rizvi ([smrazarizvi96@gmail.com](mailto:smrazarizvi96@gmail.com))
