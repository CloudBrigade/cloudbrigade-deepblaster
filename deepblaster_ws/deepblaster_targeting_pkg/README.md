# Cloud Brigade DeepBlaster Navigation Package

## Overview

The DeepBlaster Targeting ROS package creates the deepblaster_targeting_node which decides the action / controller message to send out using the normalized detection error (delta) received from object_detection_node. For more information about the DeepBlaster project, see [DeepBlaster project](https://github.com/CloudBrigade/cloudbrigade-deepblaster).

## License

The source code is released under [Apache 2.0](https://www.apache.org/licenses/LICENSE-2.0).

## Installation

### Prerequisites

The AWS DeepRacer device comes with all the pre-requisite packages and libraries installed to run the DeepBlaster project. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The deepblaster_targeting_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

1. *deepblaster_interfaces_pkg* - This packages contains the custom message and service type definitions used to support DeepBlaster project.

### Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. Set the environment variables required to run Intel OpenVino scripts:

        source /opt/intel/openvino_2021/bin/setupvars.sh

1. Create a workspace directory for the package:

        mkdir -p ~/deepracer_ws
        cd ~/deepracer_ws

1. Clone the entire DeepBlaster project on the DeepRacer device.

        git clone https://github.com/CloudBrigade/cloudbrigade-deepblaster.git
        cd ~/deepracer_ws/cloudbrigade-deepblaster/deepblaster_ws/

1. Fetch unreleased dependencies:

        cd ~/deepracer_ws/cloudbrigade-deepblaster/deepblaster_ws/
        rosws update

1. Resolve the dependencies:

        cd ~/deepracer_ws/cloudbrigade-deepblaster/deepblaster_ws/ && apt-get update
        rosdep install -i --from-path . --rosdistro foxy -y

1. Build the deepblaster_targeting_pkg and deepracer_interfaces_pkg:

        cd ~/deepracer_ws/cloudbrigade-deepblaster/deepblaster_ws/ && colcon build --packages-select deepblaster_targeting_pkg deepracer_interfaces_pkg


## Usage

Although the **deepblaster_targeting_node** is built to work with the DeepBlaster project, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built deepblaster_targeting_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Navigate to the DeepBlaster workspace:

        cd ~/deepracer_ws/cloudbrigade-deepblaster/deepblaster_ws/

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. Source the setup script for the installed packages:

        source ~/deepracer_ws/cloudbrigade-deepblaster/deepblaster_ws/install/setup.bash

1. Launch the deepblaster_targeting_node using the launch script:

        ros2 launch deepblaster_targeting_pkg deepblaster_targeting_pkg_launch.py

## Launch Files

A launch file called deepblaster_targeting_pkg_launch.py is included in this package that gives an example of how to launch the deepblaster_targeting_node.

        from launch import LaunchDescription
        from launch_ros.actions import Node

        def generate_launch_description():
            return LaunchDescription([
                Node(
                    package='deepblaster_targeting_pkg',
                    namespace='deepblaster_targeting_pkg',
                    executable='deepblaster_targeting_node',
                    name='deepblaster_targeting_node'
                )
            ])


## Node Details

### deepblaster_targeting

#### Subscribed topics

| Topic Name | Message Type | Description |
|----------- | ------------ | ----------- |
|/deepblaster_object_detection_pkg/object_detection_delta|DetectionDeltaMsg|Message with Object Detection normalized error (delta) of the detected object from the target (reference) position with respect to x and y axes.|

#### Published Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|deepblaster_target|BlasterCtrlMsg|This message is used to send flywheel, trigger, and servo angle ratios with respect to the device calibration.|

## Resources

* AWS Deepracer Opensource getting started: [https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md](https://github.com/awsdeepracer/aws-deepracer-launcher/blob/main/getting-started.md)
* DeepBlaster project getting started: [https://github.com/CloudBrigade/cloudbrigade-deepblaster/blob/main/getting-started.md](https://github.com/CloudBrigade/cloudbrigade-deepblaster/blob/main/getting-started.md)
