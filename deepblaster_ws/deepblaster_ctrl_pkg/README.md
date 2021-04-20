# DeepBlaster Ctrl Package

## Overview

The DeepBlaster Ctrl ROS package creates the *deepblaster_ctrl_node* which is part of the Cloud Brigade DeepBlaster application and will be launched from the deepblaster_launcher. More details about the application and the components can be found [here](https://github.com/CloudBrigade/cloudbrigade-deepblaster).

This node is responsible for communicating servo angles, and flywheel and trigger values to the attached DeepBlaster Control Module, and Arduino device.

## License

The source code is released under Apache 2.0 (https://www.apache.org/licenses/LICENSE-2.0).

## Installation

### Prerequisites

The DeepRacer device comes with all the pre-requisite packages and libraries installed to run the deepblaster_ctrl_pkg. More details about pre installed set of packages and libraries on the DeepRacer, and installing required build systems can be found in the [Getting Started](https://github.com/aws-deepracer/aws-deepracer-launcher/blob/main/getting-started.md) section of the AWS DeepRacer Opensource page.

The deepblaster_ctrl_pkg specifically depends on the following ROS2 packages as build and execute dependencies:

* *deepblaster_interfaces_pkg* - This packages contains the custom message and service type definitions used to support DeepBlaster project.

## Downloading and Building

Open up a terminal on the DeepRacer device and run the following commands as root user.

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. Create a workspace directory for the package:

        mkdir -p ~/deepblaster_ws
        cd ~/deepblaster_ws

1. Clone the entire DeepBlaster project on the DeepRacer device.

        git clone https://github.com/CloudBrigade/cloudbrigade-deepblaster.git
        cd ~/deepracer_ws/cb-deepblaster-project/deepblaster_ws/

1. Fetch unreleased dependencies:

        cd ~/deepblaster_ws/deepblaster_ctrl_pkg
        rosws update

1. Resolve the deepblaster_ctrl_pkg dependencies:

        cd ~/deepblaster_ws/deepblaster_ctrl_pkg && rosdep install -i --from-path . --rosdistro foxy -y

1. Build the deepblaster_ctrl_pkg and deepracer_interfaces_pkg:

        cd ~/deepblaster_ws/deepblaster_ctrl_pkg && colcon build --packages-select deepblaster_ctrl_pkg deepracer_interfaces_pkg

## Usage

The deepblaster_ctrl_node provides the core functionality to combine the camera data from the cameras connected to the USB slots at the front of the DeepRacer vehicle. Although the nodes is built to work with the Cloud Brigade DeepBlaster application, it can be run independently for development/testing/debugging purposes.

### Run the node

To launch the built deepblaster_ctrl_node as root user on the DeepRacer device open up another terminal on the DeepRacer device and run the following commands as root user:

1. Switch to root user before you source the ROS2 installation:

        sudo su

1. Source the ROS2 Foxy setup bash script:

        source /opt/ros/foxy/setup.bash

1. Source the setup script for the installed packages:

        source ~/deepblaster_ws/deepblaster_ctrl_pkg/install/setup.bash

1. Launch the deepblaster_ctrl_node using the launch script:

        ros2 launch deepblaster_ctrl_pkg deepblaster_ctrl_pkg_launch.py

## Launch Files

The  deepblaster_ctrl_pkg_launch.py is also included in this package that gives an example of how to launch the deepblaster_ctrl_node.

    from launch import LaunchDescription
    from launch_ros.actions import Node

    def generate_launch_description():
        return LaunchDescription([
            Node(
                package='deepblaster_ctrl_pkg',
                namespace='deepblaster_ctrl_pkg',
                executable='deepblaster_ctrl_node',
                name='deepblaster_ctrl_node'
            )
        ])


## Node Details

### deepblaster_ctrl_node

#### Subscribed Topics

| Topic Name | Message Type | Description |
| ---------- | ------------ | ----------- |
|/deepblaster_targeting_pkg/deepblaster_target|BlasterCtrlMsg|This message holds the flywheel and trigger values, and the servo angle values in the range of [0, 180].|

#### Services

| Service Name | Service Type | Description |
| ---------- | ------------ | ----------- |
|arm_blaster|ArmBlasterSrv|A service that is called arm/disarm (enable/disable) the DeepBlaster Control Module.|
|get_blaster_state|GetBlasterStateSrv|A service that is called to get the current state of the DeepBlaster Control Module.|

## Resources

* Cloud Brigade DeepBlaster Opensource getting started: [https://github.com/CloudBrigade/cloudbrigade-deepblaster/getting-started.md](https://github.com/CloudBrigade/cloudbrigade-deepblaster/getting-started.md)
