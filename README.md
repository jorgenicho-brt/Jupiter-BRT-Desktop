# Desktop ROS2 Install

This guide describes how to install Jupiter's ROS2 on a desktop machine.  After install
it will be possible to run jupiter ROS nodes either through `ros2 run ...` or as part of a launch file.

## Requirements
- Ubuntu 22.04, it may work with other versions but this has not been tested.
- (optional) [ROS2 humble](https://docs.ros.org/en/humble/Installation.html) in order to have access to have all ros features.
  Other ROS distros may work but this has not been tested.

## Install Steps
1. Run the following bazel cmd:
    ```
   bazel run //autonomy/jupiter/robotics/desktop:install_desktop_app
   ```
   This will install the "app" directory containing all the required libraries and executables in the home directory
   Alternatively, a destination path can be passed as an argument to the command above in order to install in that location
   ```
   bazel run //autonomy/jupiter/robotics/desktop:install_desktop_app -- <another/path>
   ```

2. Source the ROS2 environment
    Run the following
   ```
   source ~/app/desktop_setup.sh
   ```

    At this point you can use ROS2 commands, type `ros2 --help` to see what commands are available

   Alternatively, it's possible to source the system's ROS2 environment in order to have access to all the ROS2 features
   installed in the local machine.  In order to do that pass the name of the ROS2 distro installed locally as follows:
   ```
   source ~/app/desktop_setup.sh humble
   ```
### Run

In a sourced terminal run the following to publish a jupiter message
```
ros2 topic pub test_msg jupiter_msgs/msg/Xyz {}
```

In another sourced terminal run the following to echo the message
```
ros2 topic echo /test_msg
```
You should see the following output
```shell
x_m: 0.0
y_m: 0.0
z_m: 0.0
---
x_m: 0.0
y_m: 0.0
z_m: 0.0
---

```
