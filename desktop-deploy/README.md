# Desktop ROS2 Install

This guide describes how to use this repo in order to install several of the brt monorepo's libraries into a linux desktop machine. After
install it'll be possible to use libraries and applications by running them locally without the use of virtualization.
As of this version, the packages installed are as follows:
- Jupiters ROS2 libraries, it should allow running applications through `ros2 run ...` or as part of a launch file.
- adk python libraries and required dependencies.
- FACT-eSim python libraries
- Galileos fact_esim_coordinator and required dependencies

## Requirements
- Ubuntu 22.04, it may work with other versions but this has not been tested.
- [ROS2 humble](https://docs.ros.org/en/humble/Installation.html) in order to have access to have all ros features.
  Other ROS distros may work but this has not been tested.

## Install Steps
1. Clone repositories
   2. Clone brt monorepo `git clone git@github.com:BlueRiverTechnology/brt.git`
   3. Clone this repository `git clone git@github.com:jorgenicho-brt/Jupiter-BRT-Desktop.git`
2. Create symlink  
   This steps allows using this repository as a bazel submodule in the brt monorepo
   - `ln -s <path-to-this-repo>/Jupiter-BRT-Desktop <path-to-brt-monorepo>/brt/autonomy/jupiter/robotics`
3. Apply patch  
   This step is necessary in order to momentarily change the "visibility" attribute of the required libraries to public
   5. Cd into this repository
   6. Apply path `git apply main.path`
4. Cd into the monorepo
5. Run the following bazel cmd:
    ```
   bazel run //autonomy/jupiter/robotics/desktop:install_desktop_app
   ```
   This will install the "app" directory containing all the required libraries and executables in the home directory.  
   Alternatively, a destination path can be passed as an argument to the command above in order to install in that location
   ```
   bazel run //autonomy/jupiter/robotics/desktop:install_desktop_app -- <another/path>
   ```

6. Source the ROS2 environment
    Run the following
   ```
   source ~/app/desktop_setup.sh
   ```

    At this point you can use ROS2 commands, type `ros2 --help` to see what commands are available.

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
