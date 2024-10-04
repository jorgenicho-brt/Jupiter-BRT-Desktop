# Jupiter Desktop Tooling
## Summary
As of now provides the following tools
- `ros2_launch` bazel rule


## Requirements
- Ubuntu 22.04, it may work with other versions but this has not been tested.
- [ROS2 humble](https://docs.ros.org/en/humble/Installation.html) in order to have access to have all ros features.
  Other ROS distros may work but this has not been tested.

## Install Steps
1. Clone repositories
    2. Clone brt monorepo `git clone git@github.com:BlueRiverTechnology/brt.git`
    3. Clone this repository `git@github.com:jorgenicho-brt/jupiter-desktop.git`
2. Create symlink  
   This steps allows using this repository as a bazel submodule in the brt monorepo
    - `ln -s <path-to-this-repo>/jupiter-desktop <path-to-brt-monorepo>/brt/autonomy/jupiter/robotics/`
3. Apply patch to brt monorepo
   This step is necessary in order to momentarily change the "visibility" attribute of the required libraries to public
   4. Locate the `patches/ros2.patch` patch file in this repo.
   5. Cd into the brt repo
   6. Apply patch `git apply <path-to-dir>/patches/ros2.patch`

## Use
See the `examples` directory 