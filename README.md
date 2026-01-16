# Franka RealSense Camera Extrinsics (ROS Noetic)

This is package provides functionality to manually set the extrinsics of realsense cameras. This is done by adding a gui for users to adjust the pose of the realsense in rviz. To tune the parameters, the  pointcloud of the robot should match the robot model produced by forward kinematics.

![Demo](media/hero-gif.gif)

Note that a control interface and demonstration logging utility is provided by the [panda_utils](https://github.com/jstmn/panda_utils) package.


## Setup
2. Create a ros workspace at `~/ros/franka_ws/src/`
3. Download this repo to `~/ros/franka_ws/src/`
4. Make sure you have `librealsense v2.50.0` installed. You can find out your version by running `realsense-viewer`, it should be shown somewhere in the viewer gui.
5. Download the folder `realsense-ros v2.3.2` from this link: https://github.com/IntelRealSense/realsense-ros/releases/tag/2.3.2. Move `realsense-ros-2.3.2` to `~/ros/franka_ws/src/`
7. (only if you have a D405 realsense) Next, you update `franka_ws/src/realsense-ros-2.3.2/realsense_camera/src/realsense_node_factory.cpp` to support the D405. Specificially, look for `switch(pid)` on line 357. Simply add `case 0x0B5B:    // D405` into to switch case.
8. Setup a python environment. If you are installing [panda_utils](https://github.com/jstmn/panda_utils) then you can use the same one created there. Run `pip install "empy==3.3.4" catkin_pkg`
9. `cd ~/ros/franka_ws/src/; catkin_make` 

## Example usage

``` bash
# ONE TIME SETUP:
echo "ROS_WS='/PATH/TO/ros/franka_ws'" >> ~/.bashrc; source ~/.bashrc # this is to franka_ws/, not ros/

# Terminal 1:
source /opt/ros/noetic/setup.bash; source ${ROS_WS}/devel/setup.bash
roscore

# Terminal 2 - Realsense
source /opt/ros/noetic/setup.bash; source ${ROS_WS}/devel/setup.bash;
roslaunch franka_realsense_extrinsics main.launch \
    json_file_path:=${ROS_WS}/src/franka_realsense_extrinsics/config/realsense_config.json \
    clip_distance:=1.4 filters:="spatial,temporal" \
    depth_width:=640 depth_height:=480 depth_fps:=15 \
    color_width:=640 color_height:=480 color_fps:=15
```
