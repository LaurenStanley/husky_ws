Install ROS foxy for Ubuntu 20.04 following these instructions:

https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
Be sure to do the full build, not just bare bones

Source foxy
`source /opt/ros/foxy/setup.bash`

Install nav2 dependencies

`sudo apt install ros-foxy-navigation2`

`sudo apt install ros-foxy-nav2-bringup`

`sudo apt install '~ros-<distro>-turtlebot3-.*'`

Clone the husky_ws into the home directory
NB: this is a whole workspace, not just a package!
`cd`
`git clone https://github.com/LaurenStanley/husky_ws.git`

Install Dependencies

`cd ~/husky_ws`

`rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy`

Build custom-nav-stack

`colcon build --packages-select custom_nav_stack_pkg --symlink-install --cmake-args -DBoost_DEBUG=ON -DBoost_LIBRARY_DIR_RELEASE=/usr/lib/x86_64-linux-gnu`

This is important because the pcl library will not build on its own!!

Build workspace

`colcon build --symlink-install`

This might take a while (10min)

Running Local Nav
`ros2 launch nav2_tutorial robot_operations.launch.py`
`ros2 run custom_nav_stack_pkg cpp_executable2`
`python3 ~/husky_ws/src/nav2_tutorial/scripts/client_python.py`
`ros2 launch local_nav.launch.py`
Robot will then approach the nearest obstacle.

DEBUG
https://github.com/aws-robotics/aws-iot-bridge-example/issues/2

