TODO: Absolute file paths are given in macros. Change it according to the your username (for example my_robot_cell_macro.xacro line 90 
mesh filename="file:////home/cem/colcon_ws/src/Universal_Robots_ROS2_Description/meshes/ur10e/collision/linear_axis_moving_link.stl" scale="0.001 0.001 0.001"/>
to 
mesh filename="file:////home/ifarlab/colcon_ws/src/Universal_Robots_ROS2_Description/meshes/ur10e/collision/linear_axis_moving_link.stl" scale="0.001 0.001 0.001"/)
cd colcon_ws/
rosdep update
rosdep install --ignore-src --from-paths src -y
colcon build
To bringup dual robot system, ros2 launch my_robot_cell_gz dualrobot_ifarlab_gazebo.launch.py
To run the MoveIt2! setup, ros2 launch my_robot_cell_gz dual_robot_moveit_program.launch.py
