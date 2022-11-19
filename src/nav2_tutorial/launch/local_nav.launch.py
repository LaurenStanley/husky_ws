import launch
import launch_ros
import os


def generate_launch_description(): 
   launch_speed_limit = launch_ros.actions.Node(
      package='nav2_tutorial',
      executable='set_speed_limit.py',
      name='set_speed_limit')
       
   enforce_speed_limit = launch_ros.actions.Node(
      package='nav2_tutorial',
      executable='enforce_speed_limit.py',
      name='enforce_speed_limit')

   #cpp_executable = launch_ros.actions.Node(
   #   package='custom_nav_stack_pkg',
   #   exectuable='cpp_executable2',
   #   name='cpp_executable2'
   #)

   #client_python = launch_ros.actions.Node(
   #   package='nav2_tutorial',
   #   executable='client_python.py',
   #   name='client_python'
   #)

   approach_speed_controller = launch_ros.actions.Node(
      package='nav2_tutorial',
      executable='approach_speed_controller.py',
      name='approach_speed_controller'
   )

   get_obstacle_positions = launch_ros.actions.Node(
      package='nav2_tutorial',
      executable='get_obstacle_positions.py',
      name='get_obstacle_positions'
   )
                             
   return launch.LaunchDescription([
        #launch_speed_limit,
        #enforce_speed_limit,  
        #cpp_executable,
        #client_python,  
        approach_speed_controller,
        get_obstacle_positions
    ])

