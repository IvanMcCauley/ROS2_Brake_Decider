from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start brake_decider node with parameters from YAML
        Node(
            package="ros2_brake_decider",    # the package
            executable="brake_decider_node", # compiled binary
            name="brake_decider", # must match node name
            output="screen", # print logs
            parameters=["config/brake_params.yaml"] # load params
        )
    ])

# Now instead of 3-4 manual terminalsI can just run:
#            ros2 launch ros2_brake_decider brake_decider.launch.py