from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():
    
    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True",
    )
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="False",
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
    )
    wheel_separation_lr_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17",
    )
    
    wheel_separation_fr_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17",
    )
    
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation_lr = LaunchConfiguration("wheel_separation_lr")
    wheel_separation_fr = LaunchConfiguration("wheel_separation_fr")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["amr_controller", 
                    "--controller-manager", 
                    "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller),
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["simple_velocity_controller", 
                            "--controller-manager", 
                            "/controller_manager"
                ]
            ),
            Node(
                package="amr_controller",
                executable="simple_controller.py",
                parameters=[
                    {"wheel_radius": wheel_radius,
                        "wheel_separation_lr": wheel_separation_lr,
                        "wheel_separation_fr": wheel_separation_fr}],
                condition=IfCondition(use_python),
            ),
            Node(
                package="amr_controller",
                executable="simple_controller",
                parameters=[
                    {"wheel_radius": wheel_radius,
                        "wheel_separation_lr": wheel_separation_lr,
                        "wheel_separation_fr": wheel_separation_fr}],
                condition=UnlessCondition(use_python),
            ),
        ]
    )

    return LaunchDescription(
        [
            use_simple_controller_arg,
            use_python_arg,
            wheel_radius_arg,
            wheel_separation_lr_arg,
            wheel_separation_fr_arg,
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            simple_controller,
        ]
    )