from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import GroupAction

def generate_launch_description():
    ld = LaunchDescription()

    use_emc = LaunchConfiguration("use_emc", default="false")

    print(use_emc)

    """
    device_id の確認は `ros2 run joy joy_enumerate_devices` で確認すること
    """
    joy_emc = Node(package="joy",
                       executable="joy_node",
                       name="emc_joy_node",
                       namespace="er_kachaka_emc",
                       parameters=[{'device_id': 1}]
                   )
    joy_with_emc = Node(package="joy",
                       executable="joy_node",
                       name="joy_node",
                       namespace="er_kachaka",
                       parameters=[{'device_id': 0}]
                   )
    emc_joy = Node(package="erasers_kachaka_control", 
                  executable="emc_joy"
             )
    joy_with_emc_group = GroupAction(
        condition=IfCondition(use_emc),
        actions=[
            joy_emc,
            joy_with_emc,
            emc_joy
        ]
    )
    joy_without_emc = Node(package="joy",
                       executable="joy_node",
                       name="joy_node",
                       namespace="er_kachaka",
                       condition=UnlessCondition(use_emc),
                       parameters=[{'device_id': 0}]
                   )

    teleop = Node(package="erasers_kachaka_control", 
                  executable="teleop_joy"
             )

    ld.add_action(joy_with_emc_group)
    ld.add_action(joy_without_emc)
    ld.add_action(teleop)

    return ld
