from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import GroupAction

def generate_launch_description():
    ld = LaunchDescription()

    use_emc = LaunchConfiguration("use_emc", default="false")
    namespace = LaunchConfiguration("_namespace", default="er_kachaka")

    """
    device_id の確認は `ros2 run joy joy_enumerate_devices` で確認すること
    """
    joy_emc = Node(package="joy",
                       executable="joy_node",
                       name="emc_joy_node",
                       namespace="emc",
                       parameters=[{'device_id': 0}]
                   )
    joy_with_emc = Node(package="joy",
                       executable="joy_node",
                       name="joy_node",
                       namespace=namespace,
                       parameters=[{'device_id': 1}]
                   )
    emc_joy = Node(package="erasers_kachaka_control", 
                  executable="emc_joy",
                  namespace=namespace,
             )
    emc_button = GroupAction(
        actions=[
            PushRosNamespace(namespace), joy_emc
        ]
    )
    
    joy_with_emc_group = GroupAction(
        condition=IfCondition(use_emc),
        actions=[
            emc_button,
            joy_with_emc,
            emc_joy
        ]
    )
    joy_without_emc = Node(package="joy",
                       executable="joy_node",
                       name="joy_node",
                       namespace=namespace,
                       condition=UnlessCondition(use_emc),
                       parameters=[{'device_id': 0      }]
                   )

    teleop = Node(package="erasers_kachaka_control", 
                  executable="teleop_joy",
                  namespace=namespace
             )

    ld.add_action(joy_with_emc_group)
    ld.add_action(joy_without_emc)
    ld.add_action(teleop)

    return ld
