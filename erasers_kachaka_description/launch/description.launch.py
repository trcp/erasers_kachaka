#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os
import xml.etree.ElementTree as ET


def _render_prefixed_robot_description(
    xacro_path: str,
    shelf_type: str,
    frame_prefix: str,
) -> str:
    # 1. frame_prefixの検証
    if not frame_prefix:
        raise RuntimeError("frame_prefix is empty")
    if not frame_prefix.endswith('_'):
        raise RuntimeError("frame_prefix must end with '_'")
    if '/' in frame_prefix:
        raise RuntimeError("frame_prefix must not contain '/'")
    if any(c.isspace() for c in frame_prefix):
        raise RuntimeError("frame_prefix must not contain whitespace")

    # 2. xacroを展開
    doc = xacro.process_file(xacro_path, mappings={'shelf_type': shelf_type})
    xml_str = doc.toxml()

    root = ET.fromstring(xml_str)

    # 3. 元のlink名を収集
    original_links = set()
    for link in root.findall('.//link'):
        name = link.get('name')
        if name:
            original_links.add(name)

    # frame_prefixで始まるlinkがすでに存在する場合はRuntimeError
    for name in original_links:
        if name.startswith(frame_prefix):
            raise RuntimeError(f"Link '{name}' already starts with frame_prefix '{frame_prefix}'")

    link_mapping = {name: frame_prefix + name for name in original_links}

    # 4. <link name="..."> のlink定義名へframe_prefixを付ける
    for link in root.findall('.//link'):
        name = link.get('name')
        if name in link_mapping:
            link.set('name', link_mapping[name])

    # 6. <parent link="..."> と <child link="..."> は、対応表を使って置換
    for parent in root.findall('.//parent'):
        link = parent.get('link')
        if link in link_mapping:
            parent.set('link', link_mapping[link])

    for child in root.findall('.//child'):
        link = child.get('link')
        if link in link_mapping:
            child.set('link', link_mapping[link])

    # 7. <gazebo reference="..."> を置換
    for gazebo in root.findall('.//gazebo'):
        ref = gazebo.get('reference')
        if ref in link_mapping:
            gazebo.set('reference', link_mapping[ref])

    # 9. 検証 (fail-closed)
    converted_links = [link.get('name') for link in root.findall('.//link') if link.get('name')]
    if len(converted_links) != len(set(converted_links)):
        raise RuntimeError("Duplicate link names found after translation")

    for name in converted_links:
        if not name.startswith(frame_prefix):
            raise RuntimeError(f"Link '{name}' does not start with prefix '{frame_prefix}'")
        if '/' in name:
            raise RuntimeError(f"Link '{name}' contains '/'")

    original_joints = [joint.get('name') for joint in root.findall('.//joint') if joint.get('name')]
    if len(original_joints) != len(set(original_joints)):
        raise RuntimeError("Duplicate joint names found before translation")

    converted_joints = [joint.get('name') for joint in root.findall('.//joint') if joint.get('name')]
    if len(converted_joints) != len(set(converted_joints)):
        raise RuntimeError("Duplicate joint names found after translation")
    if set(original_joints) != set(converted_joints):
        raise RuntimeError("Joint name set mismatch after translation")

    converted_links_set = set(converted_links)
    for parent in root.findall('.//parent'):
        link = parent.get('link')
        if link not in converted_links_set:
            raise RuntimeError(f"Parent link '{link}' not found in converted link set")
    for child in root.findall('.//child'):
        link = child.get('link')
        if link not in converted_links_set:
            raise RuntimeError(f"Child link '{link}' not found in converted link set")

    return ET.tostring(root, encoding='utf-8').decode('utf-8')


def launch_setup(context, *args, **kwargs):
    namespace_val = LaunchConfiguration('namespace').perform(context)
    frame_prefix_val = LaunchConfiguration('frame_prefix').perform(context)
    robot_description_val = LaunchConfiguration('robot_description').perform(context)
    use_sim_time_val = LaunchConfiguration('use_sim_time').perform(context)
    shelf_type_val = LaunchConfiguration('shelf_type').perform(context)
    debug_val = LaunchConfiguration('debug').perform(context)

    prefixed_robot_description_xml = _render_prefixed_robot_description(
        robot_description_val,
        shelf_type_val,
        frame_prefix_val
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace_val,
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'robot_description': prefixed_robot_description_xml,
                'use_sim_time': use_sim_time_val.lower() == 'true',
            }
        ]
    )

    nodes_to_start = [robot_state_publisher]

    if debug_val.lower() == 'true':
        default_rviz_path = os.path.join(
            get_package_share_directory('erasers_kachaka_description'),
            'rviz', 'description.rviz'
        )
        rviz2 = Node(
            package='rviz2',
            executable='rviz2',
            arguments=[
                '-d', default_rviz_path,
                '-f', frame_prefix_val + 'base_footprint'
            ],
            emulate_tty=True,
        )
        nodes_to_start.append(rviz2)

    return nodes_to_start


def generate_launch_description():
    ld = LaunchDescription()

    # default value
    default_robot_description = os.path.join(
        get_package_share_directory('erasers_kachaka_description'),
        'urdf', 'kachaka.urdf.xacro'
    )

    # configures & arguments
    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value=os.environ.get('KACHAKA_NAME', 'er_kachaka'),
        description='robot namespace'
    )
    declare_frame_prefix = DeclareLaunchArgument(
        'frame_prefix', default_value=os.environ.get('KACHAKA_NAME', 'er_kachaka') + '_',
        description='TF frame prefix'
    )
    declare_robot_descriptione = DeclareLaunchArgument(
        'robot_description', default_value=default_robot_description,
        description='Foll path for robot description'
    )
    declare_debug = DeclareLaunchArgument(
        'debug', default_value='false',
        description='Show robot model only'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='simulation time'
    )
    declare_shelf_type = DeclareLaunchArgument(
        'shelf_type', default_value='0',
        description='Type of shelf.'
    )

    ld.add_action(declare_namespace)
    ld.add_action(declare_frame_prefix)
    ld.add_action(declare_robot_descriptione)
    ld.add_action(declare_debug)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_shelf_type)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
