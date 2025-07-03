import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # 'robot_name' 引数を定義します。
    # この引数が、ロボットの名前空間およびTFプレフィックスとして使われます。
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='kachaka_robot', # デフォルト値を設定
        description='Unique name for the robot instance (e.g., kachaka_0, kachaka_1)'
    )

    # LaunchArgumentsからrobot_nameの値を取得します。
    # これをノードの名前空間やXacroへの引数として利用します。
    robot_name = LaunchConfiguration('robot_name')

    # Xacroファイルのフルパスを取得します。
    # このXacroファイルがロボットのURDF記述を含みます。
    xacro_file_path = os.path.join(
        get_package_share_directory('erasers_kachaka_description'),
        'urdf',
        'erasers_kachaka_container_description.urdf.xacro'
    )

    # Xacroファイルを展開し、robot_nameを引数として渡します。
    # Xacroファイル内でこの'robot_name'引数を使って、各フレーム名にプレフィックスを付けます。
    # 例: base_link -> kachaka_0_base_link
    robot_description_content = Command([
        'xacro ', xacro_file_path,
        ' robot_name:=', robot_name # ここでXacroにrobot_name引数を渡します
    ])
    robot_description = {'robot_description': robot_description_content}

    # robot_state_publisher ノードを定義します。
    # このノードがXacroから生成されたURDFを読み込み、TFをブロードキャストします。
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        namespace=robot_name # robot_state_publisherノード自体に名前空間を適用
    )

    # LaunchDescriptionを構築し、定義した引数とノードを追加します。
    return LaunchDescription([
        robot_name_arg,
        robot_state_publisher_node
    ])