> [!WARNING]
> This is a personal development branch. It is planned to be merged to main in the future, but forking this branch is not recommended.

# erasers_kachaka

<img width=25% /><img src="/imgs/erasers_kachaka_description.png" width=50% />

English | [日本語](README.md)

<details>
<summary>
How to Set Up eR@sers Kachaka in a Local Environment
</summary>

# How to Set Up eR@sers Kachaka in a Local Environment or WSL

> **Important**<br>
> The recommended distribution for WSL is **Ubuntu-22.04**.

1. **Install Docker**<br>
    Execute the following command to install Docker:
    ```bash
    sudo apt update && sudo apt install -y ca-certificates curl gnupg lsb-release &&\
    sudo mkdir -p /etc/apt/keyrings &&\
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg &&\
    echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null &&\
    sudo apt update && sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin &&\
    sudo usermod -aG docker $USER
    ```
    After installation is complete, restart your computer. For WSL, execute the following command in PowerShell:
    ```
    # Execute the command on Windows PowerShell.
    wsl --shutdown
    ```
    After restarting, open the terminal and execute the following command. If Docker version information is displayed, the installation was successful:
    ```bash
    docker -v
    ```

1. **Create a Workspace**<br>
    Execute the following command to move to the home directory:
    ```bash
    cd
    ```
    Execute the following command to create a workspace directory:
    ```bash
    mkdir -p ~/colcon_ws/src
    ```

1. **Download erasers_kachaka**<br>
    Execute the following command to move to `~/colcon_ws/src`:
    ```bash
    cd  ~/colcon_ws/src
    ```
    Execute the following command to download erasers_kachaka:
    ```bash
    git clone -b wsl https://github.com/trcp/erasers_kachaka.git
    ```
    
1. **Build the Kachaka Bridge Container**<br>
    Navigate to the erasers_kachaka directory:
    ```bash
    cd erasers_kachaka
    ```
    Execute the following command to build the `nomap_bridge` container:
    ```bash
    docker compose --env-file kachaka_env build nomap_bridge
    ```
    Execute the following command to build the `official_bridge` container:
    ```bash
    docker compose --env-file kachaka_env build official_bridge
    ```
    Execute the following command to verify that the build was successful:
    ```bash
    docker images
    ```
    ```
    # Execution result
    IMAGE               ID             DISK USAGE   CONTENT SIZE   EXTRA
    gai313/ros2:kachaka_bridge.nomap
                        fde902fd84e2       2.48GB          550MB
    gai313/ros2:kachaka_bridge.official
                        5abff5c97387       2.48GB          550MB
    ```

1. **Download erasers_kachaka Dependency Packages**<br>
    Execute the following command to automatically download dependency packages:
    ```bash
    # Verify that the current directory is erasers_kachaka before executing.
    vcs import .. < ./setup.repos
    ```
    After executing the above command, verify that the dependency packages have been downloaded to `colcon_ws/src` by executing the following command:
    ```
    $ ls ~/colcon_ws/src
    cartographer_ros_kachaka  erasers_kachaka  rclpy_util
    emcl2                     kachaka-api      shelf_description
    ```

1. **Install pip**<br>
    Execute the following command to install the Python package manager `pip`:
    ```bash
    sudo apt install -y python3-pip
    ```
    Next, execute the following command to update the installed pip to the latest version:
    ```bash
    python3 -m pip install --upgrade pip
    ```
    
1. **Install Python Dependency Packages**<br>
    Execute the following command to install Python dependency packages:
    ```bash
    # Verify that the current directory is erasers_kachaka before executing.
    pip3 install -r requirements.txt
    ```
    Execute the following command to verify that `kachaka-api` is available. If no output is displayed, the installation was successful:
    ```bash
    python3 -c "import kachaka_api"
    ```

1. **Automatically Resolve ROS2 Dependencies**<br>
    Navigate to the `~/colcon_ws` directory:
    ```bash
    cd ~/colcon_ws
    ```
    Execute the following command to update apt and rosdep:
    ```bash
    sudo apt update && rosdep update
    ```
    Execute the following command to automatically install ROS2 dependency packages:
    ```bash
    rosdep install -y -i --from-path src --skip-keys=ros2_aruco_interfaces --skip-keys=ros2_aruco
    ```

1. **Build the Workspace**<br>
    Execute the following command to build the workspace:
    ```bash
    # Verify that the current directory is colcon_ws before executing.
    colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
    ```

1. **Add Environment Variables to `~/.bashrc`**<br>
    Add the following environment variables to ~/.bashrc:
    ```bash
    # kachaka
    source ~/colcon_ws/install/setup.bash
    export KACHAKA_NAME="er_kachaka"
    export KACHAKA_IP=192.168.195.125
    export USE_TOF_POINTS=True
    export USE_RVIZ=True
    export BRINGUP_TYPE=0
    export USE_SHELF=True
    export SHELF_TYPE=2
    export KACHAKA_ERK_PATH=~/colcon_ws/src/erasers_kachaka
    ## DO NOT EDIT !!!!
    export GRPC_PORT=26400 
    export API_GRPC_BRIDGE_SERVER_URI="${KACHAKA_IP}:${GRPC_PORT}"

    # ROS
    export ROS_DOMAIN_ID=0
    export ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```
    After writing, execute the following command to reload ~/.bashrc:
    ```bash
    source ~/.bashrc
    ```
    Execute the following command to verify that the environment variables correspond to the launcher arguments:
    ```bash
    ros2 launch erasers_kachaka_bringup bringup.launch.py --show-args
    ```
    With the default settings, the output should look like this:
    ```
    Arguments (pass arguments as '<name>:=<value>'):

    'namespace':
        Robot Namespace
        (default: 'er_kachaka')

    'robot_ip':
        Robot IP address
        (default: '192.168.195.125')

    'bringup_docker':
        Launch docker container automatic.
        (default: 'True')

    'bringup_type':
        Select bringup docker container type: [0, 1]. Please read doc about detail.
        (default: '0')

    'shelf_type':
        Select shelf model type:[0, 1, 2]/ Please read doc about detail.
        (default: '0')

    'publish_tof_pc2':
        Enable publish TOF Pointcloud2 topic from Kachaka front sensor.
        (default: 'True')

    'use_rviz':
        Launch Rviz2
        (default: 'True')

    'use_shelf':
        Docking shelf
        (default: 'false')

    'use_sim_time':
        simulation time
        (default: 'false')
    ```
    
</details>

<details>
<summary>
Set Up eR@sers Kachaka Using Docker
</summary>

# Set Up eR@sers Kachaka Using Docker
## Build the erasers_kachaka Image
Execute the following command to prepare an arbitrary password to be used inside the container:
```bash
export PASSWORD=<password>
```
Execute the following command to build the eR@sers Kachaka container:
```bash
docker compose build --env-file kachaka_env erasers_kachaka
```

> 
> If the following error occurs while building the erasers_kachaka container, the environment variable `PASSWORD` may be undefined or empty. Define an arbitrary password for this variable again and re-execute:
> ```
> chpasswd: (line 1, user USERNAME) password not changed
> ```

## Configure Connection Settings with Kachaka
After the build is complete, open [kachaka_env](/kachaka_env) in the repository root and set the variable `KACHAKA_IP` to the IP address of the Kachaka you want to connect to:
```diff
...
# IP Address for Kachaka
- KACHAKA_IP=192.168.195.121
+ KACHAKA_IP=XXX.XXX.XXX.XXX
...
```

> 
> When connecting to Kachaka via WiFi, you can ask Kachaka "Hey Kachaka, tell me your IP address" to get the IP address.

The kachaka_env file also contains environment variables for using erasers_kachaka. Refer to the following table for the meaning of each variable:

|Variable Name|Description|
|:---|:---|
|**UID**|User permission settings inside the container. Do not edit.|
|**USER_ID**|User permission settings inside the container. Do not edit.|
|**GID**|Group permission settings inside the container. Do not edit.|
|**GROUP_ID**|Group permission settings inside the container. Do not edit.|
|**ROS_DOMAIN_ID**|Set the ROS_DOMAIN_ID to be used inside the container. To prevent interference from other robots, it is recommended to set this to an arbitrary value.|
|**ROS_LOCALHOST_ONLY**|If you do not want to publish topics on the LAN, set this variable to `1`.|
|**RMW_IMPLEMENTATION**|Set the DDS protocol used for ROS2 communication. You can choose between `rmw_fastrtps_cpp` and `rmw_cyclonedds_cpp`.|
|**KACHAKA_NAME**|Define the robot namespace. If `er_kachaka`, topics such as `/er_kachaka/...` are subscribed to and published.|
|**KACHAKA_IP**|Define the IP address of Kachaka.|
|**USE_RVIZ**|Show or hide RViz when starting the erasers_kachaka container.|
|**USE_TOF_POINTS**|Publish PointCloud2 from Kachaka's front ToF camera.|
|**BRINGUP_TYPE**|Select the Kachaka startup mode with 0 or 1. Not used in the **Docker** environment.<br>0: Start without map<br><img src="https://i.imgur.com/IlfDoiT.png"/><br>1: Start with built-in app map<br><img src="https://i.imgur.com/B7ThilZ.png"/>|
|**USE_SHELF**|Choose whether to load the Kachaka shelf.<br>If `True`, the shelf specified in the `SHELF_TYPE` variable below will be loaded.<br><img src="https://i.imgur.com/9MUNfMY.png" /><br>If `False`, Kachaka alone is displayed.<br><img src="https://i.imgur.com/GqBfB6Q.png"/>|
|**SHELF_TYPE**|Define either 2 or 3. Select the type of shelf to be loaded.<br>2: 2-tier shelf<br><img src="https://i.imgur.com/9MUNfMY.png" /><br>3: 3-tier shelf<br><img src="https://i.imgur.com/7ds35UQ.png" />|
|**GRPC_PORT**|Required variable for communication with Kachaka. Do not edit.|
|**API_GRPC_BRIDGE_SERVER_URI**|Required variable for communication with Kachaka. Do not edit.|

## Launch erasers_kachaka
If using the computer for the first time after startup, execute the following command to allow Docker to output to the GUI:
```bash
xhost +
```
There are two options for launching:

- **When Using Maps Created in the Kachaka App**<br>
    Execute this if you want to use the built-in map on Kachaka:
    ```bash
    docker compose --env-file kachaka_env up erasers_kachaka official_bridge
    ```
    <img src="https://i.imgur.com/B7ThilZ.png"/>
- **When Using Without a Map**<br>
    Execute this if you want to use a custom map:
    ```bash
    docker compose --env-file kachaka_env up erasers_kachaka nomap_bridge
    ```
    <img src="https://i.imgur.com/IlfDoiT.png"/>

When the container starts up, a terminal called "Terminator" will open. Use this terminal to operate ROS2 and other functions inside the erasers_kachaka container.
<br><img srcc="https://i.imgur.com/ebz08kS.png"/>

</details>

---

When the connection to Kachaka is successful, Kachaka will speak "Kachaka Start!". The status from the robot is displayed in the Rviz window that appears.
<br><img src="https://i.imgur.com/We4FrEm.jpeg" />

The default information displayed in Rviz is as follows:

||||
|:---:|:---:|:---|
|**Front Camera**|<img src="https://i.imgur.com/OMsDhef.png"/>|Displays video from the Kachaka front camera. The estimated position of detected objects viewed from the front camera is also drawn, but it is not directly drawn on the camera image.|
|**Object Detect Image**|<img src="https://i.imgur.com/YQJqMSO.png"/>|Displays visualization of object information detected from the Kachaka front camera. If "Not Detected Objects" is displayed, it means Kachaka has not detected any objects.|
|**Back Camera**|<img src="https://i.imgur.com/tQFrurW.png"/>|Displays the Kachaka rear camera view.|
|**JoyStick Panel**|<img src="https://i.imgur.com/rVkfEyI.png"/>|A panel that allows you to move Kachaka with a joystick, check battery level, and send speech text.|
|**Kachaka**|<img src="https://i.imgur.com/TQKgWtz.png"/>|Displays the Kachaka robot model (Robot Description). When launching from Docker, the robot's appearance changes depending on the `SHELF_TYPE` in [kachaka_env](kachaka_env).|
|**LiDAR**|<img src="https://i.imgur.com/qymZagf.png"/>|Obstacles detected from the Kachaka LiDAR sensor are shown as purple particles.|
|**Map**|<img src="https://i.imgur.com/NTJoPX0.png"/>|Renders the map created from Kachaka. Displayed when Cartographer, Navigation, etc. are launched.|
|**LocalCostMap**|<img src="https://i.imgur.com/ZRfBbVL.png"/>|Renders the cost map for obstacles around the robot during navigation (bright cyan and red particles).|
|**GlobalCostMap**|<img src="https://i.imgur.com/ZRfBbVL.png"/>|Renders the cost map for the map and obstacles during navigation (light cyan and red particles).|
|**TF**|<img src="https://i.imgur.com/8ulIjro.png"/>|Shows the robot's current coordinate system with X (red axis), Y (green axis), and Z (blue axis).|
|**Path**|<img src="https://i.imgur.com/ZRfBbVL.png"/>|Renders the robot's trajectory during navigation (green line).|
|**Kachaka Detect Object Pose**|<img src="https://i.imgur.com/05aIscc.png"/>|Renders the estimated position of detected objects in X, Y, Z axes. Similar to TF but with thicker axes.|
|**Kachaka Detect Object Marker**|<img src="https://i.imgur.com/05aIscc.png"/>|Renders the estimated size and position of detected objects. The rendered box indicates the estimated size of the detected object, and the detected object name is drawn in the center of the box.|
|**Goal Pose**|<img src="https://i.imgur.com/ZRfBbVL.png"/>|Renders the target position for the robot during navigation (red arrow).|

---

# Tutorials
- [erasers_kachaka Launch Method and Configuration](/erasers_kachaka/erasers_kachaka_bringup/README.md)
- [Operating Kachaka from the JoyStick Panel](/erasers_kachaka/erasers_kachaka_teleop/README.md)
- [How to Create Maps with Cartographer](/erasers_kachaka/erasers_kachaka_cartographer/README.md)
- [How to Enable Autonomous Movement of Kachaka with Navigation](/erasers_kachaka/erasers_kachaka_navigation/README.md)
- [How to Control Kachaka from the ROS2 Command Line](/docs/ros2_command.md)
