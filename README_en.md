# erasers_kachaka

<img width=25% /><img src="/imgs/erasers_kachaka_description.png" width=50% />

English | [日本語](README.md)

<!-- 
# Development Guidelines, Manuals, and Troubleshooting Documentation
## 📝Manuals
- [⏩How to connect to Kachaka](/docs/howtoconnect.md)
  - [🔌How to connect via Ethernet](/docs/howtoconnect.md#ethernet)
  - [🛜How to connect wirelessly](/docs/howtoconnect.md#wireless)
- [⏩How to launch Kachaka](/docs/howtobringup.md)
  - [🕹️About launch modes](/docs/howtobringup.md#mode)
  - [🚀About Launch files](/docs/howtobringup.md#launch) 
- [🎮How to control Kachaka with a controller](/docs/howtocontrol.md)
- [🔈How to make Kachaka speak](/docs/howtospeak.md)
- [🗺How to create maps](/docs/howtomap.md)

## ⚒Development
- [🐱Getting started with development](/docs/develop.md)
- [🐳ros2_bridge kachaka Docker container startup check](/docs/erk_docker.md)

## 🗒Tutorials
- [🚗How to move Kachaka](/docs/howtomove.md)
- [🗺How to create maps](/docs/howtomap.md)
- [💫How to navigate](/docs/howtonav.md)

-->

# Setup Instructions

## 1. Create Workspace
Create a `colcon_ws` directory in your home directory by running the following command:
```bash
cd && mkdir -p colcon_ws/src
```

## 2. Download erasers_kachaka Repository
Navigate to the `colcon_ws/src` directory by running the following command:
```bash
cd colcon_ws/src
```
Download erasers_kachaka by running the following command:
```bash
git clone https://github.com/trcp/erasers_kachaka.git
```

## 3. Download Required Packages
Download the packages required to build erasers_kachaka to the src directory by running the following command:
```bash
vcs import . < ./erasers_kachaka/setup.repos
```
This command will download the following packages to the src directory:

- [**kachaka-api**](https://github.com/pf-robotics/kachaka-api.git)
- [kachaka shelf description](https://github.com/GAI-313/kachaka_shelf_description.git)
- [rclpy_util](https://github.com/GAI-313/rclpy_util.git)
- [cartographer](https://github.com/ros2/cartographer.git)
- [cartographer_ros_kachaka](https://github.com/GAI-313/cartographer_ros_kachaka.git)
- [emcl2](https://github.com/GAI-313/emcl2_for_kachaka.git)


> If you want to use OPL, run the following command:
> ```bash
> vcs import . < ./erasers_kachaka/opl.repos
> ```

## 4. Build ros2_bridge Container
Navigate to the `erasers_kachaka` directory by running the following command:
```bash
cd ./erasers_kachaka
```
Copy the necessary files for kachaka-api by running the following command:
```bash
cp docker/Dockerfile.erk ../kachaka-api/
cp customs/grpc_ros2_bridge.trcp.launch.xml ../kachaka-api/ros2/kachaka_grpc_ros2_bridge/launch/
cp customs/dynamic_tf_bridge.cpp ~/colcon_ws/src/kachaka-api/ros2/kachaka_grpc_ros2_bridge/src/dynamic_tf_bridge.cpp
cp customs/static_tf_component.cpp ~/colcon_ws/src/kachaka-api/ros2/kachaka_grpc_ros2_bridge/src/component/static_tf_component.cpp
```

---

Navigate to the `kachaka-api` directory by running the following command:
```bash
cd ../kachaka-api
```
Build the container by running the following command.
Building the container for the first time will take a very long time.
```bash
docker buildx build -t kachaka-api:erasers --target kachaka-grpc-ros2-bridge -f Dockerfile.erk . --build-arg BASE_ARCH=x86_64 --load
```

> [!TIP]
> Building may take a considerable amount of time depending on network conditions. It is recommended to proceed to the next step in a separate terminal while running the above command.

## Install Python kachaka-api
If pip3 is not installed, install pip3 by running the following command:
```bash
sudo apt install -y python3-pip
```
Update pip by running the following command:
```bash
python3 -m pip install --upgrade pip
```
To install on the actual environment, install kachaka-api by running the following command:
```bash
pip install kachaka-api
pip install "scipy>=1.13.0" transform3d matplotlib numpy==1.22.4
```
After successful installation, verify that kachaka-api is installed correctly by running the following command.
If no message is displayed when running the following command, the installation was successful.
```bash
python3 -c "import kachaka_api"
```

## Install Dependencies
Automatically install the necessary dependencies by running the following command:
```bash
cd ~/colcon_ws
sudo apt update && rosdep update
```
```bash
rosdep install -y -i --from-path src --skip-keys=ros2_aruco_interfaces --skip-keys=ros2_aruco
```

## Environment Variable Configuration
Open ~/.bashrc and add the following code at the bottom:
```bash
# kachaka
export KACHAKA_NAME="er_kachaka"
export KACHAKA_IP=192.168.195.125
export KACHAKA_ERK_PATH=~/colcon_ws/src/erasers_kachaka
export GRPC_PORT=26400
export API_GRPC_BRIDGE_SERVER_URI="${KACHAKA_IP}:${GRPC_PORT}"

# ROS
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
Set KACHAKA_IP to the actual IP address of your Kachaka.

> [!IMPORTANT]
> - Set `ROS_DOMAIN_ID` to any number depending on the situation.
> - `GRPC_PORT=26400` is a required variable for communicating with Kachaka. Do not change this value.
> - `ROS_LOCALHOST_ONLY=0` is a required variable for communicating with Kachaka. Do not change this value.
> - For `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, use any DDS depending on the situation.

## Build
Navigate to the `~/colcon_ws` directory:
```bash
cd ~/colcon_ws
```
Build the packages in the workspace by running the following command:
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
```

## Code Modification
- Modify the **`default_map_path`** variable in <a>`/colcon_ws/src/erasers_kachaka/erasers_kachaka/erasers_kachaka_navigation/launch/navigation_launch.py`</a> as needed.
```bash
mkdir ~/map
```

# How to Launch
Launch eR@sers Kachaka by running the following command:
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py
```
For detailed information on how to launch the robot, please refer to
 <a>here</a>.
