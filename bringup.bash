#!/bin/bash

export API_GRPC_BRIDGE_SERVER_URI="${KACHAKA_IP}:${GRPC_PORT}"

. /home/administrator/colcon_ws/install/setup.bash

send_tts_action() {
    local message="$1"
    echo "Action: ${message}"
    ros2 action send_goal "/${KACHAKA_NAME}/kachaka_command/execute" \
        kachaka_interfaces/action/ExecKachakaCommand \
        "{kachaka_command: {command_type: 12, speak_command_text: '${message}'}}"
}

stop_sequence() {
    echo "Starting termination sequence..."
    send_tts_action "すべてのノードを停止します"
    exit 0
}

case "$1" in
    stop)
        stop_sequence
        ;;
    *)
        echo "Bringup Type: ${TYPE_BRINGUP}"
        echo "Shelf Type: ${TYPE_SHELF}"
        
        ros2 launch erasers_kachaka_bringup bringup.launch.py use_rviz:=False
        ;;
esac
