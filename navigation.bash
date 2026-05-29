#!/bin/bash

sleep 15
echo "Map Dr: ${MAP_DIR}"
echo "Map Name: ${MAP_NAME}"

. /home/administrator/colcon_ws/install/setup.bash

send_tts() {
    local message="$1"
    ros2 action send_goal "/${KACHAKA_NAME}/kachaka_command/execute" \
        kachaka_interfaces/action/ExecKachakaCommand \
        "{kachaka_command: {command_type: 12, speak_command_text: '${message}'}}"
}

stop_sequence() {
    if [ "$USE_SLAM" = "True" -a "$BRINGUP_TYPE" = 0 ]; then
        echo "Stopping SLAM mode..."
        send_tts "マップデータ ${MAP_NAME} の作成を終了します"
    fi
    exit 0
}

case "$1" in
    stop)
        stop_sequence
        ;;
    *)
        #echo "Waiting 30 seconds for bringup to stabilize..."
        #sleep 30

        if [ "$USE_SLAM" = "True" -a "$BRINGUP_TYPE" = 0 ]; then
            send_tts "マップデータ ${MAP_NAME} の作成を開始します"
            ros2 launch erasers_kachaka_cartographer cartographer_launch.py \
                map_dir:=${MAP_DIR} map_name:=${MAP_NAME} use_navigation:=${USE_NAVIGATION}
        elif [ "$USE_NAVIGATION" = "True" -a "$BRINGUP_TYPE" = 0  ]; then
            send_tts "マップデータ ${MAP_NAME} を読み込みます"
            ros2 launch erasers_kachaka_navigation navigation_launch.py \
                map_dir:=${MAP_DIR} map_name:=${MAP_NAME}
        fi
        ;;
esac
