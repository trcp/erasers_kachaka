#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess

import os

KACHAKA_NAME = os.environ.get('KACHAKA_NAME')


def generate_launch_description():
    ld = LaunchDescription()

    bringup_docker = ExecuteProcess(
        cmd=[[
            "docker compose",
            " -f %s/docker/docker-compose.yaml"%os.environ.get('KACHAKA_ERK_PATH'),
            " up kachaka"
        ]],
        shell=True
    )

    ld.add_action(bringup_docker)

    return ld
