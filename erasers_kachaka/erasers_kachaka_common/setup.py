#!/usr/bin/env python3
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'erasers_kachaka_common'

data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append(("share/" + package_name, ["package.xml"]))


def package_files(directory, data_files):
    for path, directories, filenames in os.walk(directory):
        for filename in filenames:
            data_files.append(
                (
                    "share/" + package_name + "/" + path,
                    glob(path + "/**/*.*", recursive=True),
                )
            )
    return data_files

# Add directories
data_files = package_files("scripts", data_files)
data_files = package_files("samples", data_files)

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roboworks',
    maintainer_email='nakatogawagai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kachaka_speak_subscriber = scripts.kachaka_speaker_subscriber:main',
            'emergency_manager = scripts.emergency_manager:emergency_manager',
            'emergency_button = scripts.emergency_manager:emergency_button',
            'battery_manager = scripts.battery_manager:main',
            'volume_manager = scripts.volume_manager:main',
            'lidar_observer = scripts.lidar_observer:lidar_observer',
            'lidar_resampler = scripts.lidar_observer:lidar_resampler',
            'robot_stopper = scripts.robot_stopper:main'
        ],
    },
)
