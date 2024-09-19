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
data_files = package_files("config", data_files)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gai',
    maintainer_email='gai@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "service_tts = scripts.service_tts:main",
            "sample_tts = erasers_kachaka_common.tts:__sample",
            "emergency_manager = scripts.robot_manager:emergency_manager",
            "rth_manager = scripts.robot_manager:rth_manager",
            "docking_manager = scripts.robot_manager:docking_manager",
            "battery_manager = scripts.robot_manager:battery_manager",
            "sound_manager = scripts.robot_manager:sound_manager",
        ],
    },
)
