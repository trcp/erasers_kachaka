import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'erasers_kachaka_samples'

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

setup(
    name=package_name,
    version='0.0.0',
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
            'simple_navigation_sample = erasers_kachaka_samples.navigation_sample:simple_navigation_sample',
            'speak_sample = erasers_kachaka_samples.speak_sample:speak_sample'
        ],
    },
)