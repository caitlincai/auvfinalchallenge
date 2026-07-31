import os
from glob import glob
from setuptools import find_packages, setup

virtualenv_name = "object-detection"
home_path = os.path.expanduser("~")
executable_path = os.path.join(home_path, '.virtualenvs', virtualenv_name, 'bin', 'python')

package_name = 'bluerov2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='canon',
    maintainer_email='canonknightils@gmail.com',
    description='Control nodes for the BlueROV2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'crosshair = bluerov2_control.nodes.crosshair:main',
            'detection_node = bluerov2_control.nodes.detection_node:main',
            'april_tags = bluerov2_control.nodes.april_tags:main',
            'controller_node = bluerov2_control.nodes.controller_node:main',
            'heading = bluerov2_control.nodes.heading:main',
            'depth = bluerov2_control.nodes.depth:main',
            'torpedo_node = bluerov2_control.nodes.torpedo_node:main',
            'image_saver_node = bluerov2_control.nodes.image_saver_node:main',
            'calibrate_camera = bluerov2_control.nodes.calibrate_camera:main',
            'apriltag_fire = bluerov2_control.nodes.apriltag_fire:main',
            'orchestrator = bluerov2_control.nodes.orchestrator:main',
            'arming_client = bluerov2_control.nodes.arming_client:main',
        ],
    },
    options={
        'build_scripts': {
            'executable': executable_path,
        }
    },
)