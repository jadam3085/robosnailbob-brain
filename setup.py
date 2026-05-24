from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robosnailbob_brain'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.py')),
        ('share/' + package_name + '/config/personalities',
            glob('config/personalities/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jadam',
    maintainer_email='jadam3085@gmail.com',
    description='RoboSnailBob brain: LLM, voice I/O, personality',
    license='MIT',
    entry_points={
        'console_scripts': [
            'voice_io_node = robosnailbob_brain.voice_io_node:main',
            'llm_brain_node = robosnailbob_brain.llm_brain_node:main',
            'mega_bridge_node = robot_bringup.scripts.mega_bridge_node:main',
            'server_gui = robosnailbob_brain.server_gui:main',
        ],
    },
)
