from glob import glob
import os

from setuptools import setup

package_name = 'lite3_odom_tf_adapter'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Avtonomna project',
    maintainer_email='local@avtonomna.invalid',
    description='Lite3 odometry, depth mapping, guarded Nav2 requests, and robot web API.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odom_tf_adapter = lite3_odom_tf_adapter.odom_tf_adapter:main',
            'scan_delay = lite3_odom_tf_adapter.scan_delay:main',
            'scan_info_adjuster = lite3_odom_tf_adapter.scan_info_adjuster:main',
            'cmd_vel_gate = lite3_odom_tf_adapter.cmd_vel_gate:main',
            'motion_arbiter = lite3_odom_tf_adapter.motion_arbiter:main',
            'vendor_cmd_vel_bridge = lite3_odom_tf_adapter.vendor_cmd_vel_bridge:main',
            'save_live_map = lite3_odom_tf_adapter.save_map:main',
            'mapping_recorder = lite3_odom_tf_adapter.mapping_recorder:main',
            'robot_api = lite3_odom_tf_adapter.robot_api:main',
        ],
    },
)
