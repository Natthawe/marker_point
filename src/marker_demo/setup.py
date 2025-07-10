import os
from glob import glob
from setuptools import setup, find_packages


package_name = 'marker_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CG',
    maintainer_email='natthawejumjai@gmail.com',
    description='A demo package for publishing markers in ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_markers = marker_demo.publish_markers:main',
            'publish_markers2 = marker_demo.publish_markers2:main',
            'publish_markers3 = marker_demo.publish_markers3:main',
            'publish_markers4 = marker_demo.publish_markers4:main',
            'publish_markers5 = marker_demo.publish_markers5:main',
            'publish_markers6 = marker_demo.publish_markers6:main',
        ],
    },
)
