import os
from glob import glob
from setuptools import setup

package_name = 'map_loader'


def package_files(directory):
    """Recursively collect all files in a directory and prepare them for installation."""
    paths = []
    for (path, _, filenames) in os.walk(directory):
        for filename in filenames:
            full_path = os.path.join(path, filename)
            relative_path = os.path.relpath(full_path, start=directory)
            install_path = os.path.join(
                'share', package_name, 'maps', os.path.dirname(relative_path))
            paths.append((install_path, [full_path]))
    return paths


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'rviz2'), glob('rviz2/*.rviz')),
]

# add all files in maps/ directory while preserving the folder structure
data_files += package_files('maps')

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CG',
    maintainer_email='natthawejumjai@gmail.com',
    description='A demo package for loading maps in ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
