from glob import glob

from setuptools import setup

package_name = 'imdl_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    # These files are copied to install/imdl_nav/share/imdl_nav/
    # (only linked with --symlink-install). The launch files find them there.
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IMDL',
    maintainer_email='todo@todo.com',
    description='Minimal teaching workspace: SLAM and navigation for Create3 + RPLidar.',
    license='Apache-2.0',
    # command name = file:function.  Usage: ros2 run imdl_nav <name>
    entry_points={
        'console_scripts': [
            'odom_to_tf = imdl_nav.odom_to_tf:main',
            'scan_restamper = imdl_nav.scan_restamper:main',
            'go_to = imdl_nav.go_to:main',
            'waypoints = imdl_nav.waypoints:main',
            'save_on_complete = imdl_nav.save_on_complete:main',
        ],
    },
)
