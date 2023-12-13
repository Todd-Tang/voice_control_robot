from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'path_planner_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='toddtang',
    maintainer_email='chaotoddtang@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_node = path_planner_server.nav_to_pose_action_client:main',
            #'point_action_node = path_planner_server.amcl_pose_update_action_client:main',
            'point_service_node = path_planner_server.odom_to_point:main',
            #'point_client_node = path_planner_server.odom_to_point_client:main',
            'doa_to_point_node = wr_package.doa_to_point:main',
        ],
    },
)
