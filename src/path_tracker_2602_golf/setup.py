from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'path_tracker_2602_golf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arthur',
    maintainer_email='hermanfranco5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pure_pursuit_pt_2602_golf = path_tracker_2602_golf.pure_pursuit_pt_2602_golf:main',
            'mpc_pt_2602_golf = path_tracker_2602_golf.mpc_pt_2602_golf:main',
            'csv_path_player_golf = path_tracker_2602_golf.csv_path_player_golf:main',
            'csv_to_nav2_client = path_tracker_2602_golf.csv_to_nav2_client:main'
        ],
    },
)
