from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wall_following_2602_golf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.*')),
        (os.path.join('share', package_name,'config'), glob('config/*.*')),
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
        'console_scripts': ['dist_finder = wall_following_2602_golf.dist_finder:main',
                            'control = wall_following_2602_golf.control:main',
                            'ftw_jd = wall_following_2602_golf.ftw_jd:main'
        ],
    },
)
