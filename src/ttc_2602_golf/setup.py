import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ttc_2602_golf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        'console_scripts': ['ttc_node = ttc_2602_golf.ttc_node:main',
                            'lkp_node = ttc_2602_golf.lkp_node:main'
        ],
    },
)
