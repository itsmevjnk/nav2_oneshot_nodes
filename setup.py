from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'nav2_oneshot_nodes'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itsmevjnk',
    maintainer_email='ngtv0404@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'localisation_init = nav2_oneshot_nodes.localisation_init:main',
            'set_goal = nav2_oneshot_nodes.set_goal:main',
            'sc_localisation_init = nav2_oneshot_nodes.sc_localisation_init:main',
            'sc_set_goal = nav2_oneshot_nodes.sc_set_goal:main',
            'sc_init_and_goal = nav2_oneshot_nodes.sc_init_and_goal:main',
        ],
    },
)
