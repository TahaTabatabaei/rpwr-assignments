from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtle_party'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tahaos',
    maintainer_email='tabatabaei@uni-bremen.de',
    description='Package for the rpwr course assignment number 03',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow = turtle_party.follow:main',
	    'turtle_tf2_broadcaster = turtle_party.turtle_tf2_broadcaster:main',
        'turtle_tf2_listener = turtle_party.turtle_tf2_listener:main',
        ],
    },
)
