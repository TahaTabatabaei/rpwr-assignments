from setuptools import find_packages, setup

package_name = 'scan_filter'

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
    maintainer='zakaria',
    maintainer_email='zakariaouaddi2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['filter_scan = scan_filter.filter_scan:main',
                            'filter_plot = scan_filter.filter_plot:main',
                            'scan_filter_stats = scan_filter.scan_filter_stats:main',
        ],
    },
)
