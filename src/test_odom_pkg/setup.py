import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'test_odom_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='francois',
    maintainer_email='fjs57500@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'clustering_process = test_odom_pkg.clustering_process:main',
            'position_resolver = test_odom_pkg.position_resolver:main',
            'loc_node = test_odom_pkg.loc_node:main',
        ],
    },
)
