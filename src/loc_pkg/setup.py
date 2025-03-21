from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'loc_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py')))

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
            "localization_node = loc_pkg.localization_node:main"
        ],
    },
)
