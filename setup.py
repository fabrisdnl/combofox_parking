from setuptools import find_packages, setup
import glob

package_name = 'combofox_parking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibt',
    maintainer_email='fabrisdnl.wm@gmail.com',
    description='ComboFox package to set the robot in parking pose',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parking_executor = combofox_parking.controller_node:main'
        ],
    },
)
