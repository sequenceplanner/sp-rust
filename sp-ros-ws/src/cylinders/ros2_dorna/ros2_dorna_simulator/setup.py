import os
from glob import glob
from setuptools import setup

package_name = 'ros2_dorna_simulator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
	(os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='endree',
    author_email='',
    maintainer='endree',
    maintainer_email='',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'ros2_dorna_simulator = ros2_dorna_simulator.ros2_dorna_simulator:main',
        ],
    },
)
