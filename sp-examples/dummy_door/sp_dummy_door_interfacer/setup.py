import os
from glob import glob
from setuptools import setup

package_name = 'sp_dummy_door_interfacer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
    	'src.sp_dummy_door_interfacer',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
	(os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='RandomName',
    author_email='',
    maintainer='RandomName',
    maintainer_email='',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Autogenerated ROS2 package',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'dummy_door_interfacer = src.sp_dummy_door_interfacer:main',
        ],
    },
)
