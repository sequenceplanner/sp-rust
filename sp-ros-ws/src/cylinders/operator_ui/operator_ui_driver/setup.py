import os
from glob import glob
from setuptools import setup

package_name = 'operator_ui_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='chalmers',
    author_email='',
    maintainer='chalmers',
    maintainer_email='',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Operator driver',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'operator_ui_driver = operator_ui_driver.main:main',
        ],
    },
)
