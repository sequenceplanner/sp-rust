from setuptools import setup

package_name = 'macro_test_gen1'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
    
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='endre',
    author_email='e@e.com',
    maintainer='endre',
    maintainer_email='e@e.com',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='somedescription',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        
        ],
    },
)