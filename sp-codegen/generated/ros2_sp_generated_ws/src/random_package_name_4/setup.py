from setuptools import setup

package_name = 'random_package_name_4'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
    
    'a',
    'b',
    'c',
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
    description='somedescr',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        
        'a',
        'b',
        'c',
        'asdf',
        'asdf2',
        ],
    },
)