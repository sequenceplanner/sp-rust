from setuptools import setup

package_name = 'sp_template_node'

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
    author='Kristofer Bengtsson',
    author_email='kristofer.bengtsson@chalmers.se',
    maintainer='Kristofer Bengtsson',
    maintainer_email='kristofer.bengtsson@chalmers.se',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A template for a SP node',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'subscriber_old_school ='
            ' sp_template_node.subscriber_old_school:main',
            'subscriber_lambda = sp_template_node.subscriber_lambda:main',
            'subscriber_member_function ='
            ' sp_template_node.subscriber_member_function:main',
        ],
    },
)
