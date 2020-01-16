from setuptools import setup

package_name = '{{ package_name }}'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
    {% for module in modules %}
    '{{ module }}',
    {%- endfor %}
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='{{ author_name }}',
    author_email='{{ email_address }}',
    maintainer='{{ author_name }}',
    maintainer_email='{{ email_address }}',
    keywords=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='{{ description }}',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        {% for script in scripts %}
        '{{ script }}',
        {%- endfor %}
        ],
    },
)
