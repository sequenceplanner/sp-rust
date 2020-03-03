import setuptools
from glob import glob

with open("README.md", "r") as fh:
    readme = fh.read()

package_name = 'dorna'

setuptools.setup(
    name="dorna",
    version="1.4.2",
    author="Dorna Robotics",
    author_email="info@dorna.ai",
    description="Dorna Python API",
    long_description=readme,
    long_description_content_type='text/markdown',
    url="https://dorna.ai/",
    project_urls={
        'Latest release': 'https://github.com/dorna-robotics/dorna/releases/',
    },
    packages=[package_name],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python :: 3.7',
        "Operating System :: OS Independent",
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    install_requires=[
        "setuptools",
        "PyYAML",
        "numpy",
        "pyserial",
    ],
    license="MIT",
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['README.md']),
        ('share/' + package_name, ['LICENSE']),
        ('share/' + package_name + '/resources/firmware', glob('dorna/resources/firmware/*')),
        ('share/' + package_name + '/resources/mac', glob('dorna/resources/mac/*')),
        ('share/' + package_name + '/resources/windows', glob('dorna/resources/windows/*')),
        ('share/' + package_name + '/', glob('dorna/*.yaml')),
    ],

    zip_safe = False,
)
