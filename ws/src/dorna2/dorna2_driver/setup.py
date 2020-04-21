from setuptools import setup, find_packages
import xml.etree.ElementTree as ET
from glob import glob

ROOT = ET.parse('package.xml').getroot()
PACKAGE_NAME = ROOT.find('name').text
AUTHOR = ROOT.find('maintainer').text
DESCRIPTION = ROOT.find('description').text
VERSION = ROOT.find('version').text
LICENSE = ROOT.find('license').text

setup(
    tests_require=['pytest'],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'driver = ' 
            + PACKAGE_NAME
            + '.dorna_driver:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'dorna2_driver/dorna/config.yaml']),
        #('share/' + PACKAGE_NAME + '/dorna', glob('dorna2_driver/dorna/*.py')),
        ('share/' + PACKAGE_NAME + '/resources/firmware', glob('dorna2_driver/dorna/resources/firmware/*')),
        ('share/' + PACKAGE_NAME + '/resources/mac', glob('dorna2_driver/dorna/resources/mac/*')),
        ('share/' + PACKAGE_NAME + '/resources/windows', glob('dorna2_driver/dorna/resources/windows/*')),
    ],
    name=PACKAGE_NAME,
    version=VERSION,
    packages=find_packages(),
    zip_safe=True,
    author=AUTHOR,
    maintainer=AUTHOR,
    keywords=['ROS'],
    classifiers=[],
    description=DESCRIPTION,
    license=LICENSE,
)




