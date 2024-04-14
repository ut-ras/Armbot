from setuptools import find_packages, setup
from glob import glob
package_name = 'dynamixel_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),  # Add this line
    ],
    install_requires=['setuptools', 'dynamixel_sdk'],
    zip_safe=True,
    maintainer='Jacob Tomczeszyn',
    maintainer_email='jake.tomczeszyn@gmail.com',
    description='Package for controlling Armbot\'s Dynamixel motors using the Dynamixel SDK',    
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_node = dynamixel_control.dynamixel:main',

        ],
    },
)