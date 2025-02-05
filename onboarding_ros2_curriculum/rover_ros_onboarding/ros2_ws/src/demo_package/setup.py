import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'demo_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.xml"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dani',
    maintainer_email='daniellemclaughlin593@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = demo_package.motor_controller:main',
            'motor_degree = demo_package.motor_degree:main',
        ],
    },
)
