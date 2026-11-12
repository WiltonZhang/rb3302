import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'course_runner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ldaowen',
    maintainer_email='ldaowen@gmail.com',
    description='RB3302 graded run: visit lettered squares via Nav2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_course = course_runner.run_course:main',
            'odom_sign = course_runner.odom_sign:main',
            'turn_trim = course_runner.turn_trim:main',
        ],
    },
)
