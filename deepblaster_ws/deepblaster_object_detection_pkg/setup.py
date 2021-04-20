from setuptools import setup
import os

package_name = 'deepblaster_object_detection_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['launch/deepblaster_object_detection_pkg_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cloud Brigade DeepBlaster',
    maintainer_email='ctodd@cloudbrigade.com',
    description='This package contains logic for object detection from input camera images \
                 publish the normalized delta of the detected object from the target position.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deepblaster_object_detection_node = deepblaster_object_detection_pkg.deepblaster_object_detection_node:main'
        ],
    },
)
