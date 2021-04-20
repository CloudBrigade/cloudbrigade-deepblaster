from setuptools import setup
import os

package_name = 'deepblaster_targeting_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), ['launch/deepblaster_targeting_pkg_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cloud Brigade DeepBlaster',
    maintainer_email='ctodd@cloudbrigade.com',
    description='This package contains logic for deepblaster_targeting which decides \
                 the action messages to send to blaster ctrl node',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deepblaster_targeting_node = deepblaster_targeting_pkg.deepblaster_targeting_node:main'
        ],
    },
)
