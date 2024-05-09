import os
from glob import glob
from setuptools import setup
from distutils import sysconfig
from distutils.extension import Extension

package_name = 'swisscat_simulation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, f'{package_name}.submodules'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'params'), glob('params/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'action'), glob('action/*')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='coderey',
    maintainer_email='yannis.coderey@epfl.ch',
    description='TODO: Package description',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'transforms = swisscat_simulation.transforms:main',
        'map_publisher = swisscat_simulation.map_publisher:main',
        'ticks2odom = swisscat_simulation.tick_to_odom:main',
        'marv2ekf = swisscat_simulation.marv_to_ekf:main',
        'navwaypoints = swisscat_simulation.nav_waypoints:main',
        'calib_odom = swisscat_simulation.calib_odom:main',
        'sub_plot = swisscat_simulation.sub_plot:main',
        'nav_manager = swisscat_simulation.nav_manager:main',
        'map2gazebo = swisscat_simulation.map2gazebo:main',
        ],
    },
)