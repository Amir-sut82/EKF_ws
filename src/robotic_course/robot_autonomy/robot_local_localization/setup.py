from setuptools import find_packages, setup
from glob import glob     
import os                

package_name = 'robot_local_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # add config and launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'description'), glob('description/*.urdf')),   
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),  
        (os.path.join('share', package_name, 'world'), glob('world/*.sdf')),  
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amir',
    maintainer_email='ahmainejad.sut@gmail.com',
    description='Extended Kalman filter localization for a differential-drive robot using commanded motion, visual odometry, and IMU orientation.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'frame_id_converter_node = robot_local_localization.frame_id_converter:main',
            'ekf_node = robot_local_localization.ekf_node:main',
            'test_node = robot_local_localization.test_node:main',
            'prediction_node = robot_local_localization.prediction_node:main',
            'measurement_node = robot_local_localization.measurement_node:main'
        ],
    },
)

