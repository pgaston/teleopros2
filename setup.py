from setuptools import find_packages, setup
from glob import glob
import os
import sys

package_name = 'teleopros2'
# Get current Python version dynamically
python_version = f"python{sys.version_info.major}.{sys.version_info.minor}"


        #(os.path.join('share', package_name, 'launch'),glob('launch/*.launch.py')),

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, 'jetson_control'],
    #packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #(os.path.join('share', package_name), glob('teleopros2/launch/*launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # used to be here
        # (os.path.join('share', package_name, 'launch'), glob('teleopros2/launch/*.py')),

        # Install web files to share directory instead of site-packages
        # (os.path.join('share', package_name, 'web'), glob('teleopros2/*.html')),
        # (os.path.join('share', package_name, 'web'), glob('teleopros2/*.js')),
        # (os.path.join('share', package_name, 'web'), glob('teleopros2/*.py')),
        # (os.path.join('share', package_name, 'web', 'favicons'), glob('teleopros2/favicons/*')),
        # (os.path.join('share', package_name, 'web', 'certs'), glob('teleopros2/certs/*')),

        # Move over the website files to the package
        (f'lib/{python_version}/site-packages/teleopros2', glob('teleopros2/*.html')),
        (f'lib/{python_version}/site-packages/teleopros2', glob('teleopros2/*.js')),
        (f'lib/{python_version}/site-packages/teleopros2', glob('teleopros2/*.py')),
        (f'lib/{python_version}/site-packages/teleopros2/favicons', glob('teleopros2/favicons/*')),
        (f'lib/{python_version}/site-packages/teleopros2/certs', glob('teleopros2/certs/*')),
     ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pgaston',
    maintainer_email='peter.gaston@gmail.com',
    description='Teleop on ros2 leveraging WebRTC (aiortc).',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleopros2_node = teleopros2.teleopros2:main',        # web server, webrtc      
            'jetbot_move_node = jetson_control.jetbotMove:main',   # jetbot motor control
            'jetson_move_node = jetson_control.jetsonMove:main',   # jetson motor control
        ],
    },
)
