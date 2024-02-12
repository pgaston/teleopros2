from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'teleopros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob('launch/*.launch.py')),
        ('lib/python3.8/site-packages/teleopros2', glob('teleopros2/*.html')),
        ('lib/python3.8/site-packages/teleopros2', glob('teleopros2/*.js')),
        ('lib/python3.8/site-packages/teleopros2', glob('teleopros2/*.py')),        # shouldn't this already work?
        ('lib/python3.8/site-packages/teleopros2/favicons', glob('teleopros2/favicons/*')),
        ('lib/python3.8/site-packages/teleopros2/certs', glob('teleopros2/certs/*')),
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
            'teleopros2 = teleopros2.teleopros2:main',
            # 'testpubimages = ros2webrtc.testpubimages:main',
        ],
    },
)
