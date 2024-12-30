from setuptools import setup

package_name = 'jetbot_control_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='A ROS2 control node for JetBot',
    license='License Type',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jetbot_control_node = jetbot_control_node.control_node:main',
        ],
    },
)