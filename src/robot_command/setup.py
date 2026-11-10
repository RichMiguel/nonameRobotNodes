from setuptools import find_packages, setup

package_name = 'robot_command'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['waypoints.yaml']),
        ('share/' + package_name + '/config', ['config/command_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='acelz',
    maintainer_email='richardmiquel19@gmail.com',
    description='Robot Command, Navigation Coordinator, and Real-Time Feedback System',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'command_node = robot_command.command_node:main',
            'voice_cmd_parser = robot_command.command_node:main',
        ],
    },
)
