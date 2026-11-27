from setuptools import find_packages, setup

package_name = 'store_robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Store Cobot Team',
    maintainer_email='team@store.cobot',
    description='Robot execution, safety guard, reorientation, blackbox logging, and remote teleop',
    license='MIT',
    entry_points={
        'console_scripts': [
            'executor_node = store_robot_control.nodes.executor_node:main',
            'mock_robot = store_robot_control.nodes.mock_robot:main',
            'hand_client = store_robot_control.nodes.hand_client:main',
            'remote_node = store_robot_control.nodes.remote_node:main',
        ],
    },
)
