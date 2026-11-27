from setuptools import find_packages, setup

package_name = 'store_yolo_docker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Store Cobot Team',
    maintainer_email='team@store.cobot',
    description='Docker GPU YOLOv8n inference node for products and fluorescent vest',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_docker_node = store_yolo_docker.yolo_docker_node:main',
        ],
    },
)
