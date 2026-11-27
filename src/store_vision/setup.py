from setuptools import find_packages, setup

package_name = 'store_vision'

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
    description='Camera capture, 3D Eye-in-Hand coordinate math, depth layering, and VLM inspection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'realsense_node = store_vision.nodes.realsense_node:main',
            'webcam_node = store_vision.nodes.webcam_node:main',
            'perception_node = store_vision.nodes.perception_node:main',
        ],
    },
)
