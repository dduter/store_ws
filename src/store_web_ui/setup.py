from setuptools import find_packages, setup

package_name = 'store_web_ui'

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
    description='Real-time monitoring and control web UI node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_ui_node = store_web_ui.web_ui_node:main',
        ],
    },
)
