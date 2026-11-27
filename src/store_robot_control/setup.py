from setuptools import find_packages, setup


setup(
    name="store_robot_control",
    version="0.0.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/store_robot_control"]),
        ("share/store_robot_control", ["package.xml"]),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer="yongsoo",
    maintainer_email="yongsoo@local.dev",
    description="매장 로봇 작업 실행기와 모의 로봇",
    license="MIT",
    entry_points={"console_scripts": [
        "mock_robot = store_robot_control.nodes.mock_robot:main",
        "executor_node = store_robot_control.nodes.executor_node:main",
        "robot_demo = store_robot_control.nodes.robot_demo:main",
        "remote_node = store_robot_control.nodes.remote_node:main",
        "hand_client = store_robot_control.nodes.hand_client:main",
        "teach_zones = store_robot_control.nodes.teach_zones:main",
    ]},
)
