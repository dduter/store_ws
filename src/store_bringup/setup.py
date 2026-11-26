from glob import glob

from setuptools import find_packages, setup


setup(
    name="store_bringup",
    version="0.0.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/store_bringup"]),
        ("share/store_bringup", ["package.xml"]),
        ("share/store_bringup/launch", glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer="yongsoo",
    maintainer_email="yongsoo@local.dev",
    description="매장 로봇 ROS 2 실행 구성",
    license="MIT",
)
