from setuptools import find_packages, setup


setup(
    name="store_web_ui",
    version="0.0.0",
    packages=find_packages(exclude=["tests"]),
    package_data={"store_web_ui": ["static/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/store_web_ui"]),
        ("share/store_web_ui", ["package.xml"]),
    ],
    install_requires=["setuptools", "fastapi", "uvicorn[standard]", "numpy", "opencv-python"],
    zip_safe=False,
    maintainer="yongsoo",
    maintainer_email="yongsoo@local.dev",
    description="매장 협동로봇 관제 및 원격 조작 대시보드",
    license="MIT",
    entry_points={"console_scripts": ["web_ui_node = store_web_ui.web_ui_node:main"]},
)
