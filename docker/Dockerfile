FROM ros:jazzy-ros-base-noble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-cv-bridge \
    libgl1 \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --break-system-packages \
    ultralytics \
    "numpy<2" \
    opencv-python-headless

WORKDIR /workspace
COPY vision_ws/store_yolo_docker /workspace/src/store_yolo_docker

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=40

CMD ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && ros2 run store_yolo_docker yolo_docker_node"]
