FROM ros:humble-ros-base

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt update && apt install -y --no-install-recommends \
    python3-pip curl \
    ros-humble-gazebo-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

COPY src/ /app/
WORKDIR /app
EXPOSE 8000
CMD ["python3", "web_mission_server.py"]
