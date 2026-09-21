FROM ros:humble-ros-base

ENV ROS_DOMAIN_ID=42
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt update && apt install -y --no-install-recommends \
    python3-pip \
    libegl1 libgl1 libgomp1 libglib2.0-0 \
    libusb-1.0-0 libudev1 \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

COPY src/ /app/
WORKDIR /app
CMD ["python3", "mission_executor.py"]
