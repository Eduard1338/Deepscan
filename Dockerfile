FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Системные зависимости
RUN apt update && apt install -y \
    python3-pip \
    curl lsb-release gnupg2 git

# Добавляем репозиторий ROS2 (без него colcon не установится)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

# Теперь ставим colcon и ROS2
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-humble-ros-base

# Python-зависимости
COPY requirements.txt /tmp/
RUN pip3 install -r /tmp/requirements.txt

# Код приложения
COPY src/ /app/src/
COPY deepscan_ws/ /app/deepscan_ws/

WORKDIR /app

EXPOSE 8000

CMD ["python3", "src/web_mission_server.py"]
