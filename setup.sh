#!/bin/bash
echo "Установка системных зависимостей..."
sudo apt update
sudo apt install -y python3-pip git curl

echo "Установка ROS2..."
# ... (команды установки ROS2 из нашего руководства)

echo "Установка Python-зависимостей..."
pip3 install -r requirements.txt

echo "Сборка ROS2-пакетов..."
cd deepscan_ws
colcon build
source install/setup.bash

echo "Готово! Запустите: python3 src/web_mission_server.py"
