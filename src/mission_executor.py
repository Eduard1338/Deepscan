#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import json, math, subprocess, os, signal, sys, time, threading

class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')
        self.sub = self.create_subscription(String, '/mission_command', self.cmd_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/deepscan_boat/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/mission_status', 10)
        self.odom_sub = self.create_subscription(Odometry, '/deepscan_boat/odom', self.odom_cb, 10)
        self.boat_x = 0.0
        self.boat_y = 0.0
        self.boat_yaw = 0.0
        self.got_odom = False
        self.current_mission = False

    def odom_cb(self, msg):
        self.boat_x = msg.pose.pose.position.x
        self.boat_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.boat_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.got_odom = True

    def cmd_callback(self, msg):
        if self.current_mission:
            self.get_logger().warn("Миссия уже выполняется, дождитесь завершения.")
            self.publish_status("Миссия уже выполняется")
            return
        try:
            area = json.loads(msg.data)
        except:
            self.get_logger().error("Invalid area")
            return
        # Запускаем в отдельном потоке, гарантируя сброс флага
        threading.Thread(target=self._safe_execute, args=(area,), daemon=True).start()

    def _safe_execute(self, area):
        try:
            self.execute_mission(area)
        except Exception as e:
            self.get_logger().error(f"Ошибка миссии: {e}")
        finally:
            self.current_mission = False
            self.publish_status("Миссия завершена! Карта готова.")
            try:
                from builtins import update_mission_status
                update_mission_status(False, result_file="seabed_2d_ai.png")
            except:
                pass

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def execute_mission(self, area):
        self.current_mission = True
        x_min, x_max = area["x_min"], area["x_max"]
        y_min, y_max = area["y_min"], area["y_max"]
        swath = 2.0

        # Запускаем сборщик
        collector_proc = subprocess.Popen(
            ["python3", os.path.expanduser("~/collect_sonar_accurate_smart.py")],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        self.publish_status("Сбор данных запущен")

        # Генерация waypoints
        y = y_min
        direction = 1
        waypoints = []
        while y <= y_max:
            if direction == 1:
                waypoints.append((x_min, y))
                waypoints.append((x_max, y))
            else:
                waypoints.append((x_max, y))
                waypoints.append((x_min, y))
            y += swath
            direction *= -1
        total_wp = len(waypoints)

        # Обход
        for idx, wp in enumerate(waypoints):
            progress_pct = int((idx + 1) / total_wp * 100)
            self.publish_status(f"Точка {idx+1}/{total_wp} ({progress_pct}%)")
            self.get_logger().info(f"Waypoint {idx+1}/{total_wp}: {wp}")

            start_time = time.time()
            arrived = False
            while self.got_odom and (time.time() - start_time) < 30:
                dx = wp[0] - self.boat_x
                dy = wp[1] - self.boat_y
                dist = math.hypot(dx, dy)
                if dist < 0.5:
                    arrived = True
                    break
                desired_yaw = math.atan2(dy, dx)
                yaw_error = desired_yaw - self.boat_yaw
                if yaw_error > math.pi: yaw_error -= 2*math.pi
                elif yaw_error < -math.pi: yaw_error += 2*math.pi
                twist = Twist()
                twist.linear.x = min(0.5, dist)
                twist.angular.z = max(-0.8, min(0.8, yaw_error * 2.0))
                self.pub_cmd.publish(twist)
                time.sleep(0.2)

            self.pub_cmd.publish(Twist())
            time.sleep(0.3)

        # Завершение обхода – останавливаем сборщик
        collector_proc.send_signal(signal.SIGINT)
        collector_proc.wait()
        self.publish_status("Строим карту...")

        # Генерируем карту
        self.get_logger().info("Generating map...")
        subprocess.run(["python3", os.path.expanduser("~/detect_on_map.py")])

        # Финальное уведомление (уже вызывается в _safe_execute)

def main():
    rclpy.init()
    node = MissionExecutor()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
