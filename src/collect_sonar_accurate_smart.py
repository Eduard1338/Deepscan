#!/usr/bin/env python3
import rclpy, struct, numpy as np, os, signal, sys, math
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from rclpy.node import Node
from datetime import datetime
import open3d as o3d

class AccurateSmartCollector(Node):
    def __init__(self):
        super().__init__('accurate_smart_collector')
        self.sub = self.create_subscription(PointCloud2, '/sonar/points', self.cloud_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/deepscan_boat/odom', self.odom_cb, 10)
        self.points = []               # (Xw, Yw, Zw, R, G, B)
        self.boat_pose = np.zeros(3)
        self.boat_yaw = 0.0
        self.got_pose = False
        self.frame_count = 0
        self.saved = False
        signal.signal(signal.SIGINT, self.signal_handler)
        self.get_logger().info('Accurate smart collector started. Move the boat!')

    def signal_handler(self, sig, frame):
        self.get_logger().info('Saving filtered point cloud...')
        self.save_points()
        self.saved = True
        sys.exit(0)

    def odom_cb(self, msg: Odometry):
        self.boat_pose[0] = msg.pose.pose.position.x
        self.boat_pose[1] = msg.pose.pose.position.y
        self.boat_pose[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.boat_yaw = math.atan2(siny_cosp, cosy_cosp)
        if not self.got_pose:
            self.get_logger().info(f'Got initial odom: x={self.boat_pose[0]:.2f}, y={self.boat_pose[1]:.2f}, yaw={self.boat_yaw:.2f}')
        self.got_pose = True

    def cloud_cb(self, msg: PointCloud2):
        if not self.got_pose or msg.width == 0 or len(msg.data) == 0:
            return
        # Сохраняем каждый 5-й кадр для уменьшения объёма
        self.frame_count += 1
        if self.frame_count % 5 != 0:
            return

        Xc, Yc, Zc = self.boat_pose
        yaw = self.boat_yaw
        offs = [f.offset for f in msg.fields]

        for i in range(msg.width):
            b = i * msg.point_step
            x_s = struct.unpack('<f', bytes(msg.data[b+offs[0]:b+offs[0]+4]))[0]  # глубина
            y_s = struct.unpack('<f', bytes(msg.data[b+offs[1]:b+offs[1]+4]))[0]  # боковое смещение
            z_s = struct.unpack('<f', bytes(msg.data[b+offs[2]:b+offs[2]+4]))[0]  # ~0

            # Пересчёт в мировые координаты с учётом поворота катера
            world_x = Xc + z_s * math.cos(yaw) - y_s * math.sin(yaw)
            world_y = Yc + z_s * math.sin(yaw) + y_s * math.cos(yaw)
            world_z = Zc - x_s - 1.2

            # Цвет по глубине
            depth = x_s
            t = max(0, min(1, (depth - 8.0) / 2.0))
            r, g, b = 1.0 - t, 0.2 * t, t

            self.points.append((world_x, world_y, world_z, r, g, b))

    def save_points(self):
        if not self.points:
            self.get_logger().warn('No points collected.')
            return
        all_pts = np.array(self.points)
        # Вокселизация для ещё большего сжатия
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(all_pts[:, :3])
        pcd.colors = o3d.utility.Vector3dVector(all_pts[:, 3:6])
        downpcd = pcd.voxel_down_sample(voxel_size=0.05)
        pts = np.asarray(downpcd.points)
        colors = np.asarray(downpcd.colors)
        filtered = np.hstack((pts, colors))

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        fname = os.path.expanduser(f'~/deepscan_sonar_data/sonar_accurate_smart_{timestamp}.xyz')
        np.savetxt(fname, filtered, fmt='%.4f', header='X Y Z R G B', comments='')
        self.get_logger().info(f'Saved {len(filtered)} points (was {len(all_pts)}) to {fname}')

def main():
    rclpy.init()
    node = AccurateSmartCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.saved:
            node.save_points()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
