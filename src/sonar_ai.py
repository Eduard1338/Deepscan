#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
import numpy as np
import struct
import os
from datetime import datetime
from sklearn.cluster import DBSCAN
import math

def parse_pointcloud2(msg):
    if msg.width == 0 or len(msg.data) == 0:
        return np.empty((0, 3), dtype=np.float32)
    offsets = {field.name: field.offset for field in msg.fields}
    off_x = offsets['x']
    off_y = offsets['y']
    off_z = offsets['z']
    pts = []
    for i in range(msg.width):
        base = i * msg.point_step
        x = struct.unpack('<f', msg.data[base+off_x:base+off_x+4])[0]
        y = struct.unpack('<f', msg.data[base+off_y:base+off_y+4])[0]
        z = struct.unpack('<f', msg.data[base+off_z:base+off_z+4])[0]
        pts.append([x, y, z])
    return np.array(pts, dtype=np.float32)

def detect_objects_adaptive(points, window=5, threshold=0.2, eps=0.5, min_samples=5):
    """
    Очень чувствительный адаптивный детектор.
    """
    if len(points) < min_samples:
        return []
    sorted_idx = np.argsort(points[:, 1])
    pts = points[sorted_idx]
    N = len(pts)
    mask = np.zeros(N, dtype=bool)
    half_win = window // 2

    for i in range(N):
        left = max(0, i - half_win)
        right = min(N, i + half_win + 1)
        local_depths = pts[left:right, 0]
        local_median = np.median(local_depths)
        if pts[i, 0] < local_median - threshold:
            mask[i] = True

    if np.sum(mask) < min_samples:
        return []

    obj_pts = pts[mask]
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(obj_pts[:, :2])
    labels = clustering.labels_
    objects = []
    for lbl in set(labels):
        if lbl == -1:
            continue
        cluster = obj_pts[labels == lbl]
        center = np.mean(cluster, axis=0)
        radius = np.max(np.linalg.norm(cluster - center, axis=1))
        # Временно не фильтруем по радиусу
        width = np.ptp(cluster[:, 1])
        length = np.ptp(cluster[:, 0])
        height = np.ptp(cluster[:, 2])
        objects.append({
            'center_x': float(center[0]),
            'center_y': float(center[1]),
            'center_z': float(center[2]),
            'radius': float(radius),
            'width': float(width),
            'length': float(length),
            'height': float(height),
            'count': len(cluster)
        })
    return objects

class SonarAI(Node):
    def __init__(self):
        super().__init__('sonar_ai')
        self.sub = self.create_subscription(PointCloud2, '/sonar/points', self.cloud_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/deepscan_boat/odom', self.odom_cb, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/detected_objects', 10)
        self.boat_pose = np.zeros(3)
        self.boat_yaw = 0.0
        self.got_pose = False
        self.csv_path = os.path.expanduser('~/deepscan_sonar_data/detections.csv')
        self.all_markers = []
        self.marker_positions = []
        self.distance_threshold = 1.5
        self.next_marker_id = 0
        self.get_logger().info('Sonar AI (ultra-sensitive adaptive) started.')

    def odom_cb(self, msg: Odometry):
        self.boat_pose[0] = msg.pose.pose.position.x
        self.boat_pose[1] = msg.pose.pose.position.y
        self.boat_pose[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.boat_yaw = math.atan2(siny_cosp, cosy_cosp)
        if not self.got_pose:
            self.get_logger().info(f'Got initial pose: ({self.boat_pose[0]:.2f}, {self.boat_pose[1]:.2f}, yaw={self.boat_yaw:.2f})')
        self.got_pose = True

    def cloud_cb(self, msg):
        if not self.got_pose:
            return
        points = parse_pointcloud2(msg)
        if len(points) < 5:
            return

        Xc, Yc, Zc = self.boat_pose
        yaw = self.boat_yaw

        objects = detect_objects_adaptive(points, window=5, threshold=0.2, eps=0.5, min_samples=5)
        if objects:
            self.get_logger().info(f'Found {len(objects)} potential object(s)')

        new_markers = []
        for obj in objects:
            cx, cy, cz = obj['center_x'], obj['center_y'], obj['center_z']
            world_x = Xc - cy * math.sin(yaw)
            world_y = Yc + cy * math.cos(yaw)
            world_z = Zc - cx - 1.2

            is_duplicate = False
            for pos in self.marker_positions:
                dist = np.sqrt((world_x-pos[0])**2 + (world_y-pos[1])**2 + (world_z-pos[2])**2)
                if dist < self.distance_threshold:
                    is_duplicate = True
                    break
            if is_duplicate:
                continue

            self.marker_positions.append((world_x, world_y, world_z))
            self.get_logger().info(
                f'New object at ({world_x:.2f}, {world_y:.2f}, {world_z:.2f}) '
                f'size: W={obj["width"]:.2f}, L={obj["length"]:.2f}, H={obj["height"]:.2f} m'
            )

            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'objects'
            m.id = self.next_marker_id
            self.next_marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = world_x
            m.pose.position.y = world_y
            m.pose.position.z = world_z
            m.scale.x = obj['radius'] * 2.0
            m.scale.y = obj['radius'] * 2.0
            m.scale.z = obj['radius'] * 2.0
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            new_markers.append(m)

            self.save_detection(world_x, world_y, world_z,
                                obj['radius'], obj['width'], obj['length'], obj['height'], obj['count'])

        if new_markers:
            self.all_markers.extend(new_markers)
            self.get_logger().info(f'Added {len(new_markers)} new marker(s). Total: {len(self.all_markers)}')

        markers = MarkerArray()
        markers.markers = self.all_markers
        self.marker_pub.publish(markers)

    def save_detection(self, wx, wy, wz, radius, width, length, height, cnt):
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        with open(self.csv_path, 'a') as f:
            f.write(f"{timestamp},{wx:.3f},{wy:.3f},{wz:.3f},{radius:.3f},{width:.3f},{length:.3f},{height:.3f},{cnt}\n")

def main():
    rclpy.init()
    node = SonarAI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
