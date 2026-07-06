#!/usr/bin/env python3
import rclpy
from gazebo_msgs.srv import SpawnEntity

def main():
    rclpy.init()
    node = rclpy.create_node('spawn_cube')
    client = node.create_client(SpawnEntity, '/spawn_entity')
    if not client.wait_for_service(timeout_sec=20.0):
        node.get_logger().error('Service /spawn_entity not available')
        return 1

    import os
    urdf_path = os.path.join(
        os.getenv('DEEPSCAN_WS', '/home/eduard/deepscan_ws'),
        'install/deepscan_gazebo/share/deepscan_gazebo/models/deepscan_boat.urdf'
    )
    with open(urdf_path, 'r') as f:
        xml = f.read()

    req = SpawnEntity.Request()
    req.xml = xml
    req.name = 'deepscan_boat'
    req.initial_pose.position.x = 0.0
    req.initial_pose.position.y = 0.0
    req.initial_pose.position.z = 0.6
    req.initial_pose.orientation.w = 1.0

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info('Spawn succeeded: ' + future.result().status_message)
    else:
        node.get_logger().error('Spawn failed')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
