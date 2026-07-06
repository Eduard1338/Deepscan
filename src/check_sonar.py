import rclpy, struct, sys
from sensor_msgs.msg import PointCloud2
rclpy.init()
node = rclpy.create_node('check')
def cb(msg):
    if msg.width == 0: return
    offs = [f.offset for f in msg.fields]
    print(f'--- Сообщение: {msg.width} точек ---')
    for i in range(min(5, msg.width)):
        b = i*msg.point_step
        x = struct.unpack('<f', bytes(msg.data[b+offs[0]:b+offs[0]+4]))[0]
        y = struct.unpack('<f', bytes(msg.data[b+offs[1]:b+offs[1]+4]))[0]
        z = struct.unpack('<f', bytes(msg.data[b+offs[2]:b+offs[2]+4]))[0]
        print(f'  точка {i}: x={x:.2f}, y={y:.2f}, z={z:.2f}')
    sys.exit(0)
node.create_subscription(PointCloud2, '/sonar/points', cb, 10)
rclpy.spin(node)
