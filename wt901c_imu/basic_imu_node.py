import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import serial
import struct
import math
import time

def crc16(data: bytes):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, 'little')

class WT901CImu(Node):

    def __init__(self):
        super().__init__('wt901c_imu')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('rate', 20.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('rate').value

        self.ser = serial.Serial(port, baud, timeout=0.1)

        self.pub = self.create_publisher(Imu, 'imu/data', 10)
        self.timer = self.create_timer(1.0 / rate, self.update)

        self.SLAVE_ID = 0x50
        self.REG_ACC = 0x34
        self.REG_GYRO = 0x37
        self.REG_ANGLE = 0x3A

        self.get_logger().info('WT901C485 IMU node started')

    def build_request(self, reg, count):
        frame = struct.pack(">B B H H", self.SLAVE_ID, 0x03, reg, count)
        return frame + crc16(frame)

    def read_registers(self, reg):
        self.ser.write(self.build_request(reg, 3))
        time.sleep(0.01)
        resp = self.ser.read(11)
        if len(resp) != 11:
            return None
        payload = resp[3:-2]
        return [struct.unpack(">h", payload[i:i+2])[0] for i in range(0, 6, 2)]

    def update(self):
        acc = self.read_registers(self.REG_ACC)
        gyro = self.read_registers(self.REG_GYRO)
        angle = self.read_registers(self.REG_ANGLE)

        if not acc or not gyro or not angle:
            return

        # Scaling from WT901C datasheet
        ax, ay, az = [v / 32768.0 * 9.80665 * 16 for v in acc]
        gx, gy, gz = [math.radians(v / 32768.0 * 2000) for v in gyro]
        roll, pitch, yaw = [math.radians(v / 32768.0 * 180) for v in angle]

        q = self.euler_to_quaternion(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        msg.orientation = q

        self.pub.publish(msg)

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

def main():
    rclpy.init()
    node = WT901CImu()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
