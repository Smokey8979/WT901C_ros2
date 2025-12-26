#!/usr/bin/env python3
"""
WT901C485 ROS 2 IMU Driver (Modbus RTU / RS-485)

Publishes:
  /imu/data        -> sensor_msgs/Imu
  /imu/mag         -> sensor_msgs/MagneticField (optional)
  /imu/temperature -> sensor_msgs/Temperature (optional)
  /imu/pressure    -> sensor_msgs/FluidPressure (optional)
  /imu/altitude    -> std_msgs/Float64 (optional, derived)

Author: You
License: MIT
"""

import time
import math
import struct
from typing import Optional

import serial
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, Temperature, FluidPressure
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64


# -------------------- Modbus CRC16 --------------------
def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc.to_bytes(2, "little")


# -------------------- Node --------------------
class WT901CImu(Node):

    def __init__(self):
        super().__init__("wt901c_imu")

        # ---------- Core parameters ----------
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("slave_id", 0x50)

        # ---------- Register addresses (Modbus holding registers) ----------
        self.declare_parameter("acc_reg", 0x34)
        self.declare_parameter("gyro_reg", 0x37)
        self.declare_parameter("angle_reg", 0x3A)
        self.declare_parameter("mag_reg", 0x3D)   # set 0 to disable
        self.declare_parameter("temp_reg", 0)     # set 0 to disable
        self.declare_parameter("baro_reg", 0x3F)  # set 0 to disable
        self.declare_parameter("alt_reg", 0)      # optional

        # ---------- Scaling ----------
        self.declare_parameter("acc_scale_g", 16.0)
        self.declare_parameter("gyro_scale_dps", 2000.0)
        self.declare_parameter("angle_scale_deg", 180.0)
        self.declare_parameter("mag_scale", 1e-6)      # raw -> Tesla
        self.declare_parameter("temp_scale", 0.01)     # raw -> °C
        self.declare_parameter("temp_offset", 0.0)
        self.declare_parameter("baro_scale", 1.0)      # raw -> Pa

        # ---------- Read parameters ----------
        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.rate = float(self.get_parameter("rate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.slave_id = int(self.get_parameter("slave_id").value)

        self.acc_reg = int(self.get_parameter("acc_reg").value)
        self.gyro_reg = int(self.get_parameter("gyro_reg").value)
        self.angle_reg = int(self.get_parameter("angle_reg").value)
        self.mag_reg = int(self.get_parameter("mag_reg").value)
        self.temp_reg = int(self.get_parameter("temp_reg").value)
        self.baro_reg = int(self.get_parameter("baro_reg").value)
        self.alt_reg = int(self.get_parameter("alt_reg").value)

        self.acc_scale_g = float(self.get_parameter("acc_scale_g").value)
        self.gyro_scale_dps = float(self.get_parameter("gyro_scale_dps").value)
        self.angle_scale_deg = float(self.get_parameter("angle_scale_deg").value)
        self.mag_scale = float(self.get_parameter("mag_scale").value)
        self.temp_scale = float(self.get_parameter("temp_scale").value)
        self.temp_offset = float(self.get_parameter("temp_offset").value)
        self.baro_scale = float(self.get_parameter("baro_scale").value)

        # ---------- Serial ----------
        self.ser = serial.Serial(self.port, self.baud, timeout=0.5)

        # ---------- Publishers ----------
        self.pub_imu = self.create_publisher(Imu, "imu/data", 10)

        self.pub_mag = None
        if self.mag_reg:
            self.pub_mag = self.create_publisher(MagneticField, "imu/mag", 10)

        self.pub_temp = None
        if self.temp_reg:
            self.pub_temp = self.create_publisher(Temperature, "imu/temperature", 10)

        self.pub_pressure = None
        if self.baro_reg:
            self.pub_pressure = self.create_publisher(FluidPressure, "imu/pressure", 10)

        self.pub_altitude = None
        if self.alt_reg:
            self.pub_altitude = self.create_publisher(Float64, "imu/altitude", 10)

        # ---------- Timer ----------
        self.timer = self.create_timer(1.0 / self.rate, self.update)

        self.get_logger().info("WT901C485 IMU node started")
        self.get_logger().info(f"Port={self.port}, Baud={self.baud}, Rate={self.rate} Hz")

    # -------------------- Modbus helpers --------------------
    def build_request(self, reg: int, count: int) -> bytes:
        frame = struct.pack(">B B H H", self.slave_id, 0x03, reg, count)
        return frame + crc16(frame)

    def read_registers(self, reg: int, count: int) -> Optional[list]:
        self.ser.reset_input_buffer()
        self.ser.write(self.build_request(reg, count))
        time.sleep(0.01)

        expected_len = 5 + count * 2
        resp = self.ser.read(expected_len)
        if len(resp) != expected_len:
            return None

        if resp[-2:] != crc16(resp[:-2]):
            return None

        data = resp[3:-2]
        return [struct.unpack(">h", data[i:i+2])[0] for i in range(0, len(data), 2)]

    def read_registers_32(self, reg: int) -> Optional[int]:
        vals = self.read_registers(reg, 2)
        if not vals:
            return None
        return (vals[0] << 16) | (vals[1] & 0xFFFF)

    # -------------------- Math helpers --------------------
    def euler_to_quaternion(self, roll, pitch, yaw) -> Quaternion:
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        cp, sp = math.cos(pitch/2), math.sin(pitch/2)
        cr, sr = math.cos(roll/2), math.sin(roll/2)

        q = Quaternion()
        q.w = cr*cp*cy + sr*sp*sy
        q.x = sr*cp*cy - cr*sp*sy
        q.y = cr*sp*cy + sr*cp*sy
        q.z = cr*cp*sy - sr*sp*cy
        return q

    # -------------------- Main loop --------------------
    def update(self):
        acc = self.read_registers(self.acc_reg, 3)
        gyro = self.read_registers(self.gyro_reg, 3)
        ang = self.read_registers(self.angle_reg, 3)

        if not acc or not gyro or not ang:
            return

        # --- Scaling ---
        ax, ay, az = [v/32768.0 * self.acc_scale_g * 9.80665 for v in acc]
        gx, gy, gz = [math.radians(v/32768.0 * self.gyro_scale_dps) for v in gyro]
        roll, pitch, yaw = [math.radians(v/32768.0 * self.angle_scale_deg) for v in ang]

        # --- IMU message ---
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id

        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = gy
        imu.angular_velocity.z = gz

        imu.orientation = self.euler_to_quaternion(roll, pitch, yaw)
        self.pub_imu.publish(imu)

        # --- Magnetometer ---
        if self.pub_mag:
            mag = self.read_registers(self.mag_reg, 3)
            if mag:
                msg = MagneticField()
                msg.header = imu.header
                msg.magnetic_field.x = mag[0] * self.mag_scale
                msg.magnetic_field.y = mag[1] * self.mag_scale
                msg.magnetic_field.z = mag[2] * self.mag_scale
                self.pub_mag.publish(msg)

        # --- Temperature ---
        if self.pub_temp:
            temp = self.read_registers(self.temp_reg, 1)
            if temp:
                msg = Temperature()
                msg.header = imu.header
                msg.temperature = temp[0] * self.temp_scale + self.temp_offset
                self.pub_temp.publish(msg)

        # --- Barometer ---
        if self.pub_pressure:
            p_raw = self.read_registers_32(self.baro_reg)
            if p_raw is not None:
                msg = FluidPressure()
                msg.header = imu.header
                msg.fluid_pressure = p_raw * self.baro_scale
                msg.variance = 0.0
                self.pub_pressure.publish(msg)

        # --- Altitude (optional) ---
        if self.pub_altitude:
            alt_raw = self.read_registers_32(self.alt_reg)
            if alt_raw is not None:
                msg = Float64()
                msg.data = alt_raw * 0.01  # cm → m (common)
                self.pub_altitude.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WT901CImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
