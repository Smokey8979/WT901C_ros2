#!/usr/bin/env python3
"""
WT901C485 ROS2 node (Modbus RTU polling)

- Publishes:
    /imu/data         -> sensor_msgs/Imu
    /imu/mag          -> sensor_msgs/MagneticField  (if enabled)
    /imu/temperature  -> sensor_msgs/Temperature    (if enabled)

Parameters (with useful defaults):
- port (string)            : serial port (e.g. /dev/ttyUSB0)
- baud (int)               : serial baud (default 9600)
- rate (double)            : polling rate (Hz)
- frame_id (string)        : TF frame for IMU msgs
- slave_id (int)           : Modbus slave id (default 0x50)

- acc_reg (int)            : register address for accel (default 0x34)
- gyro_reg (int)           : register address for gyro (default 0x37)
- angle_reg (int)          : register address for roll/pitch/yaw (default 0x3A)
- mag_reg (int or 0)       : magnetometer register (0 to disable)
- temp_reg (int or 0)      : temperature register (0 to disable)

- acc_scale_g (double)     : scale of raw accel -> g-range (default 16)
- gyro_scale_dps (double)  : scale of raw gyro -> degrees/sec (default 2000)
- angle_scale_deg (double) : angle register scale (default 180)
- mag_scale (double)       : multiply raw mag -> Tesla (default 1e-6)
- temp_scale (double)      : temperature scale multiplier (default 0.01)
- temp_offset (double)     : temperature offset (degC)

- covariances (arrays)     : covariance params for IMU fields (optional)
"""

import time
import struct
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import Quaternion
import serial

# ---------- Utility: CRC16 (Modbus RTU) ----------
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

# ---------- Node ----------
class WT901CImu(Node):
    def __init__(self):
        super().__init__("wt901c_imu")

        # Parameters
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("rate", 20.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("slave_id", 0x50)

        # Register defaults
        self.declare_parameter("acc_reg", 0x34)
        self.declare_parameter("gyro_reg", 0x37)
        self.declare_parameter("angle_reg", 0x3A)
        self.declare_parameter("mag_reg", 0x3D)   # try common default; set 0 to disable
        self.declare_parameter("temp_reg", 0)     # 0 disables temp read

        # Scales
        self.declare_parameter("acc_scale_g", 16.0)
        self.declare_parameter("gyro_scale_dps", 2000.0)
        self.declare_parameter("angle_scale_deg", 180.0)
        self.declare_parameter("mag_scale", 1e-6)   # default: raw -> Tesla multiplier (adjust if needed)
        self.declare_parameter("temp_scale", 0.01)
        self.declare_parameter("temp_offset", 0.0)

        # Covariances (optional)
        self.declare_parameter("orientation_covariance", [0.001] * 9)
        self.declare_parameter("angular_velocity_covariance", [0.01] * 9)
        self.declare_parameter("linear_acceleration_covariance", [0.1] * 9)

        # Read params
        port = self.get_parameter("port").value
        baud = self.get_parameter("baud").value
        self.rate = float(self.get_parameter("rate").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.slave_id = int(self.get_parameter("slave_id").value)

        self.acc_reg = int(self.get_parameter("acc_reg").value)
        self.gyro_reg = int(self.get_parameter("gyro_reg").value)
        self.angle_reg = int(self.get_parameter("angle_reg").value)
        self.mag_reg = int(self.get_parameter("mag_reg").value)
        self.temp_reg = int(self.get_parameter("temp_reg").value)

        self.acc_scale_g = float(self.get_parameter("acc_scale_g").value)
        self.gyro_scale_dps = float(self.get_parameter("gyro_scale_dps").value)
        self.angle_scale_deg = float(self.get_parameter("angle_scale_deg").value)
        self.mag_scale = float(self.get_parameter("mag_scale").value)
        self.temp_scale = float(self.get_parameter("temp_scale").value)
        self.temp_offset = float(self.get_parameter("temp_offset").value)

        self.orientation_cov = self.get_parameter("orientation_covariance").value
        self.angular_velocity_cov = self.get_parameter("angular_velocity_covariance").value
        self.linear_acceleration_cov = self.get_parameter("linear_acceleration_covariance").value

        # Serial
        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
        except Exception as e:
            self.get_logger().error(f"Unable to open serial port {port}: {e}")
            raise

        # Publishers
        self.pub_imu = self.create_publisher(Imu, "imu/data", 10)
        self.pub_mag = None
        if self.mag_reg:
            self.pub_mag = self.create_publisher(MagneticField, "imu/mag", 10)
        self.pub_temp = None
        if self.temp_reg:
            self.pub_temp = self.create_publisher(Temperature, "imu/temperature", 2)

        # Timer
        period = 1.0 / max(1.0, float(self.rate))
        self.timer = self.create_timer(period, self.update)

        # Log
        self.get_logger().info(f"WT901C IMU node started on {port} @ {baud} baud; slave_id=0x{self.slave_id:02x}")
        self.get_logger().info(f"Polling rate {self.rate} Hz; frame_id={self.frame_id}")

    # Build function 0x03 (read holding registers) Modbus RTU request
    def build_request(self, reg: int, count: int) -> bytes:
        # Slave(1) Function(1) RegAddress(2) Quantity(2)
        frame = struct.pack(">B B H H", self.slave_id, 0x03, reg, count)
        return frame + crc16(frame)

    # Read 'count' registers starting at 'reg' -> returns list of signed 16-bit ints or None
    def read_registers(self, reg: int, count: int = 3) -> Optional[list]:
        req = self.build_request(reg, count)
        try:
            # Purge input buffer to avoid leftover bytes from previous reads
            self.ser.reset_input_buffer()
            self.ser.write(req)
            # Minimal inter-frame gap
            time.sleep(0.01)

            # Modbus response: slave(1) func(1) bytecount(1) data(bytecount) crc(2)
            # bytecount == count * 2
            expected_len = 5 + (count * 2)
            resp = self.ser.read(expected_len)

            if len(resp) < expected_len:
                # Try to read the remainder if available (some adapters/latency)
                remaining = expected_len - len(resp)
                if remaining > 0:
                    resp += self.ser.read(remaining)

            if len(resp) != expected_len:
                self.get_logger().debug(f"Read incomplete: requested {expected_len} got {len(resp)} bytes for reg 0x{reg:02x}")
                return None

            # CRC check
            data_without_crc = resp[:-2]
            received_crc = resp[-2:]
            calc_crc = crc16(data_without_crc)
            if received_crc != calc_crc:
                self.get_logger().warning("CRC mismatch: discarding frame")
                return None

            # Validate slave and function code
            if resp[0] != self.slave_id or resp[1] != 0x03:
                self.get_logger().warning("Unexpected slave/function in response")
                return None

            bytecount = resp[2]
            if bytecount != count * 2:
                self.get_logger().warning("Unexpected byte count in response")
                return None

            payload = resp[3:3 + bytecount]
            regs = []
            for i in range(0, bytecount, 2):
                regs.append(struct.unpack(">h", payload[i:i+2])[0])
            return regs

        except Exception as e:
            self.get_logger().error(f"Serial read/write error: {e}")
            return None

    # Utility to convert Euler (radians) to quaternion message
    def euler_to_quaternion_msg(self, roll, pitch, yaw) -> Quaternion:
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

    # Main periodic update
    def update(self):
        # Read primary registers
        acc_regs = self.read_registers(self.acc_reg, 3)
        gyro_regs = self.read_registers(self.gyro_reg, 3)
        angle_regs = self.read_registers(self.angle_reg, 3)

        if acc_regs is None or gyro_regs is None or angle_regs is None:
            # one or more reads failed; skip this cycle but continue
            self.get_logger().debug("One or more primary register reads failed; skipping publish")
            return

        # Convert raw -> SI
        # Accel: raw / 32768 * acc_scale_g (g) * 9.80665 (m/s^2)
        ax = acc_regs[0] / 32768.0 * self.acc_scale_g * 9.80665
        ay = acc_regs[1] / 32768.0 * self.acc_scale_g * 9.80665
        az = acc_regs[2] / 32768.0 * self.acc_scale_g * 9.80665

        # Gyro: raw / 32768 * gyro_scale_dps -> deg/s -> convert to rad/s
        gx = math.radians(gyro_regs[0] / 32768.0 * self.gyro_scale_dps)
        gy = math.radians(gyro_regs[1] / 32768.0 * self.gyro_scale_dps)
        gz = math.radians(gyro_regs[2] / 32768.0 * self.gyro_scale_dps)

        # Angles: raw / 32768 * angle_scale_deg -> degrees -> convert to radians
        roll = math.radians(angle_regs[0] / 32768.0 * self.angle_scale_deg)
        pitch = math.radians(angle_regs[1] / 32768.0 * self.angle_scale_deg)
        yaw = math.radians(angle_regs[2] / 32768.0 * self.angle_scale_deg)

        # Build IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id

        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az

        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz

        imu_msg.orientation = self.euler_to_quaternion_msg(roll, pitch, yaw)

        # Covariances from params
        imu_msg.orientation_covariance = self.orientation_cov
        imu_msg.angular_velocity_covariance = self.angular_velocity_cov
        imu_msg.linear_acceleration_covariance = self.linear_acceleration_cov

        self.pub_imu.publish(imu_msg)

        # Magnetometer (optional)
        if self.mag_reg and self.pub_mag is not None:
            mag_regs = self.read_registers(self.mag_reg, 3)
            if mag_regs:
                # By default, user sets mag_scale so final unit is Tesla
                mx = mag_regs[0] * self.mag_scale
                my = mag_regs[1] * self.mag_scale
                mz = mag_regs[2] * self.mag_scale

                mag_msg = MagneticField()
                mag_msg.header.stamp = imu_msg.header.stamp
                mag_msg.header.frame_id = self.frame_id
                mag_msg.magnetic_field.x = mx
                mag_msg.magnetic_field.y = my
                mag_msg.magnetic_field.z = mz
                # Leave covariance unknown (default all -1) unless user sets explicit param
                self.pub_mag.publish(mag_msg)

        # Temperature (optional)
        if self.temp_reg and self.pub_temp is not None:
            temp_regs = self.read_registers(self.temp_reg, 1)
            if temp_regs:
                # temp_regs[0] is signed int16; apply scale & offset
                t = temp_regs[0] * self.temp_scale + self.temp_offset
                temp_msg = Temperature()
                temp_msg.header.stamp = imu_msg.header.stamp
                temp_msg.header.frame_id = self.frame_id
                temp_msg.temperature = float(t)
                # covariance left as default
                self.pub_temp.publish(temp_msg)

    def destroy_node(self):
        try:
            if hasattr(self, "ser") and self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


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
