This is a ROS2 package for setting up and using WT901C485 RS-485type modbus mode imu module

This package provides a **plug-and-play ROS 2 node** that polls the WT901C485 IMU over RS-485 and publishes standard ROS sensor messages, making it suitable for **robot_localization**, **Nav2**, and general robotics applications.

---

## 🔌 Hardware Setup

### Required
- WT901C485 IMU
- USB → RS-485 converter (CH340 / FTDI, auto-direction)
- External **5V power supply** for IMU

### Wiring

| WT901C485 | RS-485 Adapter |
|----------|---------------|
| A / A+ | A / A+ |
| B / B- | B / B- |
| GND | GND |
| VCC | External 5V |

⚠️ **Do NOT power the IMU from TTL VCC pins** on converters.  
⚠️ **Do NOT use UART/TTL mode** for this IMU variant.


## 🖥️ Software Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python ≥ 3.10
- `pyserial`

Install dependency:
```bash
pip3 install pyserial

---

*** INSTALLATION AND RUNNING ***
  --setting up and cloning the package--
  cd ros2_ws/src
  git clone https://github.com/<your-username>/wt901c_imu.git
  cd wt901c_imu
  chmod +x install.sh
  ./install.sh
  sudo reboot

  --Running the package--
  cd ros2_ws
  colcon build --packages-select wt901c_imu
  source install/setup.bash


  NOTE: run wit_basic_imu_node for only basic imu 
        run wit_imu_node_mag_tem for imu, magnitude and temperature values  
        run full_imu_node for imu, magnitude, temperature and presure values

        TEMPERATURE READINGS AND DATA ARE NOT AVAILABLE RIGHT NOW !!!


  ** To run basic only imu **

  ros2 run wt901c_imu wit_basic_imu_node \
    --ros-args \
    -p port:=/dev/ttyUSB0 \
    -p frame_id:=imu_link


  ** To run with mag and temp **

  ros2 run wt901c_imu wit_imu_node_mag_tem \
    --ros-args -p port:=/dev/ttyUSB0 -p frame_id:=imu_link -p rate:=20.0


  ** To run with all **

  ros2 run wt901c_imu full_imu_node  \
    --ros-args \
    -p port:=/dev/ttyUSB0 \    
    -p rate:=20.0 \
    -p mag_reg:=61 \
    -p baro_reg:=63


## 📦 Published Topics

| Topic | Message Type | Description |
|------|-------------|------------|
| `/imu/data` | `sensor_msgs/Imu` | Accel, gyro, orientation |
| `/imu/mag` | `sensor_msgs/MagneticField` | Magnetometer (optional) |
| `/imu/temperature` | `sensor_msgs/Temperature` | Temperature (optional) |   !!! This is currently not working !!!
| `/imu/pressure` | `sensor_msgs/FluidPressure` | Barometric pressure (optional) |
---


## ✨ Features

- ✅ RS-485 **Modbus RTU polling** (industrial-correct, reliable)
- ✅ Accelerometer
- ✅ Gyroscope
- ✅ Orientation (Euler → Quaternion)
- ✅ Magnetometer
- ✅ Temperature (optional)
- ✅ Barometer / Pressure (optional)
- ✅ ROS-standard messages
- ✅ Parameter-driven (portable across devices)
- ✅ Works on **any ROS 2 Humble system**

---

FOR HELP WRITE TO : sdhudu@gmail.com