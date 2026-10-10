#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, FluidPressure, Temperature, RelativeHumidity
from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool

rclpy.init()
node = Node('hash_extractor')

types = [
    ('imu', Imu),
    ('mag', MagneticField),
    ('press', FluidPressure),
    ('temp', Temperature),
    ('hum', RelativeHumidity),
    ('twist', Twist),
    ('set_bool', SetBool),
]

for name, t in types:
    try:
        th = t.__class__.__name__
        print(f"[{name}] {t}")
    except Exception as e:
        print(f"Error {name}: {e}")

# ros2 topic info or type hash
from rosidl_runtime_py.utilities import get_message, get_service
import rosidl_parser.definition

print("=== rosidl type hash query ===")
import subprocess
out = subprocess.check_output(['ros2', 'interface', 'package', 'sensor_msgs']).decode()
print("sensor_msgs available")
