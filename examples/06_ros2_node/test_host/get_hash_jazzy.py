import rclpy
from sensor_msgs.msg import Imu, MagneticField, FluidPressure, Temperature, RelativeHumidity
from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool

def print_hash(name, msg_type):
    try:
        # rosidl message type hash
        th = msg_type.__type_hash__
        print(f"{name}: {th}")
    except Exception as e:
        print(f"{name} err: {e}")

print("=== Jazzy Type Hashes ===")
print_hash("Imu", Imu)
print_hash("Mag", MagneticField)
print_hash("Press", FluidPressure)
print_hash("Temp", Temperature)
print_hash("Hum", RelativeHumidity)
print_hash("Twist", Twist)
print_hash("SetBool", SetBool)
