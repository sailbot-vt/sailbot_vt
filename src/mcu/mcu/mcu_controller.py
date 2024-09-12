import time
import serial
from serial.tools import list_ports

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32


TEENSY_VID = 0x16c0
TEENSY_PID = 0x0483
BAUD_RATE = 115200

def getPort(vid, pid) -> str:
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == vid and device.pid == pid:
            return device.device
    raise OSError('Device not found')


class MCUPublisher(Node):
    
    def __init__(self):
        super().__init__("mcu_publisher")
        
        self.create_timer(0.05, self.update_mcu)

        mcu_serial_port = getPort(TEENSY_VID, TEENSY_PID)
        self.mcu_serial = serial.Serial(mcu_serial_port, BAUD_RATE)
            
        
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.sail_angle_publisher = self.create_subscription(msg_type=Float32, topic="/actions/sail_angle", callback=self.sail_angle_callback, qos_profile=sensor_qos_profile)
        self.rudder_angle_publisher = self.create_subscription(msg_type=Float32, topic="/actions/rudder_angle", callback=self.rudder_angle_callback, qos_profile=sensor_qos_profile)
        
        self.sail_angle = 0.
        self.rudder_angle = 0.


    def sail_angle_callback(self, sail_angle: Float32):
        self.sail_angle = sail_angle.data
        
    def rudder_angle_callback(self, rudder_angle: Float32):
        self.rudder_angle = rudder_angle.data

    def update_mcu(self):
        self.mcu_serial.write(f'sail angle: {self.sail_angle}; rudder angle: {self.rudder_angle}\n'.encode('ascii'))
        print(f"Received: {self.mcu_serial.read_all().decode('ascii')}")
            
        
        
def main():
    rclpy.init()
    mcu_publisher = MCUPublisher()
    rclpy.spin(mcu_publisher)

if __name__ == "__main__": main()