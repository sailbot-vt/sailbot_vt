import rclpy
import pyvesc
from pyvesc import VESC
from pyvesc.VESC.messages import GetValues, SetRPM, SetCurrent, SetRotorPositionMode, GetRotorPosition
import serial
import time
import csv
from serial.tools import list_ports

from rclpy.node import Node

from std_msgs.msg import String, Float32

motorPolePairs = 7

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('pyvesc_publisher')
        self.ser = getPort( 0x0483, 0x5740)
        self.motor = VESC(serial_port= self.ser)
        self.motorVal = 0
        self.motorType = 0 # 1-duty cycle 2-rpm 3-current 
        
        self.rpmPub = self.create_publisher(Float32, 'vesc/rpm_data', 10)
        self.vInPub = self.create_publisher(Float32, "vesc/v_in", 10)
        self.vOutPub = self.create_publisher(Float32, "vesc/v_out", 10)
        self.motorCurrentPub = self.create_publisher(Float32, "vesc/current_to_motor", 10)
        self.motorWattagePub = self.create_publisher(Float32, "vesc/wattage_to_motor", 10)
        self.dutyCyclePub = self.create_publisher(Float32, "vesc/duty_cycle", 10)
        self.timeMsPub = self.create_publisher(Float32, "vesc/time_ms", 10)
        self.amp_hoursPub = self.create_publisher(Float32, "vesc/amp_hours", 10)
        self.amp_hoursChargedPub = self.create_publisher(Float32, "vesc/amp_hours_charged", 10)
        self.temp_motorPub = self.create_publisher(Float32, "vesc/temp_motor", 10)
        self.c_inPub = self.create_publisher(Float32, "vesc/c_in", 10)

        self.controlTypeSub = self.create_subscription(String, "vesc/control_type", self.ct_callback, 10)
        self.current_valueSub = self.create_subscription(Float32, "vesc/current_value", self.cv_callback, 10)
        self.rpm_valueSub = self.create_subscription(Float32, "vesc/rpm_value", self.rpmv_callback, 10)
        self.dutycycle_valueSub = self.create_subscription(Float32, "vesc/current_value", self.dcv_callback, 10)

        self.csv_writer = csv.DictWriter(open("VescInfo.csv", 'w+'), fieldnames= ["time", "rpm", "duty_cycle", "v_in", "c_in", "c_motor", "temp_motor", "time_ms", "amp_hours", "amp_hours_charged","motor_wattage", "v_out"])
        self.csv_writer.writeheader()

        timer_period = 0.05  # seconds
        
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def ct_callback(self, msg):
        if msg == "DUTY_CYCLE":
            self.motorType = 1
        if msg == "RPM":
            self.motorType = 2
        if msg == "CURRENT":
            self.motorType = 3

    def cv_callback(self, msg):
        if self.motorType == 3:
            self.motorVal = msg

    def rpmv_callback(self, msg):
        if self.motorType == 2:
            self.motorVal = msg

    def dcv_callback(self, msg):
        if self.motorType == 1:
            self.motorVal = msg

    def timer_callback(self):
        #get data and store in var
        measurements = self.motor.get_measurements()
        if(measurements):
            rpm = measurements.rpm/motorPolePairs
            duty_cycle = measurements.duty_cycle_now
            v_in = measurements.v_in
            c_in = measurements.avg_input_current
            c_motor = measurements.avg_motor_current
            motorData = {
                "time": time.time(),
                "rpm": measurements.rpm/motorPolePairs,
                "duty_cycle": measurements.duty_cycle_now,
                "v_in": measurements.v_in,
                "c_in": measurements.avg_input_current,
                "c_motor": measurements.avg_motor_current,
                "temp_motor": measurements.temp_motor, 
                "time_ms": measurements.time_ms,
                "amp_hours": measurements.amp_hours, 
                "amp_hours_charged": measurements.amp_hours_charged,
                "motor_wattage": c_motor*rpm/180,
                "v_out": rpm/180
            }

            self.get_logger().info(f'{motorData}')
            self.csv_writer.writerow(motorData)
            #publish various data
            self.rpmPub.publish(Float32(data = rpm))
            self.vInPub.publish(Float32(data = v_in))
            self.motorCurrentPub.publish(Float32(data = c_motor))
            self.dutyCyclePub.publish(Float32(data = duty_cycle))
            self.vOutPub.publish(Float32(data = rpm/180))
            self.motorWattagePub.publish(Float32(data = c_motor*rpm/180))  
            self.temp_motorPub.publish(Float32(data = measurements.temp_motor))
            self.timeMsPub.publish(Float32(data = measurements.time_ms)) 
            self.c_inPub.publish(Float32(data = c_in))
            self.amp_hoursChargedPub.publish(Float32(data = measurements.amp_hours_charged))
            self.amp_hoursPub.publish(Float32(data = measurements.amp_hours))
    

    def __del__(self):
        self.motor.stop_heartbeat()


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    
    rclpy.shutdown()

def getPort(vid, pid) -> str:
    device_list = list_ports.comports()
    for device in device_list:
        print(device.serial_number)
        if device.vid == vid and device.pid == pid:
            return device.device
    raise OSError('Device not found')


if __name__ == '__main__':
    main()

"""
TO DO (one node)

Subscriber that is able to take data abotu which control we want and value of that control--DUTY CYCLE RPM CURRENT 

Type of control is a topic

current_value - topic
duty_value - topic
rpm_value - topic

Output/publish
rpm data
voltage in (battery voltage)
voltage out (motor voltage) rpm = 180 * V
current to motor 
total wattage to motor I*V
duty cycle
time_ms
amp_hours
amp_hours_charged
temp_motor

"""