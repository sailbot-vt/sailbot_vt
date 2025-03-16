import rclpy
import pyvesc
from pyvesc import VESC
from pyvesc.VESC.messages import GetValues, SetRPM, SetCurrent, SetRotorPositionMode, GetRotorPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import serial
import time
import csv
from serial.tools import list_ports
import os, signal

from rclpy.node import Node

from std_msgs.msg import String, Float32

from sailbot_msgs.msg import VESCData, VESCControlData

motorPolePairs = 7

class VESCPublisher(Node):

    def __init__(self):
        super().__init__('pyvesc_publisher')
        self.ser = getPort( 0x0483, 0x5740)
        # self.get_logger().info(f"{self.ser}")
        try:
            self.motor = VESC(serial_port= self.ser)
        except:
            self.get_logger().error("failed to connect to the motor")
            self.destroy_node()
            rclpy.shutdown()
            os.kill(os.getpid(), signal.SIGTERM)

        self.motorVal = 0
        self.motorType = 0 # 1-duty cycle 2-rpm 3-current 
        self.missed_measurements_in_a_row = 0
        self.last_command_time = 0
        
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.vesc_data_publisher = self.create_publisher(VESCData, "/vesc_data", sensor_qos_profile)

        
        
        """
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
        """
        
        self.controlTypeSub = self.create_subscription(msg_type= VESCControlData, topic='/motor_control_struct', callback=self.receive_control_data_callback, qos_profile=sensor_qos_profile)
        """
        self.controlTypeSub = self.create_subscription(String, "vesc/control_type", self.ct_callback, 10)
        self.current_valueSub = self.create_subscription(Float32, "vesc/current_value", self.cv_callback, 10)
        self.rpm_valueSub = self.create_subscription(Float32, "vesc/rpm_value", self.rpmv_callback, 10)
        self.dutycycle_valueSub = self.create_subscription(Float32, "vesc/duty_cycle_value", self.dcv_callback, 10)
        """

        # (this doesn't work with systemctl automatic startup on boot)
        # self.csv_writer = csv.DictWriter(open("VescInfo.csv", 'w+'), fieldnames= ["time", "rpm", "duty_cycle", "v_in", "c_in", "c_motor", "temp_motor", "time_ms", "amp_hours", "amp_hours_charged","motor_wattage", "v_out"])
        # self.csv_writer.writeheader()

        timer_period = 0.05  # seconds
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    
    def receive_control_data_callback(self, msg: VESCControlData):
        self.last_command_time = time.time()
        
        self.get_logger().info(f'{self.motorVal}')
        
        try:
            if(msg.control_type_for_vesc == "rpm"):
                self.motorVal = msg.desired_vesc_rpm * motorPolePairs
                self.motor.set_rpm(int(self.motorVal))
            elif(msg.control_type_for_vesc == "duty_cycle"):
                self.motorVal = msg.desired_vesc_duty_cycle
                self.motor.set_duty_cycle(int(self.motorVal))
            else:
                self.motorVal = msg.desired_vesc_current
                self.motor.set_current(int(self.motorVal))
        except:
            self.get_logger().error("Disconnected from the VESC")
            self.destroy_node()
            rclpy.shutdown()
            os.kill(os.getpid(), signal.SIGTERM)
            
    """
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
    """

    def timer_callback(self):
        
        if (time.time() - self.last_command_time >= 3):
            self.motor.set_rpm(0)
        
        #get data and store in dictionary
        try:
            measurements = self.motor.get_measurements()
        except:
            self.get_logger().error("Disconnected from the VESC")
            self.destroy_node()
            rclpy.shutdown()
            os.kill(os.getpid(), signal.SIGTERM)
        if not measurements:
            self.missed_measurements_in_a_row += 1
            if (self.missed_measurements_in_a_row >= 20):
                self.get_logger().error("Disconnected from the VESC")
                self.destroy_node()
                rclpy.shutdown()
                os.kill(os.getpid(), signal.SIGTERM)
            
            return
        
        else:
            self.missed_measurements_in_a_row = 0
        
        rpm = measurements.rpm/motorPolePairs
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

        #write vesc data to csv file (this doesn't work with systemctl automatic startup on boot)
        # self.csv_writer.writerow(motorData)
        
        #publish vesc data to topic
        self.vesc_data_publisher.publish(
            VESCData(
                rpm= motorData["rpm"], duty_cycle= motorData["duty_cycle"], 
                voltage_to_vesc= motorData["v_in"], current_to_vesc= motorData["c_in"],
                voltage_to_motor = motorData["v_out"], avg_current_to_motor = motorData["c_motor"],
                wattage_to_motor = motorData["motor_wattage"], motor_temperature = motorData["temp_motor"],
                time_since_vesc_startup_in_ms= motorData["time_ms"], amp_hours = motorData["amp_hours"], 
                amp_hours_charged = motorData["amp_hours_charged"]  
            )
        )
    

    def __del__(self):
        if hasattr(self, "motor"):
            self.motor.stop_heartbeat()


def main(args=None):
    rclpy.init(args=args)
    vesc_publisher = VESCPublisher()
    
    rclpy.spin(vesc_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vesc_publisher.destroy_node()
    
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


#Pragya - Yes, you can play tetris on the phallic ice cream