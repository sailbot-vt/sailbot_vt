import rclpy
import pyvesc
from pyvesc import VESC
from pyvesc.VESC.messages import GetValues, SetRPM, SetCurrent, SetRotorPositionMode, GetRotorPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import serial
import time
import csv
from serial.tools import list_ports

from rclpy.node import Node

from std_msgs.msg import String, Float32

from sailbot_msgs.msg import VESCData, VESCControlData

motorPolePairs = 7

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('pyvesc_publisher')
        self.ser = getPort( 0x0483, 0x5740)
        self.motor = VESC(serial_port= self.ser)
        self.motorVal = 0
        self.motorType = 0 # 1-duty cycle 2-rpm 3-current 

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

        self.csv_writer = csv.DictWriter(open("VescInfo.csv", 'w+'), fieldnames= ["time", "rpm", "duty_cycle", "v_in", "c_in", "c_motor", "temp_motor", "time_ms", "amp_hours", "amp_hours_charged","motor_wattage", "v_out"])
        self.csv_writer.writeheader()

        timer_period = 0.05  # seconds
        
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def receive_control_data_callback(self, msg: VESCControlData):
        self.get_logger().info(f'{self.motorVal}')
        if(msg.control_type_for_vesc == "rpm"):
            self.motorVal = msg.desired_vesc_rpm * motorPolePairs
            self.motor.set_rpm(int(self.motorVal))
        elif(msg.control_type_for_vesc == "duty_cycle"):
            self.motorVal = msg.desired_vesc_duty_cycle
            self.motor.set_duty_cycle(int(self.motorVal))
        else:
            self.motorVal = msg.desired_vesc_current
            self.motor.set_current(int(self.motorVal))

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
        
        #get data and store in dictionary
        measurements = self.motor.get_measurements()
        if(measurements):
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

            #write vesc data to csv file
            #self.get_logger().info(f'{motorData}')
            self.csv_writer.writerow(motorData)
            
            #publish vesc data to topic
            self.vesc_data_publisher.publish(VESCData(rpm= motorData["rpm"], duty_cycle= motorData["duty_cycle"], 
                                                      voltage_to_vesc= motorData["v_in"], current_to_vesc= motorData["c_in"],
                                                      voltage_to_motor = motorData["v_out"], avg_current_to_motor = motorData["c_motor"],
                                                      wattage_to_motor = motorData["motor_wattage"], 
                                                      motor_temperature = motorData["temp_motor"],
                                                      time_since_vesc_startup_in_ms= motorData["time_ms"],
                                                      amp_hours = motorData["amp_hours"], amp_hours_charged = motorData["amp_hours_charged"]  
                                                      ))
            """
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
            """
    

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


#Pragya - Yes, you can play tetris on the phallic ice cream