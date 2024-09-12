#!/usr/bin/python3

# This all uses the following library (Thank you very much for actually making a parser for this insufferable protocol with no documentation)
# https://github.com/AlessioMorale/crsf_parser/tree/master 

from sys import argv
argv = argv[1:]


import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import serial
from serial.tools import list_ports
from std_msgs.msg import Bool
from sailbot_msgs.msg import RCData
from rclpy.node import Node

from crsf_parser.payloads import PacketsTypes
from crsf_parser.frames import crsf_frame
from crsf_parser import CRSFParser, PacketValidationStatus


RC_VID = 0x0403
RC_PID = 0x6001
RC_SERIAL_NUMBER = "A9001WL3"
BAUD_RATE = 420000

def getPort(vid, pid, serial_number) -> str:
    device_list = list_ports.comports()
    for device in device_list:
        print(device.serial_number)
        if device.vid == vid and device.pid == pid and device.serial_number == serial_number:
            return device.device
    raise OSError('Device not found')

class RCPublisher(Node):
    
    def __init__(self):
        super().__init__("rc_publisher")
        
        self.create_timer(0.025, self.timer_callback)
        
        serial_port = getPort(RC_VID, RC_PID, RC_SERIAL_NUMBER)
        self.sensor_serial = serial.Serial(serial_port, BAUD_RATE)

        self.crsf_parser = CRSFParser(self.save_frame)
        self.crsf_frame_rc_data = None
        self.serial_stream = bytearray()
        self.callback_count = 0
        
        sensor_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.rc_data_publisher = self.create_publisher(RCData, '/rc_data', sensor_qos_profile)
        self.termination_listener = self.create_subscription(msg_type=Bool, topic="/should_terminate", callback=self.should_terminate_callback, qos_profile=10)


    def save_frame(self, frame: crsf_frame, status: PacketValidationStatus) -> crsf_frame:
        """
        this frame will come back as a crsf frame with the following declaration:
        
        crsf_frame = Struct(
        "header" / crsf_header,
        "payload"
        / Switch(
            this.header.type,
            {
                PacketsTypes.HEARTBEAT: payload_heartbeat,
                PacketsTypes.BATTERY_SENSOR: payload_battery_sensor,
                PacketsTypes.LINK_STATISTICS: payload_link_statistics,
                PacketsTypes.RC_CHANNELS_PACKED: payload_rc_channels_packed,
            },
            default=Array(this.frame_length - 2, Byte),
          ),
            "crc_offset" / Tell,
            "CRC" / Int8ub,
        )
        
        The only packets that we care about are the PacketsTypes.RC_CHANNELS_PACKED
        payload_rc_channels_packed objects are created with the following declaration:
        
        payload_rc_channels_packed= ByteSwapped(
            BitStruct("channels" / Array(16, BitsInteger(11)))
        )
        
        See this page for more information about the BitStruct: https://construct.readthedocs.io/en/latest/bitwise.html 
        """
        
        # We need to make sure the only packets we save are information about the rc inputs just to be safe
        if frame.header.type != PacketsTypes.RC_CHANNELS_PACKED or status.value != 1: return
        
        self.crsf_frame_rc_data = frame
        
        
        

    def normalize_joystick_input(self, input, cur_min, cur_max):
        """
        Normalizes a number between cur_min and cur_max to between -100 and 100.
        https://stackoverflow.com/questions/929103/convert-a-number-range-to-another-range-maintaining-ratio 
        """
        return (((input - cur_min) * 200) / (cur_max - cur_min)) - 100

    def parse_toggle(self, toggle_state, state1, state2, state3):
        if toggle_state == state1: return 0
        elif toggle_state == state2: return 1
        elif toggle_state == state3: return 2
        else: 
            print(f"WARNING: Toggle state was not properly accounted for: {toggle_state}")
            return -1
    
    def process_raw_channels(self, raw_channel_array: list) -> RCData:
        """
        Processes the raw channel outputs from the receiver and returns the ros2 msg to publish
        Toggle states:
            State 1: Fully Up
            State 2: Middle
            State 3: Fully Down

        THROTTLE Up/Down: 13 (175-1811)
        THROTTLE Left/Right: 12 (174-1793)
        STEERING Up: 7 (191-997)
        STEERING Left/Right: 15 (193-1811)
        A Button: Makes ch7 always greater than 997 maxing out at 1792 and transforms it to steering Down. Don't use this
        B Toggle: 10 (State 1: 191, State 2: 191, State 3: 997)
        C Toggle: 8 (State 1: 191, State 2: 997, State 3: 1792)
        D Button: 6 (191/ 1792)
        E Toggle: 11 (State 1: 191, State 2: 191, State 3: 1792)
        F Toggle: 10 + 14 (State 1: 174, State 2: 992, State 3: 1811). This interferes with B Switch so don't use B switch, only use the F Switch
        Left Potentiometer: 5 (191-1792)
        Right Potentiometer: 4 (191-1792)
        """
        
        # tranform from [175, 1811] to [-100, 100]
        joystick_left_y = self.normalize_joystick_input(raw_channel_array[15], 174, 1811)
        joystick_left_x = self.normalize_joystick_input(raw_channel_array[14], 174, 1811)
        
        joystick_right_y = self.normalize_joystick_input(raw_channel_array[13], 191, 1792)
        joystick_right_x = self.normalize_joystick_input(raw_channel_array[12], 174, 1811)
    
        button_a = False
        toggle_b = self.parse_toggle(raw_channel_array[10], 191, 997, 1792)
        toggle_c = self.parse_toggle(raw_channel_array[9], 191, 997, 1792)
        button_d = False
        toggle_e = self.parse_toggle(raw_channel_array[11], 191, 997, 1792)
        toggle_f = self.parse_toggle(raw_channel_array[8], 191, 997, 1792)
        
        if toggle_b == -1 or toggle_c == -1 or toggle_e == -1 or toggle_f == -1: return None
        
        return RCData(
            joystick_left_x=joystick_left_x, joystick_left_y=joystick_left_y,
            joystick_right_x=joystick_right_x, joystick_right_y=joystick_right_y,
            button_a=button_a, toggle_b=toggle_b, toggle_c=toggle_c, 
            button_d=button_d, toggle_e=toggle_e, toggle_f=toggle_f
        )
        
        
        
    def timer_callback(self):
        num_unread_bytes = self.sensor_serial.in_waiting
        values = self.sensor_serial.read(num_unread_bytes)
    
        self.serial_stream.extend(values)

        # parse the data into self.crsf_frame
        self.crsf_parser.parse_stream(self.serial_stream)

        if not self.crsf_frame_rc_data: return 

        rc_data = self.process_raw_channels(self.crsf_frame_rc_data.payload.channels)
        print(rc_data)
        print()
        
        if rc_data == None: return
        
        self.rc_data_publisher.publish(rc_data)


    def should_terminate_callback(self, msg: Bool):
        if msg.data == False: return
        rclpy.shutdown()
        
        
    def __del__(self):
        self.get_logger().info("closing rc publisher")
        self.sensor_serial.close()
        rclpy.shutdown()    


def main(args=None):
    rclpy.init(args=args)

    rc_publisher = RCPublisher()

    rclpy.spin(rc_publisher)


if __name__ == "__main__":
    main()
