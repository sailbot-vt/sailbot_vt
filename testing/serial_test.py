import serial
from serial.tools import list_ports

def getPort(vid, pid) -> str:
    device_list = list_ports.comports()
    for device in device_list:
        print(f"pid:{device.pid}, vid: {device.vid}")
        if device.vid == vid and device.pid == pid:
            return device.device
    raise OSError('Device not found')

VID = 0x8086
PID = 0x0b5c 

serial_port = getPort(VID, PID)
sensor_serial = serial.Serial(serial_port, 9600)

print(sensor_serial.read_all())