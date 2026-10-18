# ruff: noqa:I001 E402, F401
import gi

gi.require_version('Gst', '1.0')
from gi.repository import GLib, Gst

import json
import os
import re
import shutil
import subprocess
import sys
import time
import threading

import cv2
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

IS_DEV_CONTAINER = re.search("/home/ws", os.getcwd()) is not None
PATH_TO_PKG_DIR = "/home/ws/ros_packages" if IS_DEV_CONTAINER else f"{os.path.expanduser('~')}/autoboat_vt/ros_packages"
CAMERA_CONFIG = f"{PATH_TO_PKG_DIR}/object_detection/object_detection/config/camera_config.yaml"

class CamCorderNode(Node):
    def __init__(self) -> None:
        self.storage_cap = 0.6 # Do not go above this disk utilization
        self.last_time = time.time()

        if self._get_storage_util() > self.storage_cap:
            raise OSError(f"Current disk usage is above {self.storage_cap * 100:.0f}%. Exitting")
        
        os.makedirs("./frame_logs/", exist_ok=True)
        count = 0
        while os.path.exists(f"./frame_logs/run{count}"):
            count += 1
        self.log_file = f"./frame_logs/run{count}/frame_logs.jsonl"
        self.run_dir = f"./frame_logs/run{count}/frames/"
        os.makedirs(self.run_dir, exist_ok=True)

        self.cam_list = self._read_camera_config()
        self.cam_list[0]["device"] = self._find_camera()

        # Gstreamer Initialization
        Gst.init(None)
        self.pipeline = None
        self.loop = None
        self.filesink = None

        """
        formatting for self.log_file
        Each line is a JSON object representing a frame
        {
            head: <current_heading>,
            lat: <current_lat>,
            long: <current_lon>,
            time: <current_time>
        }
        """

        self.pos_lock = threading.Lock()
        self.position = {
            "lon": 0,
            "lat": 0,
            "head": 0
        }
        
        self.create_subscription(
            msg_type=NavSatFix, topic="/position", callback=self._position_callback, qos_profile=qos_profile_sensor_data
        )
        self.create_subscription( # heading is counterclockwise of true east
            msg_type=Float32, topic="/heading", callback=self._heading_callback, qos_profile=qos_profile_sensor_data
        )

        self._init_pipeline()
        cv_thread = threading.Thread(target=self.run, daemon=True)
        cv_thread.start()
    
    def _init_pipeline(self):
        self.pipeline = Gst.Pipeline()
        
        source0 = Gst.ElementFactory.make('v4l2src', 'usb-cam-0')
        source0.set_property('device', self.cam_list[0]["device"])
        self.get_logger().info(f"Opening camera device {self.cam_list[0]['name']} on {self.cam_list[0]['device']}")

        caps_source0 = Gst.ElementFactory.make('capsfilter', 'source0-caps')
        caps_source0.set_property('caps', Gst.Caps.from_string(f'video/x-raw,'
                                                               f'width={self.cam_list[0]["width"]},'
                                                               f'height={self.cam_list[0]["height"]},'
                                                               f'format={self.cam_list[0]["gst_format"]},'
                                                               f'framerate={self.cam_list[0]["framerate_n"]}/'
                                                               f'{self.cam_list[0]["framerate_d"]}'
                                                               ))

        pngenc = Gst.ElementFactory.make('pngenc', 'pngenc')

        self.filesink = Gst.ElementFactory.make('multifilesink', 'filesink')
        self.filesink.set_property('location', f'{self.run_dir}%06d.png')

        self.pipeline.add(source0)
        self.pipeline.add(caps_source0)
        self.pipeline.add(pngenc)
        self.pipeline.add(self.filesink)

        source0.link(caps_source0)
        caps_source0.link(pngenc)
        pngenc.link(self.filesink)

        self.loop = GLib.MainLoop()
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._bus_call, self.loop)

        sink_probe_pad = filesink.get_static_pad('sink')
        sink_probe_pad.add_probe(Gst.PadProbeType.BUFFER, self._sink_probe, 0)

    def _bus_call(self, bus: Gst.Bus, message: Gst.Message, loop: GLib.MainLoop) -> bool: # noqa: ARG002
        t = message.type
        if t == Gst.MessageType.EOS:
            self.get_logger().info("End-of-stream\n")
            self.close_pipeline()
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            self.get_logger().warn(f"Warning: {err}: {debug}\n")
        elif t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            self.get_logger().error(f"Error: {err}: {debug}\n")
            self.close_pipeline()
        return True
    
    def run(self) -> None:
        """Turn on the pipeline and start the main loop. This will block until the pipeline is closed."""
        self.get_logger().info("Starting pipeline\n")
        self.pipeline.set_state(Gst.State.PLAYING)
        try:
            self.loop.run()
        except Exception as e:
            self.get_logger().error(f"Exception in main loop: {e}\n")
        finally:
            self.pipeline.set_state(Gst.State.NULL)
            self.get_logger().info("Pipeline stopped\n")
    
    def close_pipeline(self) -> None:
        """Safely closes the GStreamer pipeline and stops the main loop."""
        self.get_logger().info("Quitting pipeline\n")
        self.pipeline.set_state(Gst.State.NULL)
        self.loop.quit()
        self.get_logger().info("Pipeline stopped\n")
    
    def _sink_probe(self, pad: Gst.Pad, info: Gst.PadProbeInfo, user_data: int) -> Gst.PadProbeReturn: # noqa: ARG002
        count = self.filesink.get_property('index')
        if count % 120 == 0:
            current_time = time.time()
            fps = 120 / (current_time - self.last_time)
            self.last_time = current_time
            self.get_logger().info(f"Current frame count: {count}, FPS: {fps:.2f}")
            if self._get_storage_util() > self.storage_cap:
                self.get_logger().warn(f"Passed {(self.storage_cap * 100):.0f}% disk usage. Exiting")
                self.close_pipeline()
                raise OSError(f"Passed {(self.storage_cap * 100):.0f}% disk usage. Exiting")
        
        with self.pos_lock:
            curr_log = {
                "lat": self.position["lat"],
                "lon": self.position["lon"],
                "head": self.position["head"],
                "time": time.time()
            }
        with open(self.log_file, 'a') as file:
            file.write(json.dumps(curr_log) + '\n')

    def _read_camera_config(self) -> dict:
        with open(CAMERA_CONFIG, 'r') as file:
            return yaml.safe_load(file)
    
    def _find_camera(self, cam_id: int = 0) -> str:
        """
        This is just a way to figure out which /dev/video* is the camera<br>
        The camera outputs on 3 devices<br>
        Each device is a different format, but the order can change or extra cameras can cause the number to increase<br>
        While this finds the device with the specified format,
        it does not guarantee that the correct resolution and framerate are available.
        
        Returns
        -------
            str: The /dev/video* device path.
        """

        cam_format = self.cam_list[cam_id]["v4l2_format"]
        cam_name = self.cam_list[cam_id]["name"]
        ls = shutil.which("ls")
        cat = shutil.which("cat")
        v4l2_ctl = shutil.which("v4l2-ctl")
        if ls is None or cat is None or v4l2_ctl is None:
            self.get_logger().error("ls, cat, or v4l2-ctl command not found. Cannot find camera device.")
            raise OSError("Required command not found")
        try:
            camera_devices_output = subprocess.run([ls, '/sys/class/video4linux/'],
                                                   capture_output=True, text=True, check=True).stdout
        except subprocess.CalledProcessError as err:
            self.get_logger().error("Failed to list camera devices in /sys/class/video4linux/.")
            raise OSError("Failed to list camera devices in /sys/class/video4linux/.") from err
        for device in camera_devices_output.splitlines():
            try:
                if ((re.search(cam_name, subprocess.run([cat, f'/sys/class/video4linux/{device}/name'],
                                                        capture_output=True, text=True, check=True).stdout) is not None) and
                (re.search(cam_format, subprocess.run([v4l2_ctl, '--device', f'/dev/{device}', '--list-formats'],
                                                            capture_output=True, text=True, check=True).stdout) is not None)):
                        return f"/dev/{device}"
            except subprocess.CalledProcessError as err:
                self.get_logger().error(f"Command v4l2-ctl failed for device {device}.")
                raise OSError(f"Command v4l2-ctl failed for device {device}.") from err
        self.get_logger().error(f"Could not find {cam_name} device with {cam_format} format")
        raise OSError("Camera device not found")
    
    def _position_callback(self, msg: NavSatFix) -> None:
        with self.pos_lock:
            self.position["lat"] = msg.latitude
            self.position["lon"] = msg.longitude
    
    def _heading_callback(self, msg: Float32) -> None:
        with self.pos_lock:
            self.position["head"] = msg.data

    def _get_storage_util(self) -> float:
        usage = shutil.disk_usage('/')
        total = usage.total
        used = usage.used
        return used / total

def main() -> None:
    rclpy.init()
    cam_corder_node = CamCorderNode()
    try:
        rclpy.spin(cam_corder_node)
    finally:
        cam_corder_node.destroy_node()
        rclpy.shutdown()
