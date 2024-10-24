import aiohttp
import asyncio
import threading
import time
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int32
import requests

class HomeSeerPublisher(Node):

    def __init__(self):
        super().__init__('smartthings_publisher')
        # self.publisher_motion_door = self.create_publisher(Bool, 'smartthings_sensors_motion_door', 10)
        self.publisher_pills_motion_sensor = self.create_publisher(Int32, 'person_taking_medicine', 10)
        # self.publisher_dining_motion_sensor = self.create_publisher(Int32, 'person_eating', 10)

        self.publisher_bedroom_sensor_door = self.create_publisher(Bool, 'smartthings_sensors_door_bedroom', 10)
        # self.publisher_living_sensor_door = self.create_publisher(Bool, 'smartthings_living_sensor_door', 10)
        # self.publisher_bathroom_sensor_door = self.create_publisher(Bool, 'smartthings_sensors_door_bathroom', 10)
        # self.publisher_main_sensor_door = self.create_publisher(Bool, 'smartthings_sensors_door_outdoor', 10)
        update_period = 5 # sec
        self.timer = self.create_timer(update_period, self.timer_callback)
        self.url = 'http://192.168.50.77/json?request=getstatus&ref='

    def timer_callback(self):
        # Make the request to get the JSON data just add ref number
        msg_bool = Bool()
        msg_int = Int32()

        bedroom_door_sensor_response = requests.get(self.url+"316")
        # Parse the JSON content
        data = bedroom_door_sensor_response.json()
        # Find the value associated with the "ref"
        devices = data.get("Devices", [])
        for device in devices:
            if device.get("ref") == 316:
                value = device.get("value")
                if value ==22: # door is open
                    msg_bool.data = True
                else:
                    msg_bool.data = False
                self.publisher_bedroom_sensor_door.publish(msg_bool)

                print(f'The value is: {value}')


        motion_sensor_response = requests.get(self.url+"320")
        # Parse the JSON content
        data = motion_sensor_response.json()
        # Find the value associated with the "ref"
        devices = data.get("Devices", [])
        for device in devices:
            if device.get("ref") == 320:
                value = device.get("value")
                if value ==7: # motion detected
                    msg_int.data = 1
                else:
                    msg_int.data = 0
                self.publisher_pills_motion_sensor.publish(msg_int)

                print(f'The value is: {value}')


def main(args=None):

    rclpy.init(args=args)

    minimal_publisher = HomeSeerPublisher()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
