import aiohttp
import asyncio
import pysmartthings
import threading
import time
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class SmartthingsPublisher(Node):

    def __init__(self, smartthings_response, update_period):
        super().__init__('smartthings_publisher')
        self.publisher_motion_door = self.create_publisher(Bool, 'smartthings_sensors_motion_door', 10)
        self.publisher_motion_pills = self.create_publisher(Bool, 'smartthings_sensors_motion_pills', 10)
        self.publisher_sensor_door = self.create_publisher(Bool, 'smartthings_sensors_door', 10)
        #need new sensor for food
        self.publisher_motion_eat = self.create_publisher(Bool, 'smartthings_sensors_eat', 10)

        self.timer = self.create_timer(update_period, self.timer_callback)
        self.smartthings_response = smartthings_response

    def timer_callback(self):
        if self.smartthings_response.updated:
            msg = Bool()
            msg.data = not self.smartthings_response.closed
            self.publisher_sensor_door.publish(msg)
            msg = Bool()
            msg.data = not self.smartthings_response.active_2
            self.publisher_motion_door.publish(msg)
            msg = Bool()
            msg.data = not self.smartthings_response.active_1
            self.publisher_motion_pills.publish(msg)



class SmartthingsResponse:
    def __init__(self, update_period):
        self.updated = False
        self.closed = None
        self.active_1 = None
        self.active_2 = None
        self.update_period = update_period

    async def print_devices(self):
        # link to get tokens: https://developer.smartthings.com/docs/advanced/authorization-and-permissions#personal-access-tokens
        token = os.getenv("SMARTTHINGS_TOKEN")
        # device_names = ['multi-sensor']
        async with aiohttp.ClientSession() as session:
            api = pysmartthings.SmartThings(session, token)
            devices = await api.devices()
            # devices_ = [motion-temp-battery, multi-sensor, motion-temp-battery]
            devices_id = ['c5472f1f-c05f-4d16-bb11-dc645668568a', '70434ef2-10c7-425e-93bc-0495197817d7', 'b6b02811-f5d5-4c36-abb4-47d9206ffbb5']
            # order = [motion_Sensor_1, door sensor, motion sensor 2]
            while True:
                start = float(time.time_ns() // 1_000_000_000)
                for device in devices:
                    if device.device_id == devices_id[0]: #sensor_1
                        await device.status.refresh()
                        print('device id',device.name, device.status.values['motion'])
                        self.active_1 = device.status.values['motion'] =='inactive'
                    elif device.device_id == devices_id[1]: # door sensor
                        print('name',device.name)
                        await device.status.refresh()
                        self.closed = device.status.values['contact'] == 'closed'           
                    elif device.device_id == devices_id[2]: #sensor_2
                        await device.status.refresh()
                        print('name',device.name,device.status.values['motion'])
                        self.active_2 = device.status.values['motion'] =='inactive'
                end = float(time.time_ns() // 1_000_000_000)
                await asyncio.sleep(self.update_period - (end - start))
                self.updated = True


def main(args=None):
    update_period = 1.5  # 1 sec
    smartthings_response = SmartthingsResponse(update_period)
    x = threading.Thread(target=asyncio.run, args=(smartthings_response.print_devices(),))
    x.start()

    rclpy.init(args=args)

    minimal_publisher = SmartthingsPublisher(smartthings_response, update_period)

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
