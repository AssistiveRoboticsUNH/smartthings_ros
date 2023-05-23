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
        self.publisher_motion_bedroom = self.create_publisher(Bool, 'smartthings_sensors_motion_bed_side', 10)
        self.publisher_motion_eat = self.create_publisher(Bool, 'smartthings_sensors_motion_eat', 10)

        self.publisher_sensor_door = self.create_publisher(Bool, 'smartthings_sensors_door', 10)

        self.timer = self.create_timer(update_period, self.timer_callback)
        self.smartthings_response = smartthings_response

    def timer_callback(self):
        if self.smartthings_response.updated:
            msg = Bool()
            msg.data = not self.smartthings_response.door_sensor_closed
            self.publisher_sensor_door.publish(msg)
            msg = Bool()
            msg.data = not self.smartthings_response.door_motion_sensor_active
            self.publisher_motion_door.publish(msg)
            msg = Bool()
            msg.data = not self.smartthings_response.pill_motion_sensor_active
            self.publisher_motion_pills.publish(msg)
            msg = Bool()
            msg.data = not self.smartthings_response.dining_motion_sensor_active
            self.publisher_motion_eat.publish(msg)
            msg = Bool()
            msg.data = not self.smartthings_response.bedroom_motion_sensor_active
            self.publisher_motion_bedroom.publish(msg)


class SmartthingsResponse:
    def __init__(self, update_period):
        self.updated = False
        self.door_sensor_closed = None
        self.door_motion_sensor_active = None
        self.pill_motion_sensor_active = None
        self.dining_motion_sensor_active = None
        self.bedroom_motion_sensor_active = None
        self.update_period = update_period

    async def print_devices(self):
        # link to get tokens: https://developer.smartthings.com/docs/advanced/authorization-and-permissions#personal-access-tokens
        token = os.getenv("SMARTTHINGS_TOKEN")
        # device_names = ['multi-sensor']
        async with aiohttp.ClientSession() as session:
            api = pysmartthings.SmartThings(session, token)
            devices = await api.devices()
            # devices_ = [motion-temp-battery, multi-sensor, motion-temp-battery]
            # devices_id = ['c5472f1f-c05f-4d16-bb11-dc645668568a', '70434ef2-10c7-425e-93bc-0495197817d7',
            #               'b6b02811-f5d5-4c36-abb4-47d9206ffbb5']
            # order = [motion_Sensor_1, door sensor, motion sensor 2]
            while True:
                start = float(time.time_ns() // 1_000_000_000)
                for device in devices:
                    print(device.label)

                    if device.label == 'door_motion_sensor':  # sensor_1
                        print('1')
                        await device.status.refresh()
                        print('device id', device.name, device.status.values['motion'])
                        self.door_motion_sensor_active = device.status.values['motion'] == 'inactive'
                    elif device.label == 'door_sensor':  # door sensor
                        print('2')
                        await device.status.refresh()
                        self.door_sensor_closed = device.status.values['contact'] == 'closed'
                    elif device.label == 'dining_motion_sensor':  # sensor_2
                        await device.status.refresh()
                        print('3')
                        self.dining_motion_sensor_active = device.status.values['motion'] == 'inactive'
                    elif device.label == 'dining_motion_sensor':  # sensor_2
                        await device.status.refresh()
                        print('5')
                        self.bedroom_motion_sensor_active = device.status.values['motion'] == 'inactive'
                    elif device.label == 'pills_motion_sensor':  # sensor_2
                        await device.status.refresh()
                        print('6')
                        self.pill_motion_sensor_active = device.status.values['motion'] == 'inactive'

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
