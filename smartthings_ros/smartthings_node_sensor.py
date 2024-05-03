import aiohttp
import asyncio
import pysmartthings
import threading
import time
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int32


class SmartthingsPublisher(Node):

    def __init__(self, smartthings_response, update_period):
        super().__init__('smartthings_publisher')
        # self.publisher_motion_door = self.create_publisher(Bool, 'smartthings_sensors_motion_door', 10)
        self.publisher_pills_motion_sensor = self.create_publisher(Int32, 'person_taking_medicine', 10)
        # self.publisher_bedroom_motion_sensor = self.create_publisher(Bool, 'smartthings_bedroom_motion_sensor', 10)
        # self.publisher_bathroom_motion_sensor = self.create_publisher(Bool, 'smartthings_bathroom_motion_sensor', 10)
        self.publisher_dining_motion_sensor = self.create_publisher(Int32, 'person_eating', 10)

        self.publisher_bedroom_sensor_door = self.create_publisher(Bool, 'smartthings_sensors_door_bedroom', 10)
        # self.publisher_living_sensor_door = self.create_publisher(Bool, 'smartthings_living_sensor_door', 10)
        self.publisher_bathroom_sensor_door = self.create_publisher(Bool, 'smartthings_sensors_door_bathroom', 10)
        self.publisher_main_sensor_door = self.create_publisher(Bool, 'smartthings_sensors_door_outdoor', 10)

        self.timer = self.create_timer(update_period, self.timer_callback)
        self.smartthings_response = smartthings_response

    def timer_callback(self):
        if self.smartthings_response.updated:
            # door sensors
            msg = Bool()
            msg.data = not self.smartthings_response.bedroom_door_sensor_closed
            self.publisher_bedroom_sensor_door.publish(msg)
            
            print("i am calling")
            # msg = Bool()
            # msg.data = not self.smartthings_response.livingroom_door_sensor_closed
            # self.publisher_living_smartthings_sensors_door_outdoorsensor_door.publish(msg)

            msg = Bool()
            msg.data = not self.smartthings_response.bathroom_door_sensor_closed
            self.publisher_bathroom_sensor_door.publish(msg)
            
            msg = Bool()
            msg.data = not self.smartthings_response.main_door_sensor_closed
            self.publisher_main_sensor_door.publish(msg)
            

            # motion sensors
            msg = Int32()
            msg.data = int(not self.smartthings_response.pills_motion_sensor_inactive)
            self.publisher_pills_motion_sensor.publish(msg)
            
            msg = Int32()
            msg.data = int(not self.smartthings_response.dining_motion_sensor_active)
            # self.publisher_dining_motion_sensor.publish(msg)
            
            # msg = Bool()
            # msg.data = not self.smartthings_response.bedroom_motion_sensor_active
            # self.publisher_bedroom_motion_sensor.publish(msg)
            # msg = Bool()
            # msg.data = not self.smartthings_response.bathroom_motion_sensor_action
            # self.publisher_bathroom_motion_sensor.publish(msg)


class SmartthingsResponse:
    def __init__(self, update_period):
        self.updated = False
        # self.door_sensor_closed = None   #
        self.livingroom_door_sensor_closed = None  # livingroom door sensor
        self.bedroom_door_sensor_closed = None  # bedroom door sensor
        self.bathroom_door_sensor_closed = None  # bathroom door sensor
        self.main_door_sensor_closed = None

        # self.door_motion_sensor_active = None
        self.pills_motion_sensor_inactive = None
        self.dining_motion_sensor_active = None
        # self.bedroom_motion_sensor_activesmartthings_sensors_door_outdoor = None
        # self.bathroom_motion_sensor_action = None

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
            # order = [motion_Sensor_1, door sensor/person_eating, motion sensor 2]
            while True:
                start = float(time.time_ns() // 1_000_000_000)
                for device in devices:
                    print(device.label)

                    # door sensor
                    if device.label == 'bedroom_sensor_door':  # bedroom door sensor check
                        print('bedroom_door 1', device.status.values['contact'])
                        await device.status.refresh()
                        self.bedroom_door_sensor_closed = device.status.values['contact'] == 'closed'

                    # elif device.label == 'livingroom_sensor_door':  # livingroom door sensor check
                    #     print('livingroom_door 2', device.status.values['contact'])
                    #     await device.status.refresh()
                    #     self.livingroom_door_sensor_closed = device.status.values['contact'] == 'closed'

                    elif device.label == 'bathroom_sensor_door':  # bathroom door sensor
                        print('bathroom_door 3', device.status.values['contact'])
                        await device.status.refresh()
                        self.bathroom_door_sensor_closed = device.status.values['contact'] == 'closed'

                    elif device.label == 'main_sensor_door':  # main door sensor
                        print('main_door 4', device.status.values['motion'])
                        await device.status.refresh()
                        self.main_door_sensor_closed = device.status.values['contact'] == 'closed'

                    # # motion sensor
                    elif device.label == 'dining_motion_sensor':  # sensor_2
                        await device.status.refresh()
                        print('dining_motion 5', device.status.values['motion'])
                        self.dining_motion_sensor_active = device.status.values['motion'] == 'inactive'
                    #
                    # elif device.label == 'bedroom_motion_sensor':  # sensor_2
                    #     await device.status.refresh()
                    #     print('bedroom_motion_sensor', device.status.values['motion'])
                    #     self.bedroom_motion_sensor_active = device.status.values['motion'] == 'inactive'

                    elif device.label == 'pills_motion_sensor':  # sensor_2
                        await device.status.refresh()
                        print('pills_motion 7,', device.status.values['motion'])
                        self.pills_motion_sensor_inactive = device.status.values['motion'] == 'inactive'

                    # elif device.label == 'bathroom_motion_sensor':  # sensor_2
                    #     await device.status.refre/person_eatingsh()
                    #     print('bathroom_motion 8', device.status.values['motion'])
                    #     self.bathroom_motion_sensor_action = device.status.values['motion'] == 'inactive'

                end = float(time.time_ns() // 1_000_000_000)
                await asyncio.sleep(self.update_period - (end - start))
                self.updated = True


def main(args=None):
    update_period = 1  # 1 sec
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
