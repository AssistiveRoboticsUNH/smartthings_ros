import asyncio
from kasa import SmartPlug

import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class SmartPlugPublisher(Node):

    def __init__(self, smartthings_response, update_period):
        super().__init__('smart_pulg')
        # self.publisher_motion_door = self.create_publisher(Bool, 'smartthings_sensors_motion_door', 10)
        self.charging_publisher = self.create_publisher(Int32, 'charging', 10)

        self.timer = self.create_timer(update_period, self.timer_callback)
        self.smartplug_response = smartthings_response

    def timer_callback(self):
        if self.smartplug_response.updated:
            msg = Int32()
            msg.data = self.smartplug_response.powered
            self.charging_publisher.publish(msg)


class SmartPlugResponse:
    def __init__(self, update_period):
        self.updated = False
        self.smart_plug = SmartPlug("192.168.1.44")
        self.powered = 0

        self.update_period = update_period

    async def read_device(self):
        while True:
            start = float(time.time_ns() // 1_000_000_000)
            await self.smart_plug.update()
            if (self.smart_plug.emeter_realtime.power > 3):
                self.powered = 1
            else:
                self.powered = 0
            end = float(time.time_ns() // 1_000_000_000)
            await asyncio.sleep(self.update_period - (end - start))
            self.updated = True


def main(args=None):
    update_period = 1  # 1 sec
    smartplug_response = SmartPlugResponse(update_period)
    x = threading.Thread(target=asyncio.run, args=(smartplug_response.read_device(),))
    x.start()

    rclpy.init(args=args)

    minimal_publisher = SmartPlugPublisher(smartplug_response, update_period)

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
