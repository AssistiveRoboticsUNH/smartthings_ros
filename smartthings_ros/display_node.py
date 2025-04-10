import zmq
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DisplayStatusSubscriber(Node):
    def __init__(self, zmq_socket):
        super().__init__('display_status_subscriber')
        
        # ✅ ROS 2 Subscriber to "display_status" topic
        self.subscription = self.create_subscription(
            String,
            "display_status",
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning
        
        self.zmq_socket = zmq_socket  # ✅ Use the shared ZeroMQ socket

        self.get_logger().info("✅ Subscribed to 'display_status' and connected to ZMQ")

    def listener_callback(self, msg):
        """ Callback function for processing received ROS messages """
        self.get_logger().info(f"📡 Received Display Status: {msg.data}")

        try:
            if msg.data == "TURN_ON":
                self.zmq_socket.send_string("TURN_ON")  # ✅ Send "0" when TURN_ON
                self.get_logger().info("📤 Sent ZeroMQ message: TURN_ON")

            elif msg.data == "TURN_OFF":
                self.zmq_socket.send_string("TURN_OFF")  # ✅ Send "1" when TURN_OFF
                self.get_logger().info("📤 Sent ZeroMQ message: TURN_OFF")

            else:
                self.get_logger().warn(f"⚠️ Unknown message: {msg.data}")

        except Exception as e:
            self.get_logger().error(f"❌ Error sending message to ZMQ: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    # ✅ Initialize ZeroMQ Context and Socket (PUSH pattern)
    context = zmq.Context()
    zmq_socket = context.socket(zmq.PUB)
    zmq_socket.connect("tcp://192.168.50.57:5556")  # ✅ Change this if needed

    # ✅ Create the subscriber node
    display_status_subscriber = DisplayStatusSubscriber(zmq_socket)

    try:
        while True:
            rclpy.spin_once(display_status_subscriber, timeout_sec=5.0)

    except KeyboardInterrupt:
        display_status_subscriber.get_logger().info("🛑 Shutting down DisplayStatusSubscriber")

    finally:
        display_status_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()