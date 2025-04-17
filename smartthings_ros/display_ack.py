import zmq
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ZmqToRosPublisher(Node):
    def __init__(self, zmq_socket):
        super().__init__('zmq_to_ros_publisher')
        
        # ✅ ROS 2 Publisher to "screen_ack"
        self.publisher = self.create_publisher(String, "screen_ack", 10)
        self.zmq_socket = zmq_socket  # ✅ Shared ZeroMQ socket for incoming messages

        self.get_logger().info("✅ Ready to receive ZMQ messages and publish to /screen_ack")

    def poll_and_publish(self):
        """ Poll ZMQ socket and publish to ROS 2 if message received """
        try:
            msg = self.zmq_socket.recv_string(flags=zmq.NOBLOCK)  # Non-blocking recv
            ros_msg = String()
            ros_msg.data = msg
            self.publisher.publish(ros_msg)
            self.get_logger().info(f"📥 Received from Android & published: {msg}")
        except zmq.Again:
            pass  # No message yet, skip this round
        except Exception as e:
            self.get_logger().error(f"❌ Error reading from ZMQ: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    # ✅ ZMQ PULL socket binds to port 5557
    context = zmq.Context()
    zmq_socket = context.socket(zmq.PULL)
    zmq_socket.bind("tcp://*:5557")  # Android app connects to this port

    # ✅ Create the ZMQ→ROS publisher node
    zmq_to_ros = ZmqToRosPublisher(zmq_socket)

    try:
        while True:
            rclpy.spin_once(zmq_to_ros, timeout_sec=0.1)
            zmq_to_ros.poll_and_publish()  # Check ZMQ socket and publish if needed

    except KeyboardInterrupt:
        zmq_to_ros.get_logger().info("🛑 Shutting down ZmqToRosPublisher")

    finally:
        zmq_to_ros.destroy_node()
        rclpy.shutdown()
        zmq_socket.close()
        context.term()

if __name__ == "__main__":
    main()
