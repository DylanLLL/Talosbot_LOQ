import rclpy
import paho.mqtt.client as mqtt
import os #library to locate the exact location of file
import subprocess #library to simulate CLI

from configparser import ConfigParser
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

#Reading and parsing the contents in the configuration file
current_dir = os.path.dirname(os.path.realpath(__file__))
file = os.path.join(current_dir, 'config.ini')
config = ConfigParser()
config.read(file)

MQTT_BROKER = "192.168.1.183" #Your PC's IP
MQTT_PORT = 1883
MQTT_TOPIC_BUTTON1 = "/esp32/poseHome1"
MQTT_TOPIC_BUTTON2 = "/esp32/poseHome2"

class MQTTtoROSBridge(Node):
    def __init__(self):
        super().__init__('mqtt_to_ros_bridge')

        # Publisher for /target_pose
        self.publisher = self.create_publisher(PoseStamped, '/target_pose', 10)

        # Action client for NavigateToPose
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # MQTT setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT Broker!")
        client.subscribe(MQTT_TOPIC_BUTTON1)
        client.subscribe(MQTT_TOPIC_BUTTON2)

    def on_message(self, client, userdata, msg):
        self.get_logger().info(f"Received MQTT message: {msg.payload.decode()}")

        try:
            data = int(msg.payload.decode())
        except ValueError:
            self.get_logger().error("Invalid MQTT payload received")
            return

        if data == 1:
            self.get_logger().info("Processing button 1 message")
            pos_x = float(config['home1']['pos_x'])
            pos_y = float(config['home1']['pos_y'])
            quat_w = float(config['home1']['quat_w'])
            quat_z = float(config['home1']['quat_z'])

            # Create and publish PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.pose.position.x = pos_x
            pose_msg.pose.position.y = pos_y
            pose_msg.pose.orientation.w = quat_w
            pose_msg.pose.orientation.z = quat_z

            self.publisher.publish(pose_msg)
            self.get_logger().info("Published pose to /target_pose")

            # Send goal to Nav2
            self.send_goal_to_nav2(pose_msg)

        elif data == 2:
            self.get_logger().info("Processing button 2 message: Running behavior tree")
            self.run_behavior_tree()

        else:
            self.get_logger().error(f"Message unrecognized: {data}")
            return

    def send_goal_to_nav2(self, pose):
        # Create a NavigateToPose goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose  # Use the received PoseStamped

        self.get_logger().info(f"Sending goal to Nav2: {pose}")
        
        # Wait for the action server to become available
        self.nav_to_pose_client.wait_for_server()

        # Send goal and handle the result
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by Nav2.')
            return

        self.get_logger().info('Goal accepted by Nav2.')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        if result:
            self.get_logger().info(f'Nav2 Result: {result.status}')
        else:
            self.get_logger().info('Goal failed.')

    def run_behavior_tree(self):
        # Define the command to run the behavior trees
        bt_file = "tryloop3"

        bt_command = [
            "ros2", "run", "talosbot_bt", "start_tree",
            "--ros-args", "-p",
            f"bt_file:=/home/gdnuser/ros2_ws/src/talosbot/talosbot_bt/behavior_trees/{bt_file}.xml"
        ]

        try:
            # Run the command using subprocess
            self.get_logger().info("Starting behavior tree...")
            result = subprocess.run(bt_command, capture_output=True, text=True)

            # Log the output
            if result.returncode == 0:
                self.get_logger().info(f"Behavior tree started successfully:\n{result.stdout}")
            else:
                self.get_logger().error(f"Behavior tree failed to start:\n{result.stderr}")

        except Exception as e:
            self.get_logger().error(f"Failed to run behavior tree: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    mqtt_to_ros_bridge = MQTTtoROSBridge()
    rclpy.spin(mqtt_to_ros_bridge)
    mqtt_to_ros_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
