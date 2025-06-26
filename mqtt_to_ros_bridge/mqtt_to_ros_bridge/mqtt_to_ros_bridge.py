import rclpy
import paho.mqtt.client as mqtt
import os
import subprocess
import threading
import requests
import json
from datetime import datetime
from configparser import ConfigParser
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from mqtt_to_ros_bridge.config import MQTT_BROKER, MQTT_PORT, MQTT_TOPIC_BUTTON1, MQTT_TOPIC_BUTTON2, MQTT_TOPIC_BUTTON3, MQTT_TOPIC_BUTTON4, MQTT_TOPIC_BUTTON5, WEB_API_BASE_URL, BUTTON_PRESS_ENDPOINT, TASK_STATUS_ENDPOINT


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

        # Task descriptions for web API
        self.task_descriptions = {
            1: "Fetching trolley from chute, delivering to GIN",
            2: "Fetching trolley from outbound",
            3: "Fetching trolley from outbound",
            4: "Mengambil trolley berisi ke BES point port",
            5: "Mengambil trolley kosong ke INPUT point port"
        }
        self.task_start_times = {}

    def send_to_web_api(self, endpoint, data):
        """Send data to web API"""
        try:
            response = requests.post(endpoint, json=data, timeout=5)
            if response.status_code == 200:
                self.get_logger().info(f"Successfully sent data to web API: {endpoint}")
                return response.json()
            else:
                self.get_logger().warning(f"Web API responded with status {response.status_code}")
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"Failed to send data to web API: {str(e)}")
        return None

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT Broker!")
        client.subscribe(MQTT_TOPIC_BUTTON1)
        client.subscribe(MQTT_TOPIC_BUTTON2)
        client.subscribe(MQTT_TOPIC_BUTTON3)
        client.subscribe(MQTT_TOPIC_BUTTON4)
        client.subscribe(MQTT_TOPIC_BUTTON5)

    def on_message(self, client, userdata, msg):
        self.get_logger().info(f"Received MQTT message: {msg.payload.decode()}")

        try:
            global data 
            data = int(msg.payload.decode())
        except ValueError:
            self.get_logger().error("Invalid MQTT payload received")
            return

        if data in [1, 2, 3, 4, 5]:
            self.get_logger().info(f"Processing button {data} message: {self.task_descriptions[data]}")
            
            # Send button press data to web API
            button_press_data = {
                "buttonId": data,
                "taskDescription": self.task_descriptions[data]
            }
            self.send_to_web_api(BUTTON_PRESS_ENDPOINT, button_press_data)
            
            # Store start time for duration calculation
            self.task_start_times[data] = datetime.now().isoformat()
            
            # Run behavior tree
            self.run_behavior_tree(data)
        else:
            self.get_logger().error(f"Message unrecognized: {data}")

    def run_behavior_tree(self, button_id):
        # Define the command to run the behavior trees
        bt_files = {
            # Button 2 & 3 have the same functionalities
            1: "tryBT1",
            2: "tryBT2",
            3: "tryBT2",
            4: "tryBT1",
            5: "tryBT2"
        }

        bt_file = bt_files.get(button_id)
        if not bt_file:
            self.get_logger().error(f"No behavior tree file found for button {button_id}")
            return

        bt_command = [
            "ros2", "run", "talosbot_bt", "start_tree",
            "--ros-args", "-p",
            f"bt_file:=/home/gdnuser/ros2_ws/src/talosbot/talosbot_bt/behavior_trees/{bt_file}.xml"
        ]

        try:
            self.get_logger().info(f"Publishing 'in_progress' for button {button_id}")
            self.mqtt_client.publish(f"/button{button_id}/status", "in_progress")
            self.mqtt_client.publish(f"/task/status", "in_progress")

            status_data = {
                "buttonId": button_id,
                "status": "in_progress",
                "startTime": self.task_start_times.get(button_id)
            }
            self.send_to_web_api(TASK_STATUS_ENDPOINT, status_data)

            # Run the behavior tree in a separate thread
            threading.Thread(target=self.execute_behavior_tree, args=(bt_command, button_id)).start()

        except Exception as e:
            self.get_logger().error(f"Failed to run behavior tree: {str(e)}")
            self.publish_status(button_id, "failed")

    def publish_status(self, button_id, status):
        """Publish status to both MQTT and Web API"""
        # Send to MQTT
        self.mqtt_client.publish(f"/button{button_id}/status", status)
        self.mqtt_client.publish(f"/task/status", status)
        
        # Send to Web API
        status_data = {
            "buttonId": button_id,
            "status": status,
            "startTime": self.task_start_times.get(button_id)
        }
        api_response = self.send_to_web_api(TASK_STATUS_ENDPOINT, status_data)
        
        if api_response and 'stats' in api_response:
            stats = api_response['stats']
            self.get_logger().info(f"Overall stats - Total: {stats['total']}, Success: {stats['success']}, Failed: {stats['failed']}")

    def execute_behavior_tree(self, bt_command, button_id):
        try:
            self.get_logger().info("Starting behavior tree...")

            process = subprocess.Popen(bt_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

            # Wait for the full process to finish and collect all output
            stdout, stderr = process.communicate()
            result_code = process.returncode

            # Log everything
            self.get_logger().info(f"[BT STDOUT]\n{stdout}")
            self.get_logger().error(f"[BT STDERR]\n{stderr}")

            # Analyze full output after process completes
            joined_stdout = stdout.strip()

            if "FAILURE" in joined_stdout or "Task FAILED" in joined_stdout:
                self.get_logger().info("Behavior tree execution FAILED based on output.")
                self.publish_status(button_id, "failed")

            elif "SUCCESS" in joined_stdout or "Task SUCCEEDED" in joined_stdout:
                self.get_logger().info("Behavior tree execution SUCCEEDED based on output.")
                self.publish_status(button_id, "success")

            else:
                # Fallback to return code
                if result_code == 0:
                    self.get_logger().info("BT process completed with code 0 (success fallback).")
                    self.publish_status(button_id, "success")
                else:
                    self.get_logger().error("BT process failed with non-zero return code.")
                    self.publish_status(button_id, "failed")

        except Exception as e:
            self.get_logger().error(f"Failed to run behavior tree: {str(e)}")
            self.publish_status(button_id, "failed")

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

def main(args=None):
    rclpy.init(args=args)
    mqtt_to_ros_bridge = MQTTtoROSBridge()
    rclpy.spin(mqtt_to_ros_bridge)
    mqtt_to_ros_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()