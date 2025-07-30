import paho.mqtt.client as mqtt
import requests
import json
import threading
import time
from datetime import datetime, timezone, timedelta

# Config dari file config Anda
MQTT_BROKER = "10.177.143.122"
MQTT_PORT = 1883
MQTT_TOPIC_BUTTON = "/button/task"
MQTT_TOPIC_GIN = "/gin/task"  # New GIN topic

WEB_API_BASE_URL = "http://34.1.143.136:8080"
BUTTON_PRESS_ENDPOINT = f"{WEB_API_BASE_URL}/api/button-press"
TASK_STATUS_ENDPOINT = f"{WEB_API_BASE_URL}/api/task-status"
TASK_HISTORY_ENDPOINT = f"{WEB_API_BASE_URL}/api/task-history"
GIN_DATA_ENDPOINT = f"{WEB_API_BASE_URL}/api/gin-data"  # New GIN endpoint


class MQTTBridgeWithGIN:
    def __init__(self):
        # MQTT setup
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message

        # Task descriptions
        self.task_descriptions = {
            1: "Mengambil trolley berisi ke BES point port",
            2: "Mengambil trolley kosong ke INPUT point port",
            3: "Fetching trolley from outbound",
            4: "Fetching trolley from chute, delivering to GIN",
            5: "Fetching trolley from chute, delivering to GIN"
        }

        self.button_names = {
            1: "Go",
            2: "Back",
            3: "Button 3",
            4: "Button 4",
            5: "Button 5"
        }

        self.task_start_times = {}

        # ✅ NEW: GIN data storage
        self.gin_data_buffer = []  # Temporary storage for scanned GIN data
        self.gin_lock = threading.Lock()  # Thread safety for GIN data

    def send_to_web_api(self, endpoint, data, endpoint_name="API"):
        """Send data to web API"""
        try:
            print(f"🚀 Sending to {endpoint_name}: {json.dumps(data, indent=2)}")
            response = requests.post(endpoint, json=data, timeout=5)

            if response.status_code == 200:
                print(f"✅ Successfully sent data to {endpoint_name}")
                try:
                    return response.json()
                except:
                    return {"success": True}
            else:
                print(f"⚠️ {endpoint_name} responded with status {response.status_code}")
                print(f"Response: {response.text}")
        except requests.exceptions.RequestException as e:
            print(f"❌ Failed to send data to {endpoint_name}: {str(e)}")
        return None

    def generate_task_attributes(self, button_id, status, additional_attrs=None):
        """Generate task attributes"""
        attrs = {
            "attr1": f"Zone {button_id}",
            "attr2": f"Cart-{str(int(datetime.now().timestamp()))[-3:]}",
            "attr3": self.get_load_type(button_id),
            "attr4": self.get_status_description(status),
            "attr5": "Auto"
        }

        if additional_attrs:
            attrs.update(additional_attrs)

        return attrs

    def get_load_type(self, button_id):
        """Get load type based on button"""
        load_types = {1: "Full", 2: "Empty", 3: "Empty", 4: "Full", 5: "Empty"}
        return load_types.get(button_id, "Unknown")

    def get_status_description(self, status):
        """Get status description"""
        descriptions = {
            "in_progress": "Task In Progress",
            "success": "Task Completed Successfully",
            "failed": "Task Failed"
        }
        return descriptions.get(status, status)

    # ✅ NEW: Handle GIN scanning
    def handle_gin_scan(self, gin_message):
        """Handle GIN barcode scanning"""
        try:
            # Parse message format: "no_gin;num_coli" (e.g., "A1;30")
            if ';' not in gin_message:
                print(f"❌ Invalid GIN message format: {gin_message}")
                return False

            no_gin, num_coli = gin_message.split(';', 1)
            num_coli = int(num_coli)

            wib = timezone(timedelta(hours=7))
            scan_time = datetime.now(wib).isoformat()

            gin_entry = {
                "no_gin": no_gin.strip(),
                "jumlah_coli": num_coli,
                "scan_time": scan_time
            }

            with self.gin_lock:
                self.gin_data_buffer.append(gin_entry)

            print(f"📦 GIN Scanned: {no_gin} with {num_coli} coli")
            print(f"📊 Total GIN in buffer: {len(self.gin_data_buffer)}")

            # Show current buffer
            print("📋 Current GIN buffer:")
            for i, gin in enumerate(self.gin_data_buffer, 1):
                print(f"   {i}. {gin['no_gin']} - {gin['jumlah_coli']} coli")

            return True

        except ValueError as e:
            print(f"❌ Error parsing GIN message '{gin_message}': {e}")
            return False
        except Exception as e:
            print(f"❌ Unexpected error handling GIN scan: {e}")
            return False

    # ✅ NEW: Send GIN data to backend
    def send_gin_data_to_backend(self, status):
        """Send all buffered GIN data to backend when button 4 is pressed"""
        if status == "success":
            with self.gin_lock:
                if not self.gin_data_buffer:
                    print("📦 No GIN data to send")
                    return []

                gin_data_copy = self.gin_data_buffer.copy()

            wib = timezone(timedelta(hours=7))
            start_time = datetime.now(wib).isoformat()

            # Prepare GIN data for backend
            gin_records = []
            for gin_entry in gin_data_copy:
                gin_record = {
                    "no_gin": gin_entry["no_gin"],
                    "jumlah_coli": gin_entry["jumlah_coli"],
                    "start_time": start_time,
                    "scan_time": gin_entry["scan_time"]
                }
                gin_records.append(gin_record)

            # Send to backend
            gin_payload = {
                "gin_records": gin_records
            }

            print(f"📤 Sending {len(gin_records)} GIN records to backend...")
            response = self.send_to_web_api(GIN_DATA_ENDPOINT, gin_payload, "GIN Data")

            if response and response.get('success'):
                # Clear buffer after successful send
                with self.gin_lock:
                    self.gin_data_buffer.clear()
                print("✅ GIN buffer cleared after successful send")

            return gin_records
        with self.gin_lock:
            self.gin_data_buffer.clear()
        print("✅ GIN buffer cleared after successful send")
        return None

    # ✅ MODIFIED: Generate GIN-enhanced attributes for button 4
    def generate_gin_enhanced_attributes(self, button_id, status):
        """Generate task attributes with GIN data for button 4"""
        attrs = self.generate_task_attributes(button_id, status)

        if button_id == 1:  # Only for button 1 (Go button)
            with self.gin_lock:
                if self.gin_data_buffer:
                    # Use first GIN for ATTR2 and ATTR3
                    first_gin = self.gin_data_buffer[0]
                    attrs["attr2"] = first_gin["no_gin"]  # GIN number
                    attrs["attr3"] = str(first_gin["jumlah_coli"])  # Coli count

                    # If multiple GINs, show summary in attr5
                    if len(self.gin_data_buffer) > 1:
                        total_coli = sum(gin["jumlah_coli"] for gin in self.gin_data_buffer)
                        attrs["attr5"] = f"Multi-GIN ({len(self.gin_data_buffer)} GINs, {total_coli} total coli)"
                    else:
                        attrs["attr5"] = "Single-GIN"

        return attrs

    def send_to_task_history(self, button_id, status, start_time=None, additional_attrs=None):
        """Send data to task history endpoint"""
        # ✅ MODIFIED: Use GIN-enhanced attributes for button 4
        if button_id == 1:
            attrs = self.generate_gin_enhanced_attributes(button_id, status)
        else:
            attrs = self.generate_task_attributes(button_id, status, additional_attrs)

        history_data = {
            "buttonId": button_id,
            "status": status,
            "startTime": start_time,
            "attr1": attrs.get("attr1", f"Zone {button_id}"),
            "attr2": attrs.get("attr2", f"Cart-{str(int(datetime.now().timestamp()))[-3:]}"),
            "attr3": attrs.get("attr3", self.get_load_type(button_id)),
            "attr4": attrs.get("attr4", self.get_status_description(status)),
            "attr5": attrs.get("attr5", "Auto")
        }

        # return self.send_to_web_api(TASK_HISTORY_ENDPOINT, history_data, "Task History")

    def on_connect(self, client, userdata, flags, rc):
        print(f"🔗 Connected to MQTT Broker! Result code: {rc}")

        # Subscribe to all button topics + GIN topic
        topics = [
            MQTT_TOPIC_BUTTON,
            MQTT_TOPIC_GIN  # ✅ NEW: Subscribe to GIN topic
        ]

        for topic in topics:
            client.subscribe(topic)
            print(f"📡 Subscribed to: {topic}")

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode()

        print(f"\n📨 Received MQTT message:")
        print(f"   Topic: {topic}")
        print(f"   Payload: {payload}")

        # ✅ NEW: Handle GIN topic
        if topic == MQTT_TOPIC_GIN:
            print("📦 Processing GIN scan...")
            self.handle_gin_scan(payload)
            return

        # Handle button topics
        try:
            button_id = int(payload)
        except ValueError:
            print("❌ Invalid MQTT payload received")
            return

        if button_id in [1, 2, 3, 4, 5]:
            print(f"🎯 Processing button {button_id}: {self.task_descriptions[button_id]}")

            # ✅ MODIFIED: Special handling for button 4 (Go button)
            if button_id == 1:
                print("🚛 Button 4 pressed - Processing with GIN data...")
            # Send button press data to web API
            button_press_data = {
                "buttonId": button_id,
                "taskDescription": self.task_descriptions[button_id]
            }
            self.send_to_web_api(BUTTON_PRESS_ENDPOINT, button_press_data, "Button Press")

            wib = timezone(timedelta(hours=7))
            # Store start time
            start_time = datetime.now(wib).isoformat()
            self.task_start_times[button_id] = start_time

            # Send initial status
            self.publish_status(button_id, "in_progress", start_time)

            # Send to task history
            self.send_to_task_history(button_id, "in_progress", start_time)

            # Simulate task execution
            threading.Thread(target=self.simulate_task_execution, args=(button_id,)).start()

        else:
            print(f"❌ Message unrecognized: {button_id}")

    def publish_status(self, button_id, status, start_time=None):
        """Publish status to MQTT and Web API"""
        print(f"📤 Publishing status '{status}' for button {button_id}")

        # Send to MQTT
        self.mqtt_client.publish(f"/button{button_id}/status", status)
        self.mqtt_client.publish(f"/task/status", status)

        # Send to Web API
        status_data = {
            "buttonId": button_id,
            "status": status,
            "startTime": start_time or self.task_start_times.get(button_id)
        }

        api_response = self.send_to_web_api(TASK_STATUS_ENDPOINT, status_data, "Task Status")

        if api_response and 'stats' in api_response:
            stats = api_response['stats']
            print(f"📊 Overall stats - Total: {stats['total']}, Success: {stats['success']}, Failed: {stats['failed']}")

    def simulate_task_execution(self, button_id):
        """Simulate task execution (replace behavior tree)"""
        print(f"⏳ Simulating task execution for button {button_id}...")

        # Setup placeholder to capture status from MQTT
        status_event = threading.Event()
        status_holder = {"status": None}

        def on_mqtt_message(client, userdata, msg):
            topic = msg.topic
            payload = msg.payload.decode()
            if topic == "/task/status":
                print(f"📥 Received MQTT status: {payload}")
                status_holder["status"] = payload
                status_event.set()
        # Subscribe to MQTT topic
        self.mqtt_client.subscribe("/task/status")
        self.mqtt_client.message_callback_add("/task/status", on_mqtt_message)

        status_received = status_event.wait(timeout=500)

        # Unsubscribe and remove callback to clean up
        self.mqtt_client.message_callback_remove("/task/status")
        self.mqtt_client.unsubscribe("/task/status")

        if status_received:
            final_status = status_holder["status"]
        else:
            print("⚠️ Timeout waiting for MQTT status. Marking as 'failed'")
            final_status = "failed"

        print(f"✅ Task simulation completed: {final_status}")

        # Send final status
        start_time = self.task_start_times.get(button_id)
        self.publish_status(button_id, final_status, start_time)

        # Send to task history
        additional_attrs = {
            "attr2": f"Cart-{button_id}-{str(int(datetime.now().timestamp()))[-4:]}",
            "attr5": "Auto-Simulated"
        }
        self.send_to_task_history(button_id, final_status, start_time, additional_attrs)
        if button_id == 1:
            self.send_gin_data_to_backend(final_status)

        # Cleanup
        if button_id in self.task_start_times:
            del self.task_start_times[button_id]

    # ✅ NEW: Method to show current GIN buffer status
    def show_gin_status(self):
        """Show current GIN buffer status"""
        with self.gin_lock:
            if not self.gin_data_buffer:
                print("📦 GIN Buffer: Empty")
            else:
                print(f"📦 GIN Buffer ({len(self.gin_data_buffer)} items):")
                for i, gin in enumerate(self.gin_data_buffer, 1):
                    print(f"   {i}. {gin['no_gin']} - {gin['jumlah_coli']} coli (scanned: {gin['scan_time']})")

    def start(self):
        """Start the MQTT bridge"""
        try:
            print("🚀 Starting MQTT Bridge with GIN Support...")
            print(f"🔗 Connecting to MQTT Broker: {MQTT_BROKER}:{MQTT_PORT}")
            print(f"🌐 Backend API: {WEB_API_BASE_URL}")
            print(f"📦 GIN Topic: {MQTT_TOPIC_GIN}")
            print("=" * 60)
            print("📋 Usage:")
            print("   1. Scan GIN barcodes -> sends to /gin/task with format 'no_gin;num_coli'")
            print("   2. Press Button 4 -> processes all buffered GIN data")
            print("   3. GIN data will be stored in separate table and used in task attributes")
            print("=" * 60)

            self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
            self.mqtt_client.loop_forever()

        except KeyboardInterrupt:
            print("\n👋 Shutting down...")
            self.show_gin_status()  # Show final GIN status
            self.mqtt_client.disconnect()
        except Exception as e:
            print(f"❌ Connection error: {e}")


if __name__ == "__main__":
    bridge = MQTTBridgeWithGIN()
    bridge.start()
