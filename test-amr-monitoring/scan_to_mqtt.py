import keyboard
import paho.mqtt.client as mqtt
import time

MQTT_BROKER = "10.177.143.122"
MQTT_PORT = 1883
MQTT_TOPIC = "/gin/task"

buffer = ""
connected = False

client = mqtt.Client()

# === MQTT CALLBACKS ===
def on_connect(client, userdata, flags, rc):
    global connected
    if rc == 0:
        connected = True
        print("[MQTT] Connected to broker")
    else:
        print(f"[MQTT] Failed to connect, return code {rc}")

def on_disconnect(client, userdata, rc):
    global connected
    connected = False
    print("[MQTT] Disconnected. Will retry to reconnect...")

# Assign callbacks
client.on_connect = on_connect
client.on_disconnect = on_disconnect

# Non-blocking loop
client.loop_start()

# Try to connect (retry loop)
def connect_mqtt():
    while not connected:
        try:
            print("[MQTT] Attempting to connect...")
            client.connect(MQTT_BROKER, MQTT_PORT)
            time.sleep(2)
        except Exception as e:
            print(f"[MQTT] Connection failed: {e}")
            time.sleep(3)

connect_mqtt()

# === Keyboard Scanner Handler ===
def on_key(event):
    global buffer
    if event.name == "enter":
        process_scan(buffer.strip())
        buffer = ""
    elif len(event.name) == 1:
        buffer += event.name
    elif event.name == "space":
        buffer += " "
    elif event.name == "backspace":
        buffer = buffer[:-1]

def process_scan(data):
    if not data:
        return
    print(f"[✓] Scanned: {data}")
    if connected:
        result = client.publish(MQTT_TOPIC, data)
        status = result.rc
        if status == 0:
            print(f"[MQTT] Sent → Topic: {MQTT_TOPIC} | Payload: {data}")
        else:
            print("[MQTT] Failed to send message")
    else:
        print("[MQTT] Not connected, message not sent")

keyboard.on_press(on_key)

print("🔍 Waiting for scan input (global)... Press Ctrl+C to stop.")
try:
    while True:
        if not connected:
            connect_mqtt()
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[!] Stopped.")
    client.loop_stop()
    client.disconnect()
