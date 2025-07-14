import keyboard
import paho.mqtt.client as mqtt
import time

MQTT_BROKER = "192.168.240.222"   # ← Change this to your MQTT broker's IP
MQTT_PORT = 1883
MQTT_TOPIC = "/gin/task"

client = mqtt.Client()
client.connect(MQTT_BROKER, MQTT_PORT)

buffer = ""

def on_key(event):
    global buffer
    if event.name == "enter":
        process_scan(buffer.strip())
        buffer = ""
    elif len(event.name) == 1:  # Single character keys
        buffer += event.name
    elif event.name == "space":
        buffer += " "
    elif event.name == "backspace":
        buffer = buffer[:-1]

def process_scan(data):
    if not data:
        return
    print(f"[✓] Scanned: {data}")
    payload = f"{data}"

    client.publish(MQTT_TOPIC, payload)
    print(f"[MQTT] Sent → Topic: {MQTT_TOPIC} | Payload: {payload}")

keyboard.on_press(on_key)

print("🔍 Waiting for scan input (global)... Press Ctrl+C to stop.")
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n[!] Stopped.")