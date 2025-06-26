# config backend
WEB_API_BASE_URL = "http://localhost:3001/api"
BUTTON_PRESS_ENDPOINT = f"{WEB_API_BASE_URL}/button-press"
TASK_STATUS_ENDPOINT = f"{WEB_API_BASE_URL}/task-status"

# config mqtt
MQTT_BROKER = "192.168.10.222" #Your PC's IP
MQTT_PORT = 1883
MQTT_TOPIC_BUTTON1 = "/button1/task"
MQTT_TOPIC_BUTTON2 = "/button2/task"
MQTT_TOPIC_BUTTON3 = "/button3/task"
MQTT_TOPIC_BUTTON4 = "/button4/task"
MQTT_TOPIC_BUTTON5 = "/button5/task"