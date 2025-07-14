# config backend
WEB_API_BASE_URL = "http://localhost:3001/api"
BUTTON_PRESS_ENDPOINT = f"{WEB_API_BASE_URL}/button-press"
TASK_STATUS_ENDPOINT = f"{WEB_API_BASE_URL}/task-status"
TASK_HISTORY_ENDPOINT = f"{WEB_API_BASE_URL}/task-history"
GIN_DATA_ENDPOINT = f"{WEB_API_BASE_URL}/gin-data"

# config mqtt
MQTT_BROKER = "192.168.240.222" #Your PC's IP
MQTT_PORT = 1883
MQTT_TOPIC_BUTTON = "/button/task"