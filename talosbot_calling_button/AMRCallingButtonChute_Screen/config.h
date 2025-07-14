#define BTN1_PIN 18
#define BTN2_PIN 19
// LCD SDA pin = 21
// LCD SCL pin = 22

#define HOME_WIFI 0
#define WH_WIFI 1
#define HOTSPOT_WIFI 0

#if HOME_WIFI
  const char* ssid = "Myhome918";
  const char* password = "68740109";
  const char* mqtt_server = "192.168.1.183";
#endif

#if HOTSPOT_WIFI
  const char* ssid = "Dylan";
  const char* password = "koenigsegg";
  const char* mqtt_server = "192.168.10.222";
#endif

#if WH_WIFI
  const char* ssid = "Incubus";
  const char* password = "bl1bl1iot";
  const char* mqtt_server = "10.176.164.12";
#endif

//button debouncing
unsigned long prev_button1_press = 0;
bool prev_button1_read = true;
unsigned long prev_button2_press = 0;
bool prev_button2_read = true;
const int debounce_time = 250;

//MQTT topic related
String button1_topic = "/button1/task";
String button2_topic = "/button2/task";
String feedback_button1_topic = "/button1/status";
String feedback_button2_topic = "/button2/status";
String feedback_task_status = "/task/status";
bool task_status = false;