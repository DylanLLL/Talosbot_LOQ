#define LED1_PIN 25 
#define BTN1_PIN 19

#define HOME_WIFI 0
#define HOTSPOT_WIFI 0
#define WH_WIFI 1

#if HOME_WIFI
  const char* ssid = "Myhome918";
  const char* password = "68740109";
  const char* mqtt_server = "192.168.1.183";
#endif

#if HOTSPOT_WIFI
  const char* ssid = "Dylan";
  const char* password = "koenigsegg";
  const char* mqtt_server = "192.168.60.222";
#endif

#if WH_WIFI
  const char* ssid = "Incubus";
  const char* password = "bl1bl1iot";
  const char* mqtt_server = "10.176.164.12";
#endif

unsigned long prev_button3_press = 0;
bool prev_button3_read = true;

#include <WiFi.h>
#include <PubSubClient.h>

String button3_topic = "/button3/task";
String feedback_button3_topic = "/button3/status";
String feedback_task_status = "/task/status";
bool task_status = false;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);

  int try_connect = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    if (try_connect == 0){
      WiFi.begin(ssid, password);
    }
    if (try_connect == 25){
      ESP.restart();
    }
    Serial.println("...");
    led_blink();
    try_connect++;
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// this callback function is like a FEEDBACK that you RECEIVED from the PC
void callback(char* topic, byte* payload, unsigned int length) {

  unsigned long currentMillis = millis();
  Serial.printf("[%lu ms] Received Topic: %s\n", currentMillis, topic);
  Serial.printf("[%lu ms] Received Payload: ", currentMillis);

  //Payload null terminator (\0)
  char terminated_payload[length + 1];
  memcpy(terminated_payload, payload, length);
  terminated_payload[length] = '\0';
  Serial.println(terminated_payload);

  // if (String(topic) == feedback_button1_topic || String(topic) == feedback_button2_topic) {
  //     Serial.printf("Handling feedback for %s\n", topic);
  // }

  // //Handle feedback messages
  if (String(topic) == feedback_button3_topic) {
    if (strcmp(terminated_payload, "in_progress") == 0) {
      Serial.println("Button 3: Behavior tree is in progress.");
      digitalWrite(LED1_PIN, HIGH); // Indicate in-progress state
    } 
    else if (strcmp(terminated_payload, "success") == 0) {
      Serial.println("Button 3: Behavior tree succeeded.");
      digitalWrite(LED1_PIN, LOW); // Turn off LED1
      task_status = false; 
    } 
    else if (strcmp(terminated_payload, "failed") == 0) {
      Serial.println("Button 3: Behavior tree failed.");
      task_status = false;
    }
  }

  if (String(topic) == feedback_task_status){
    if (strcmp(terminated_payload, "in_progress") == 0) {
      Serial.println("Task is in progress");
      task_status = true;
    } 
    else if (strcmp(terminated_payload, "success") == 0) {
      Serial.println("Task finished");
      task_status = false;
    } 
    else if (strcmp(terminated_payload, "failed") == 0) {
      Serial.println("Task failed");
      task_status = false;
    }
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "UniqueMQTTClientID";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      client.subscribe(button3_topic.c_str(), 0);
      client.subscribe(feedback_button3_topic.c_str(), 0); // Subscribe to feedback topics
      client.subscribe(feedback_task_status.c_str(), 0);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 seconds");
      led_blink();
      delay(1000);
    }
  }
}

void led_blink(){
  digitalWrite(LED1_PIN, HIGH);
  delay(250);
  digitalWrite(LED1_PIN, LOW);
  delay(250);
}


void setup() {
  pinMode(LED1_PIN, OUTPUT);
  pinMode(BTN1_PIN, INPUT_PULLUP);

  Serial.begin(115200);

  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  bool button3_read = digitalRead(BTN1_PIN);

  if(button3_read != prev_button3_read && task_status == false){
    if(millis() - prev_button3_press > 250){
      if(!button3_read){
        String msg = String(3);
        //String msg = String(1.0) + "," + String(2.0) + "," + String(0.000);
        client.publish("/button3/task", msg.c_str());
      }
      prev_button3_read = button3_read;
      prev_button3_press = millis();
    }
  }
} 
