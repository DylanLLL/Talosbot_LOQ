#include "config.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setup_wifi() {
  delay(10);
  
  // Connect to wifi network
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
    lcd_connecting();
    try_connect++;
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  delay(30);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connected!");
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

  // 1ST BUTTON FEEDBACK
  if (String(topic) == feedback_button1_topic) {
    if (strcmp(terminated_payload, "in_progress") == 0) {
      Serial.println("Button 1: Behavior tree is in progress.");
      delay(30);
      lcd_print("Kirim Troli"); // Indicate in-progress state
      lcd_print_row2("Ke BES");
    } 
    else if (strcmp(terminated_payload, "success") == 0) {
      Serial.println("Button 1: Behavior tree succeeded.");
      delay(30);
      lcd_print("Troli Terkirim");
      task_status = false;
    } 
    else if (strcmp(terminated_payload, "failed") == 0) {
      Serial.println("Button 1: Behavior tree failed.");
      delay(30);
      lcd_print("Tugas Gagal");
      task_status = false;
    }
  }

  // 2ND BUTTON FEEDBACK
  if (String(topic) == feedback_button2_topic) {
    if (strcmp(terminated_payload, "in_progress") == 0) {
      Serial.println("Button 2: Behavior tree is in progress.");
      delay(30);
      lcd_print("Jemput Troli");
      lcd_print_row2("Kosong");
    } 
    else if (strcmp(terminated_payload, "success") == 0) {
      Serial.println("Button 2: Behavior tree succeeded.");
      delay(30);
      lcd_print("Troli Kosong");
      lcd_print_row2("Sampai");
      task_status = false;
    } 
    else if (strcmp(terminated_payload, "failed") == 0) {
      Serial.println("Button 2: Behavior tree failed.");
      delay(30);
      lcd_print("Tugas Gagal");
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
      delay(1000);
      lcd_print("AMR Ready!");  
    } 
    else if (strcmp(terminated_payload, "failed") == 0) {
      Serial.println("Task failed");
      task_status = false;
      delay(1000);
      lcd_print_row2("Coba Lagi!"); 
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
      client.subscribe(button1_topic.c_str(), 0);
      client.subscribe(button2_topic.c_str(), 0);
      client.subscribe(feedback_button1_topic.c_str(), 0); // Subscribe to feedback topics
      client.subscribe(feedback_button2_topic.c_str(), 0);
      client.subscribe(feedback_task_status.c_str(), 0);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 seconds");
      lcd_connecting();
      delay(100);
    }
  }
}

void lcd_connecting(){
  lcd.init();
  lcd_print("Connecting...");
  delay(750);
  lcd_print("   ");
  delay(100);
}


void setup() {
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Calling Button");
  lcd.setCursor(0, 1);
  lcd.print("CHUTE");

  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP); 

  Serial.begin(115200);

  setup_wifi();
  delay(1000);

  lcd_print("AMR Ready!");

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();
  bool button1_read = digitalRead(BTN1_PIN);
  bool button2_read = digitalRead(BTN2_PIN);

  if(button1_read != prev_button1_read && task_status == false){
    if(millis() - prev_button1_press > debounce_time){
      if(!button1_read){
        String msg = String(1);
        //String msg = String(1.0) + "," + String(2.0) + "," + String(0.000);
        client.publish("/button1/task", msg.c_str());
      }
      prev_button1_read = button1_read;
      prev_button1_press = millis();
    }
  }

  if(button2_read != prev_button2_read && task_status == false){
    if(millis() - prev_button2_press > debounce_time){
      if(!button2_read){
        String msg = String(2);
        //String msg = String(1.0) + "," + String(2.0) + "," + String(0.000);
        client.publish("/button2/task", msg.c_str());
      }
      prev_button2_read = button2_read;
      prev_button2_press = millis();
    }
  }
} 

void lcd_print(String message){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message);
}

void lcd_print_row2(String message){
  lcd.setCursor(0, 1);
  lcd.print(message);
}
