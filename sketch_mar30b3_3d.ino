
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>

// Wi-Fi Credentials
const char* WIFI_SSID = "OPTUS_FF9A94L";
const char* WIFI_PASS = "hazel20212qv";

// MQTT Broker Configuration
const char* MQTT_BROKER = "broker.emqx.io";
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC = "SIT210/wave";

// ultrasonic Sensor and LED Pins
const int TRIG_PIN = 2;
const int ECHO_PIN = 3;
const int LED_PIN = 4;

// Wi-Fi & MQTT Clients
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// Timer for debounce
unsigned long lastEventTime = 0;
const unsigned long MIN_INTERVAL = 3000; // 3 seconds

void setup() {
    Serial.begin(115200);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    
    connectToWiFi();
    connectToMQTT();
}

void loop() {
    mqttClient.poll(); // Listen for incoming messages

    float distance = measureDistance();
    unsigned long currentTime = millis();

    if (distance > 0 && distance <= 20 && (currentTime - lastEventTime > MIN_INTERVAL)) {
        lastEventTime = currentTime;
        sendMQTTMessage(distance);
    }
}

void connectToWiFi() {
    Serial.print("Connecting to Wi-Fi: ");
    Serial.println(WIFI_SSID);
    while (WiFi.begin(WIFI_SSID, WIFI_PASS) != WL_CONNECTED) {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("\nWi-Fi Connected!");
}

void connectToMQTT() {
    Serial.print("Connecting to MQTT Broker: ");
    Serial.println(MQTT_BROKER);
    while (!mqttClient.connect(MQTT_BROKER, MQTT_PORT)) {
        Serial.print("Connection failed, retrying...");
        delay(3000);
    }
    Serial.println("Connected to MQTT!");
    
    mqttClient.onMessage(handleMQTTMessage);
    mqttClient.subscribe(MQTT_TOPIC);
    Serial.println("Subscribed to topic: " + String(MQTT_TOPIC));
}

float measureDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    return (duration * 0.0343) / 2; // Convert time to distance in cm
}

void sendMQTTMessage(float distance) {
    String message;
    if (distance > 10) {
        message = "Jasveena waved";
    } else {
        message = "PAT";
    }
    Serial.println("Publishing: " + message);
    
    mqttClient.beginMessage(MQTT_TOPIC);
    mqttClient.print(message);
    mqttClient.endMessage();
}

void handleMQTTMessage(int messageSize) {
    Serial.print("Received: ");
    String receivedMessage = "";
    while (mqttClient.available()) {
        receivedMessage += (char)mqttClient.read();
    }
    Serial.println(receivedMessage);
    
    if (receivedMessage.indexOf("Jasveena waved") != -1) {
        blinkLED(3, 400); // 3 slow blinks
    } else if (receivedMessage.indexOf("PAT") != -1) {
        blinkLED(6, 150); // 6 fast blinks
    }
}

void blinkLED(int times, int duration) {
    for (int i = 0; i < times; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(duration);
        digitalWrite(LED_PIN, LOW);
        delay(duration);
    }
}
