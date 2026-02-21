#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// ===== WiFi =====
const char* ssid = "YourWifi";
const char* password = "YourWifiPassword";

// ===== MQTT =====
const char* mqtt_server = "broker.emqx.io";
const char* controlTopic   = "car/control";
const char* voltageTopic   = "car/voltage";
const char* tempTopic      = "car/temperature";
const char* ldrTopic       = "car/ldr";
const char* ledTopic       = "car/led";
const char* ledStatusTopic = "car/led/status";
const char* buzzerTopic    = "car/buzzer";
const char* distanceTopic  = "car/distance";

// ===== MOTOR PINS =====
const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int enable1Pin = 14;

const int motor2Pin1 = 25;
const int motor2Pin2 = 23;
const int enable2Pin = 13;

// ===== VOLTAGE SENSOR =====
const int voltagePin = 35;
const float dividerRatio = 5.0;

// ===== DHT11 =====
#define DHTPIN 22
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ===== LDR + LED + MIRROR =====
const int ldrPin = 34;
const int ledPin = 21;
const int mirrorPin = 19;
const int ldrThreshold = 1600;
bool ledManualControl = false;

// ===== BUZZER =====
const int buzzerPin = 18;

// ===== ULTRASONIC =====
const int trigPin = 4;
const int echoPin = 5;
const float stopDistance = 7.0;   // cm
bool obstacleDetected = false;

// ===== PWM =====
const int freq = 30000;
const int resolution = 8;
int speedMotor = 200;

// ===== STATE =====
bool movingForward = false;

WiFiClient espClient;
PubSubClient client(espClient);

// ===== MOTOR FUNCTIONS =====
void stopMotor() {
  movingForward = false;
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  ledcWrite(enable1Pin, 0);
  ledcWrite(enable2Pin, 0);
}

void moveForward() {
  movingForward = true;
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  ledcWrite(enable1Pin, speedMotor);
  ledcWrite(enable2Pin, speedMotor);
}

void moveBackward() {
  movingForward = false;
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  ledcWrite(enable1Pin, speedMotor);
  ledcWrite(enable2Pin, speedMotor);
}

void turnLeft() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  ledcWrite(enable1Pin, 0);
  ledcWrite(enable2Pin, speedMotor);
}

void turnRight() {
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  ledcWrite(enable2Pin, 0);
  ledcWrite(enable1Pin, speedMotor);
}

// ===== READ VOLTAGE =====
float readBatteryVoltage() {
  int sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(voltagePin);
    delay(2);
  }
  float adc = sum / 10.0;
  float voltageAtPin = (adc / 4095.0) * 3.3;
  return voltageAtPin * dividerRatio;
}

// ===== ULTRASONIC READ =====
float readDistanceCM() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return -1;

  float distance = duration * 0.034 / 2.0;
  return distance;
}

// ===== MQTT CALLBACK =====
void callback(char* topic, byte* message, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)message[i];

  if (String(topic) == controlTopic) {
    if (msg == "forward") moveForward();
    else if (msg == "backward") moveBackward();
    else if (msg == "left") turnLeft();
    else if (msg == "right") turnRight();
    else if (msg == "stop") stopMotor();
  }

  if (String(topic) == ledTopic) {
    ledManualControl = true;
    if (msg == "on") {
      digitalWrite(ledPin, HIGH);
      digitalWrite(mirrorPin, HIGH);
    }
    else if (msg == "off") {
      digitalWrite(ledPin, LOW);
      digitalWrite(mirrorPin, LOW);
    }
    else if (msg == "auto") {
      ledManualControl = false;
    }
  }

  if (String(topic) == buzzerTopic) {
    if (msg == "on") digitalWrite(buzzerPin, HIGH);
    else if (msg == "off") digitalWrite(buzzerPin, LOW);
  }
}

// ===== WIFI =====
void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

// ===== MQTT =====
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32_CAR")) {
      client.subscribe(controlTopic);
      client.subscribe(ledTopic);
      client.subscribe(buzzerTopic);  
    } else delay(2000);
  }
}

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // LED / Mirror / Buzzer
  pinMode(ledPin, OUTPUT);
  pinMode(mirrorPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  digitalWrite(mirrorPin, LOW);
  digitalWrite(buzzerPin, LOW);

  // Ultrasonic pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // PWM
  ledcAttach(enable1Pin, freq, resolution);
  ledcAttach(enable2Pin, freq, resolution);
  stopMotor();

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  dht.begin();
  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  // ===== ULTRASONIC SAFETY =====
  float distance = readDistanceCM();
  if (movingForward && distance > 0 && distance <= stopDistance) {
    stopMotor();
    digitalWrite(buzzerPin, HIGH);
    obstacleDetected = true;
  } else {
    if (obstacleDetected) {
      digitalWrite(buzzerPin, LOW);
      obstacleDetected = false;
    }
  }

  // ===== PUBLISH DATA EVERY 2 SECONDS =====
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 2000) {

    // Voltage
    float voltage = readBatteryVoltage();
    char vBuf[8];
    dtostrf(voltage, 4, 2, vBuf);
    client.publish(voltageTopic, vBuf);

    // Temperature
    float temp = dht.readTemperature();
    if (!isnan(temp)) {
      char tBuf[8];
      dtostrf(temp, 4, 1, tBuf);
      client.publish(tempTopic, tBuf);
    }

    // LDR
    int ldrValue = analogRead(ldrPin);
    char ldrBuf[8];
    itoa(ldrValue, ldrBuf, 10);
    client.publish(ldrTopic, ldrBuf);

    // LED AUTO
    if (!ledManualControl) {
      bool ledState = (ldrValue > ldrThreshold);
      digitalWrite(ledPin, ledState);
      digitalWrite(mirrorPin, ledState);
    }
    client.publish(ledStatusTopic, digitalRead(ledPin) ? "ON" : "OFF");

    // Distance
    if (distance > 0) {
      char dBuf[8];
      dtostrf(distance, 4, 1, dBuf);
      client.publish(distanceTopic, dBuf);
    }

    lastSend = millis();
  }
}
