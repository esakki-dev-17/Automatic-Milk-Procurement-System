#include <SPI.h>
#include <MFRC522.h>
#include <LiquidCrystal_I2C.h>
#include <ultrasonic.h>
#include <ESP32Servo.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#define MQTT_VERSION_3_1     3
#define MQTT_VERSION_3_1_1   4

#ifndef MQTT_VERSION
#define MQTT_VERSION MQTT_VERSION_3_1_1
#endif

#define FLOW_DETECTOR 13
#define PUMP          12
#define IN1           26
#define IN2           25
#define IN3           33
#define IN4           32
#define SS_PIN        5
#define RST_PIN       15
#define DOOR_SERVO    14
#define CLEANING_SERVO 27
#define FLOW_SENSOR   34

#define STOP_DIST  3.4
#define START_DIST 3.8

const char* ssid        = "balG";
const char* password    = "tryagain001";
const char* mqtt_server = "10.108.154.166";

WiFiClient espClient;
PubSubClient client(espClient);

const char* clientId = "esp32";
const char* userName = "tryagain001";
const char* Pass     = "#Tryagain001#";

const char* ntpServer           = "pool.ntp.org";
const long  gmtOffset_sec       = 19800;
const int   daylightOffset_sec  = 0;

#define MSG_BUFFER_SIZE 256
char msg[MSG_BUFFER_SIZE];

StaticJsonDocument<128> doc;

float volume    = 0.0;
float fat_value = 0.0;
float snf_value = 0.0;

char serialBuffer[30];
int serialIndex = 0;

int pulse     = 0;
int time1     = 0;
int fwd_steps = 0;

int stepDelay = 3;

int stepSequence[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

Servo door_servo;
Servo cleaning_servo;

MFRC522 rfid(SS_PIN, RST_PIN);
LiquidCrystal_I2C lcd(0x27, 16, 2);

const String validUIDs[] = {
  "03927731",
  "A3891F31",
  "7326FE30",
  "13727B31"
};

const int UID_COUNT = sizeof(validUIDs) / sizeof(validUIDs[0]);

static bool process_finished = false;
static bool motorRunning     = false;
static bool isCleaned        = false;

volatile bool flow      = false;
volatile bool milk_data = false;

int getLocalHour() {
  struct tm timeinfo;

  if (!getLocalTime(&timeinfo)) {
    return -1;
  }

  int hour12 = (timeinfo.tm_hour % 12 == 0) ? 12 : timeinfo.tm_hour % 12;
  return hour12;
}

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect(clientId, userName, Pass)) {
    } else {
      delay(5000);
    }
  }
}

void setStep(int s) {
  digitalWrite(IN1, stepSequence[s][0]);
  digitalWrite(IN2, stepSequence[s][1]);
  digitalWrite(IN3, stepSequence[s][2]);
  digitalWrite(IN4, stepSequence[s][3]);
}

void rotateSteps(int steps) {
  if (steps == 0 || fwd_steps > 1650) {
    return;
  }

  if (steps > 0) fwd_steps += steps;

  int dir = (steps > 0) ? 1 : -1;
  steps   = abs(steps);

  for (int i = 0; i < steps; i++) {
    for (int s = 0; s < 8; s++) {
      int idx = (dir == 1) ? s : (7 - s);
      setStep(idx);
      delay(stepDelay);
    }
  }
}

void clean() {
  cleaning_servo.write(160);
  int cleaning_time = millis();

  digitalWrite(PUMP, HIGH); //Turning ON Pump 

  while (millis() - cleaning_time <= 60000) {
    float distance = read_ultrasonic_sensor();

    if (!motorRunning && distance > START_DIST) {
      motorRunning = true;
    }

    if (motorRunning && distance < STOP_DIST) {
      motorRunning = false;
    }

    if (motorRunning) rotateSteps(20);
    else rotateSteps(0);
  }

  digitalWrite(PUMP, LOW);

  fwd_steps = 0;
  rotateSteps(-1650);

  cleaning_servo.write(0);
  isCleaned = true;
}

bool isValid(String tag) {
  for (int i = 0; i < UID_COUNT; i++) {
    if (tag == validUIDs[i]) {
      return true;
    }
  }
  return false;
}

void readAnalyzerSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      serialBuffer[serialIndex] = '\0';

      sscanf(serialBuffer, "%f,%f", &fat_value, &snf_value);

      serialIndex = 0;
      milk_data   = true;
    } else {
      if (serialIndex < sizeof(serialBuffer) - 1) {
        serialBuffer[serialIndex++] = c;
      }
    }
  }
}

void door(int angle) {
  if (angle == 0) {
    for (int i = 99; i >= 0; i--) {
      door_servo.write(i);
      delay(5);
    }
  } else if (angle == 100) {
    for (int i = 1; i <= 100; i++) {
      door_servo.write(i);
      delay(5);
    }
  }
}

void pulseCount() {
  if (flow) {
    pulse++;
  }
}

void flow_detection() {
  flow = true;
}

void setup() {
  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  Serial.begin(9600);

  setup_ultrasonic();

  pinMode(FLOW_SENSOR, INPUT);
  pinMode(PUMP, OUTPUT);
  pinMode(FLOW_DETECTOR, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR), pulseCount, RISING);
  attachInterrupt(digitalPinToInterrupt(FLOW_DETECTOR), flow_detection, RISING);

  door_servo.attach(DOOR_SERVO);
  cleaning_servo.attach(CLEANING_SERVO);

  SPI.begin();
  rfid.PCD_Init();
  rfid.PCD_SetAntennaGain(rfid.RxGain_max);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Place your card");
}

void loop() {
  if (getLocalHour() >= 6 && getLocalHour() <= 7) {
    isCleaned = false;

    if (!rfid.PICC_IsNewCardPresent()) return;
    if (!rfid.PICC_ReadCardSerial()) return;

    String uid = "";

    for (byte i = 0; i < rfid.uid.size; i++) {
      if (rfid.uid.uidByte[i] < 0x10) uid += "0";
      uid += String(rfid.uid.uidByte[i], HEX);
    }

    uid.toUpperCase();

    if (isValid(uid)) {
      lcd.setCursor(0, 0);
      door(0);

      lcd.print("Access Granted");
      lcd.setCursor(0, 1);
      lcd.print("ID: ");
      lcd.print(uid);

      delay(1000);

      process_finished = false;
      milk_data        = false;
      pulse            = 0;
      time1            = 0;

      digitalWrite(PUMP, HIGH);

      while (!process_finished) {
        if (flow && digitalRead(FLOW_DETECTOR) == LOW) {
          if (time1 == 0) {
            time1 = millis();
          } else if (millis() - time1 >= 2000) {
            digitalWrite(PUMP, LOW);
            process_finished = true;
          }
        }

        readAnalyzerSerial();

        lcd.clear();

        float distance = read_ultrasonic_sensor();

        lcd.setCursor(0, 0);
        lcd.print("Descending hose:");

        lcd.setCursor(0, 1);
        lcd.print(distance, 1);

        if (!motorRunning && distance > START_DIST) {
          motorRunning = true;
        }

        if (motorRunning && distance < STOP_DIST) {
          motorRunning = false;
        }

        if (motorRunning) rotateSteps(20);
        else rotateSteps(0);
      }

      fwd_steps = 0;
      rotateSteps(-1650);

      door(100);

      volume = pulse / 540;

      while (milk_data != true) {
        readAnalyzerSerial();
      }

      lcd.setCursor(0, 0);
      lcd.print("Vol   Fat   SNF");

      lcd.setCursor(0, 1);
      lcd.print(volume, 1);

      lcd.setCursor(6, 1);
      lcd.print(fat_value);

      lcd.setCursor(12, 1);
      lcd.print(snf_value, 1);

      doc["vol"] = volume;
      doc["fat"] = fat_value;
      doc["snf"] = snf_value;

      size_t n = serializeJson(doc, msg);

      if (!client.connected()) {
        reconnect();
      }

      String topic = "/milkBooth1/" + uid + "/data";
      client.publish(topic.c_str(), msg, n);
    } else {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Access Denied");
    }

    client.loop();

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();

    delay(3000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Place your card");
  } else if (!isCleaned && getLocalHour() > 7) {
    clean();
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Automatic Milk");

    lcd.setCursor(0, 1);
    lcd.print("Proc System");
  }
}
