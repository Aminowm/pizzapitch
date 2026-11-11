/*
  =====================================================================
   БАРЛАУШЫ — БАЗОВАЯ СТАНЦИЯ v2 (LilyGO LoRa32)
  ---------------------------------------------------------------------
   Ровер --LoRa--> БАЗА --Wi-Fi (хотспот телефона)--> MQTT-брокер --> сайт
                     └--USB--> ноутбук (локальный пульт, запасной путь)

   В USB:   TEL,<seq>,<STATE>,...,<rssi>,<snr>  (как раньше)
   В MQTT (все сообщения "retained" — сайт сразу видит последнее):
     barlaushy/zhz1407/tel      последняя строка TEL
     barlaushy/zhz1407/mission  все точки замеров текущей миссии (строки TEL через \n)
     barlaushy/zhz1407/status   online / offline (offline ставит сам брокер, если база пропала)
   Из USB "CMD:START" -> LoRa "C,<id>,START" (3 повтора). С сайта команд НЕТ — только просмотр.

   Плата: "TTGO LoRa32-OLED". Библиотеки: LoRa (Sandeep Mistry), Adafruit SSD1306,
   Adafruit GFX, PubSubClient (Nick O'Leary).
  =====================================================================
*/

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ============================ НАСТРОЙКИ ============================

// Wi-Fi: назовите хотспот телефона ровно так (или впишите своё)
const char* WIFI_SSID[2] = { "Barlaushy",     "" };   // [1] — запасная сеть, можно оставить пустой
const char* WIFI_PASS[2] = { "barlaushy2026", "" };

// MQTT: бесплатный публичный брокер. Сайт слушает тот же адрес по WSS (порт 8084)
const char* MQTT_HOST  = "broker.emqx.io";
const int   MQTT_PORT  = 1883;
const char* TOPIC_ROOT = "barlaushy/zhz1407";          // поменяете здесь — поменяйте и в site/js/config.js

#define BOARD_V2 1
#if BOARD_V2
const int OLED_SDA = 21, OLED_SCL = 22, OLED_RST = -1, LORA_RST = 23, LED_PIN = 25;
#else
const int OLED_SDA = 4,  OLED_SCL = 15, OLED_RST = 16, LORA_RST = 14, LED_PIN = 2;
#endif
const int LORA_SCK = 5, LORA_MISO = 19, LORA_MOSI = 27, LORA_SS = 18, LORA_DIO0 = 26;

const long LORA_FREQ = 868E6;     // как на ровере (модуль 868/915 МГц)
const int  LORA_SF   = 8;
const uint8_t LORA_SYNC = 0x42;

const uint8_t CMD_REPEATS = 3;
const unsigned long CMD_GAP_MS = 300;
const int MISSION_MAX_POINTS = 60;

// ========================== КОНЕЦ НАСТРОЕК ==========================

Adafruit_SSD1306 oled(128, 64, &Wire, OLED_RST);
WiFiClient   net;
PubSubClient mqtt(net);

String topicTel, topicMission, topicStatus;
String usbBuf;
bool loraOk = false;

// экран
String stState = "--", prevState = "";
long   stDist = 0, stCh4 = 0, stCo = 0;
int    stOverall = 0, stRssi = 0;
unsigned long lastRxMs = 0, lastLinkMsg = 0, lastOled = 0;
bool   roverLost = false;     // уже сообщили сайту, что ровер пропал

// миссия (для тех, кто откроет сайт посередине)
String missionLines[MISSION_MAX_POINTS];
int    missionCount = 0;

// сеть
int    wifiIdx = 0;
unsigned long wifiStart = 0, mqttTry = 0;

// команда (одна активная)
int    cmdId = 0;
String cmdName;
uint8_t cmdLeft = 0;
unsigned long cmdNext = 0;

String field(const String &s, int n) {
  int start = 0;
  for (int i = 0; i < n; i++) {
    start = s.indexOf(',', start);
    if (start < 0) return "";
    start++;
  }
  int end = s.indexOf(',', start);
  return end < 0 ? s.substring(start) : s.substring(start, end);
}

// ---------------------------- СЕТЬ ----------------------------

void wifiBegin(int idx) {
  wifiIdx = idx;
  wifiStart = millis();
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID[idx], WIFI_PASS[idx]);
  Serial.print("# WiFi -> "); Serial.println(WIFI_SSID[idx]);
}

void publishMission() {
  if (!mqtt.connected()) return;
  String all = "";
  for (int i = 0; i < missionCount; i++) { all += missionLines[i]; all += '\n'; }
  if (all.length() == 0) all = "EMPTY";
  mqtt.publish(topicMission.c_str(), all.c_str(), true);
}

void networkLoop() {
  if (WiFi.status() != WL_CONNECTED) {
    // 15 с не подключились — пробуем другую сеть (если задана)
    if (millis() - wifiStart > 15000) {
      int next = (wifiIdx == 0 && strlen(WIFI_SSID[1]) > 0) ? 1 : 0;
      wifiBegin(next);
    }
    return;
  }
  if (!mqtt.connected()) {
    if (millis() - mqttTry < 8000) return;
    mqttTry = millis();
    String id = "barlaushy-base-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    // "завещание": если база пропадёт, брокер сам опубликует offline
    if (mqtt.connect(id.c_str(), topicStatus.c_str(), 0, true, "offline")) {
      mqtt.publish(topicStatus.c_str(), "online", true);
      publishMission();
      Serial.println("# MQTT connected");
    } else {
      Serial.print("# MQTT fail, rc="); Serial.println(mqtt.state());
    }
    return;
  }
  mqtt.loop();
}

// ---------------------------- LoRa ----------------------------

void checkLora() {
  int size = LoRa.parsePacket();
  if (size == 0) return;
  String pkt;
  while (LoRa.available()) pkt += (char)LoRa.read();
  if (!pkt.startsWith("T,")) return;

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();
  lastRxMs = millis();
  if (roverLost && mqtt.connected()) mqtt.publish(topicStatus.c_str(), "online", true);
  roverLost = false;

  // TEL,seq,STATE,dist,ch4,co,q135,q2,i1,i2,i3,i4,overall,front,pt,ack,bat,warm,rssi,snr
  String line = "TEL," + pkt.substring(2) + "," + String(rssi) + "," + String(snr, 1);
  Serial.println(line);

  prevState = stState;
  stState   = field(pkt, 2);
  stDist    = field(pkt, 3).toInt();
  stCh4     = field(pkt, 4).toInt();
  stCo      = field(pkt, 5).toInt();
  stOverall = field(pkt, 12).toInt();
  stRssi    = rssi;
  bool point = field(pkt, 14).toInt() == 1;

  // новая миссия: SAMPLE на 0 м после READY / DONE / HOLD
  if (stState == "SAMPLE" && stDist == 0 &&
      (prevState == "READY" || prevState == "DONE" || prevState == "HOLD")) {
    missionCount = 0;
    publishMission();
  }
  if (point && missionCount < MISSION_MAX_POINTS) {
    missionLines[missionCount++] = line;
    publishMission();
  }
  if (mqtt.connected()) mqtt.publish(topicTel.c_str(), line.c_str(), true);
}

void checkUsb() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      usbBuf.trim();
      if (usbBuf.startsWith("CMD:")) {
        cmdName = usbBuf.substring(4);
        cmdId = (cmdId % 9999) + 1;
        cmdLeft = CMD_REPEATS;
        cmdNext = 0;
        Serial.print("SENT,"); Serial.print(cmdId); Serial.print(','); Serial.println(cmdName);
      }
      usbBuf = "";
    } else if (c != '\r' && usbBuf.length() < 40) usbBuf += c;
  }
}

void pumpCommand() {
  if (cmdLeft == 0 || millis() < cmdNext || !loraOk) return;
  LoRa.beginPacket();
  LoRa.print("C,"); LoRa.print(cmdId); LoRa.print(','); LoRa.print(cmdName);
  LoRa.endPacket();
  cmdLeft--;
  cmdNext = millis() + CMD_GAP_MS;
}

// ---------------------------- ЭКРАН ----------------------------

void drawOled() {
  bool link = millis() - lastRxMs < 5000 && lastRxMs > 0;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print("BAZA ");
  if (link) { oled.print(stRssi); oled.print("dBm"); }
  else oled.print(loraOk ? "NET ROVERA" : "LoRa ERR");

  oled.setCursor(0, 9);
  oled.print("WiFi:"); oled.print(WiFi.status() == WL_CONNECTED ? "OK" : "--");
  oled.print(" Sait:"); oled.print(mqtt.connected() ? "OK" : "--");

  oled.setTextSize(2);
  oled.setCursor(0, 20);
  oled.print(stOverall == 2 ? "OPASNO!" : stOverall == 1 ? "VNIMANIE" : "NORMA");

  oled.setTextSize(1);
  oled.setCursor(0, 40);
  oled.print(stState); oled.print("  "); oled.print(stDist / 100.0, 1); oled.print(" m");
  oled.setCursor(0, 52);
  oled.print("CH4 "); oled.print(stCh4); oled.print("  CO "); oled.print(stCo);
  oled.display();

  digitalWrite(LED_PIN, (stOverall == 2 && link) ? ((millis() / 200) % 2) : LOW);
}

// ---------------------------- SETUP / LOOP ----------------------------

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  topicTel     = String(TOPIC_ROOT) + "/tel";
  topicMission = String(TOPIC_ROOT) + "/mission";
  topicStatus  = String(TOPIC_ROOT) + "/status";

  Wire.begin(OLED_SDA, OLED_SCL);
  if (OLED_RST >= 0) { pinMode(OLED_RST, OUTPUT); digitalWrite(OLED_RST, LOW); delay(20); digitalWrite(OLED_RST, HIGH); }
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay(); oled.display();

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  loraOk = LoRa.begin(LORA_FREQ);
  if (loraOk) {
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setSyncWord(LORA_SYNC);
    LoRa.setTxPower(17);
    LoRa.enableCrc();
  }
  Serial.println(loraOk ? "BASE,READY" : "BASE,LORA_FAIL");

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  wifiBegin(0);
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(8192);       // вся миссия одним сообщением
  mqtt.setKeepAlive(15);
  mqtt.setSocketTimeout(3);       // чтобы не "зависать" надолго без интернета
}

void loop() {
  checkLora();
  checkUsb();
  pumpCommand();
  networkLoop();

  if (lastRxMs > 0 && millis() - lastRxMs > 5000 && millis() - lastLinkMsg > 2000) {
    lastLinkMsg = millis();
    Serial.println("LINK,0");
    if (!roverLost && mqtt.connected()) mqtt.publish(topicStatus.c_str(), "norover", true);
    roverLost = true;
  }
  if (millis() - lastOled > 300) { lastOled = millis(); drawOled(); }
}
