/*
  =====================================================================
   БАРЛАУШЫ — LilyGO LoRa32 НА РОВЕРЕ (мост Mega <-> LoRa)
  ---------------------------------------------------------------------
   Mega (Serial1) --строки $T,...--> LilyGO --LoRa--> База
   База --LoRa "C,id,CMD"--> LilyGO --"$C,id,CMD"--> Mega

   Плата в Arduino IDE: "TTGO LoRa32-OLED" (пакет esp32 by Espressif).
   Библиотеки: "LoRa" by Sandeep Mistry, "Adafruit SSD1306", "Adafruit GFX".

   ВАЖНО: частота LORA_FREQ должна совпадать с наклейкой на плате
   (433 / 868 / 915 МГц) и с прошивкой базы.
  =====================================================================
*/

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---- Версия платы ----
// T3 V1.6 / LoRa32 V2.1 (самые частые, OLED снизу у антенны): 1
// Старая LoRa32 V1.0 (OLED с ножкой RST=16): 0
#define BOARD_V2 1
#if BOARD_V2
const int PIN_OLED_SDA = 21, PIN_OLED_SCL = 22, PIN_OLED_RST = -1, PIN_LORA_RST = 23;
#else
const int PIN_OLED_SDA = 4,  PIN_OLED_SCL = 15, PIN_OLED_RST = 16, PIN_LORA_RST = 14;
#endif
const int PIN_LORA_SCK = 5, PIN_LORA_MISO = 19, PIN_LORA_MOSI = 27, PIN_LORA_SS = 18, PIN_LORA_DIO0 = 26;

const long LORA_FREQ = 868E6;     // модуль 868/915 МГц. В Казахстане используем 868 МГц
const int  LORA_SF   = 8;
const uint8_t LORA_SYNC = 0x42;   // "свой" канал, чтобы не ловить чужие LoRa

// ---- UART к Mega ----
const int MEGA_RX = 34;           // сюда: Mega TX1 (пин 18) ЧЕРЕЗ делитель 5В -> 3.3В
const int MEGA_TX = 13;           // отсюда: на Mega RX1 (пин 19) напрямую
const long MEGA_BAUD = 57600;

Adafruit_SSD1306 oled(128, 64, &Wire, PIN_OLED_RST);

String megaBuf;
String lastState = "--";
int    lastOverall = 0;
String lastCmd = "--";
int    lastCmdRssi = 0;
unsigned long seq = 0;
unsigned long lastMegaMs = 0, lastTxMs = 0, lastOled = 0;
bool   loraOk = false;

// поле номер n (с 0) в строке CSV
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

void sendLora(const String &payload) {
  if (!loraOk) return;
  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();          // блокирует ~100–200 мс, это нормально
  lastTxMs = millis();
}

// строка от Mega: $T,STATE,dist,...
void handleMegaLine(const String &line) {
  if (!line.startsWith("$T,")) return;
  lastMegaMs = millis();
  String payload = line.substring(3);
  lastState   = field(payload, 0);
  lastOverall = field(payload, 10).toInt();
  seq++;
  sendLora("T," + String(seq) + "," + payload);
}

void checkLora() {
  int size = LoRa.parsePacket();
  if (size == 0) return;
  String pkt;
  while (LoRa.available()) pkt += (char)LoRa.read();
  if (pkt.startsWith("C,")) {                 // C,<id>,<CMD>
    Serial2.print('$');
    Serial2.println(pkt);
    lastCmd = field(pkt, 2);
    lastCmdRssi = LoRa.packetRssi();
    Serial.print("CMD from base: "); Serial.println(pkt);
  }
}

void drawOled() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 0);  oled.print("BARLAUSHY  ROVER");
  oled.setCursor(0, 12);
  bool megaAlive = millis() - lastMegaMs < 3000;
  oled.print("Mega: "); oled.print(megaAlive ? "OK" : "NET SVYAZI");
  oled.setCursor(0, 22);
  oled.print("LoRa: "); oled.print(loraOk ? "TX #" : "OSHIBKA"); if (loraOk) oled.print(seq);
  oled.setCursor(0, 32);
  oled.print("CMD: "); oled.print(lastCmd); oled.print(" "); oled.print(lastCmdRssi); oled.print("dBm");
  oled.setTextSize(2);
  oled.setCursor(0, 46);
  oled.print(lastOverall == 2 ? "OPASNO!" : lastOverall == 1 ? "VNIMANIE" : lastState.c_str());
  oled.display();
}

void setup() {
  Serial.begin(115200);
  Serial2.setRxBufferSize(1024);   // пока LoRa передаёт, строки от Mega копятся здесь
  Serial2.begin(MEGA_BAUD, SERIAL_8N1, MEGA_RX, MEGA_TX);

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (PIN_OLED_RST >= 0) { pinMode(PIN_OLED_RST, OUTPUT); digitalWrite(PIN_OLED_RST, LOW); delay(20); digitalWrite(PIN_OLED_RST, HIGH); }
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay(); oled.display();

  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_SS);
  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);
  loraOk = LoRa.begin(LORA_FREQ);
  if (loraOk) {
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setSyncWord(LORA_SYNC);
    LoRa.setTxPower(17);
    LoRa.enableCrc();
  }
  Serial.println(loraOk ? "LoRa OK" : "LoRa INIT FAILED");
}

void loop() {
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') { handleMegaLine(megaBuf); megaBuf = ""; }
    else if (c != '\r' && megaBuf.length() < 160) megaBuf += c;
  }
  checkLora();
  if (millis() - lastOled > 400) { lastOled = millis(); drawOled(); }
}
