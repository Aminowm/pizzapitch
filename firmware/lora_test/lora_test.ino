/*
  ======================================================================
   ПРОВЕРКА LoRa  —  ОДИН И ТОТ ЖЕ файл заливается в ОБЕ платы LilyGO
  ======================================================================
   Что происходит:
     Каждая плата раз в ~2 секунды "кричит" в эфир короткое сообщение
     PING и всё остальное время слушает, не кричит ли другая плата.

   Что видно на маленьком экране каждой платы:
     - имя платы (4 символа, у каждой своё)
     - LoRa: OK      -> радиомодуль найден и включён
     - Otpravleno: N -> сколько раз эта плата крикнула
     - Prinyato:   M -> сколько раз она УСЛЫШАЛА другую плату
     - RSSI          -> сила сигнала (чем ближе к нулю, тем сильнее)
     - внизу крупно: ZHDU... / SVYAZ OK / PROPALA

   Если на ОБЕИХ платах "Prinyato" растёт и написано SVYAZ OK —
   LoRa работает, можно переходить к роботу.

   ЕСЛИ ЭКРАН СЛОМАН — смотри на маленький СВЕТОДИОД на плате:
     - быстро-быстро мигает всё время  -> радиомодуль НЕ найден (ошибка)
     - не горит                        -> модуль ОК, но другую плату не слышит
     - вспыхивает примерно раз в 2 сек -> СЛЫШИТ другую плату = СВЯЗЬ ЕСТЬ
   Сломанный экран программе не мешает: если экрана нет, она его пропускает.

   Нужны библиотеки: LoRa (Sandeep Mistry), Adafruit SSD1306, Adafruit GFX.
   Плата в Arduino IDE: esp32 -> TTGO LoRa32-OLED.
  ======================================================================
*/

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- НАСТРОЙКИ (менять только если не работает) ----------

// Частота. На чипе написано 868/915 MHz -> в Казахстане берём 868.
const long FREQ = 868E6;

// Версия платы. 1 = почти все современные LilyGO (T3 v1.6).
// Если экранчик остаётся чёрным — поставь 0 и залей ещё раз.
#define BOARD_V2 1

// ----------------------------------------------------------------

// Какие ножки ESP32 соединены с экраном и радиомодулем (менять не нужно)
#if BOARD_V2
const int PIN_OLED_SDA = 21, PIN_OLED_SCL = 22, PIN_OLED_RST = -1, PIN_LORA_RST = 23;
#else
const int PIN_OLED_SDA = 4,  PIN_OLED_SCL = 15, PIN_OLED_RST = 16, PIN_LORA_RST = 14;
#endif
const int PIN_LORA_SCK = 5, PIN_LORA_MISO = 19, PIN_LORA_MOSI = 27, PIN_LORA_SS = 18, PIN_LORA_DIO0 = 26;
const int PIN_LED = 25;   // маленький светодиод на плате, мигает при приёме

Adafruit_SSD1306 oled(128, 64, &Wire, PIN_OLED_RST);

String myName;                     // имя этой платы (из её заводского номера)
bool   loraOk = false;             // нашёлся ли радиомодуль
bool   oledOk = false;             // живой ли экранчик
unsigned long sent = 0, got = 0;   // счётчики: отправлено / принято
unsigned long nextSend = 0, lastGot = 0, lastScreen = 0;
int    lastRssi = 0;
float  lastSnr = 0;
String lastFrom = "----";

void setup() {
  Serial.begin(115200);
  delay(300);
  pinMode(PIN_LED, OUTPUT);

  // У каждой ESP32 свой заводской номер — берём из него 4 символа как имя
  uint64_t mac = ESP.getEfuseMac();
  char buf[8];
  snprintf(buf, sizeof(buf), "%04X", (unsigned int)((mac >> 32) & 0xFFFF));
  myName = buf;
  randomSeed((unsigned long)mac);

  // Включаем экранчик
  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (PIN_OLED_RST >= 0) { pinMode(PIN_OLED_RST, OUTPUT); digitalWrite(PIN_OLED_RST, LOW); delay(20); digitalWrite(PIN_OLED_RST, HIGH); }
  Wire.setTimeOut(20);
  Wire.beginTransmission(0x3C);           // "экран, ты тут?"
  oledOk = (Wire.endTransmission() == 0); // ответил -> экран живой
  if (oledOk) { oled.begin(SSD1306_SWITCHCAPVCC, 0x3C); oled.clearDisplay(); oled.display(); }

  // Включаем радиомодуль LoRa
  SPI.begin(PIN_LORA_SCK, PIN_LORA_MISO, PIN_LORA_MOSI, PIN_LORA_SS);
  LoRa.setPins(PIN_LORA_SS, PIN_LORA_RST, PIN_LORA_DIO0);
  loraOk = LoRa.begin(FREQ);
  if (loraOk) {
    // Эти настройки ДОЛЖНЫ совпадать с настройками робота и базы
    LoRa.setSpreadingFactor(8);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setSyncWord(0x42);
    LoRa.setTxPower(17);
    LoRa.enableCrc();
  }

  Serial.println();
  Serial.println("=== ПРОВЕРКА LoRa ===");
  Serial.print("Имя этой платы: "); Serial.println(myName);
  Serial.println(oledOk ? "Экран: найден" : "Экран: НЕ найден (не страшно, смотри на светодиод)");
  Serial.println(loraOk ? "LoRa: OK, модуль найден" : "LoRa: ОШИБКА — модуль не найден (проверь BOARD_V2 и выбор платы)");
}

void loop() {
  unsigned long now = millis();

  // 1) Раз в ~2 секунды отправляем PING (со случайной добавкой,
  //    чтобы две платы не кричали одновременно)
  if (loraOk && now >= nextSend) {
    LoRa.beginPacket();
    LoRa.print("PING,");
    LoRa.print(myName);
    LoRa.print(",");
    LoRa.print(sent);
    LoRa.endPacket();
    sent++;
    Serial.print("> отправил PING #"); Serial.println(sent);
    nextSend = now + 2000 + random(0, 600);
  }

  // 2) Всё остальное время слушаем эфир
  if (loraOk) {
    int size = LoRa.parsePacket();
    if (size > 0) {
      String msg;
      while (LoRa.available()) msg += (char)LoRa.read();
      // Сообщение вида: PING,ИМЯ,НОМЕР
      if (msg.startsWith("PING,")) {
        int c2 = msg.indexOf(',', 5);
        String from = c2 > 0 ? msg.substring(5, c2) : "?";
        if (from != myName) {                // своё эхо не считаем
          got++;
          lastFrom = from;
          lastRssi = LoRa.packetRssi();
          lastSnr  = LoRa.packetSnr();
          lastGot  = now;
          Serial.print("< услышал плату "); Serial.print(from);
          Serial.print("  сигнал "); Serial.print(lastRssi); Serial.print(" dBm");
          Serial.print("  SNR "); Serial.println(lastSnr, 1);
        }
      }
    }
  }

  // 3) Светодиод:
  //    ошибка LoRa -> мигает быстро всё время
  //    иначе       -> загорается на полсекунды после каждого приёма
  if (!loraOk) digitalWrite(PIN_LED, (now / 100) % 2);
  else         digitalWrite(PIN_LED, (lastGot > 0 && now - lastGot < 500) ? HIGH : LOW);

  // 4) Обновляем экранчик (он не знает русских букв — пишем латиницей)
  if (oledOk && now - lastScreen > 300) {
    lastScreen = now;
    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);  oled.print("LoRa TEST  "); oled.print(myName);
    oled.setCursor(0, 10); oled.print(loraOk ? "LoRa: OK  868MHz" : "LoRa: OSHIBKA!");
    oled.setCursor(0, 20); oled.print("Otpravleno: "); oled.print(sent);
    oled.setCursor(0, 30); oled.print("Prinyato:   "); oled.print(got);
    oled.setCursor(0, 40);
    if (got > 0) { oled.print(lastRssi); oled.print("dBm ot "); oled.print(lastFrom); }
    else oled.print("signala eshe net");
    oled.setTextSize(2);
    oled.setCursor(0, 50 - 2);
    if (!loraOk)                     oled.print("OSHIBKA");
    else if (got == 0)               oled.print("ZHDU...");
    else if (now - lastGot < 6000)   oled.print("SVYAZ OK");
    else                             oled.print("PROPALA");
    oled.display();
  }
}
