/*
  =====================================================================
   БАРЛАУШЫ — робот-разведчик газа (прототип для Pizza Pitch)
   Плата: Arduino Mega 2560
  ---------------------------------------------------------------------
   Что делает:
   1. Прогревает MQ-датчики, затем калибруется в ЧИСТОМ воздухе.
   2. По команде START едет вперёд шагами по 50 см, на каждом шаге
      останавливается и делает замер (точка на "карте газа").
   3. HC-SR04 на серво: видит препятствие -> смотрит влево/вправо ->
      объезжает.
   4. Опасный газ -> стоп, сирена, отъезд назад, ждёт команды человека.
   5. Каждую секунду шлёт телеметрию в LilyGO (Serial1) -> LoRa -> база.

   Библиотеки: Servo (встроена) + LiquidCrystal I2C (Frank de Brabander) —
   поставить через Менеджер библиотек.

   ПИНЫ (под фактическую сборку робота):
     L298N:  ENA=D5  IN1=D7  IN2=D8  IN3=D9  IN4=D10  ENB=D6
     MQ-4=A0  MQ-7=A1  MQ-135=A2  MQ-2=A3
     Серво=D11   HC-SR04: TRIG=D12  ECHO=D13   Буззер=D3
     LCD1602 I2C: SDA=20  SCL=21
     LilyGO: TX1 (18) -> делитель -> GPIO34,  LilyGO GPIO13 -> RX1 (19)
   Если серво воткнут в D4 — поменяй PIN_SERVO на 4.
   Если буззер воткнут в D22 — поменяй PIN_BUZZER на 22.

   Отладка без LilyGO: открой Монитор порта (115200, "Новая строка")
   и вводи команды: START, STOP, RESUME, RECAL, BEEP, MAN_F, MAN_B,
   MAN_L, MAN_R.
  =====================================================================
*/

#include <Arduino.h>
#include <Servo.h>

// ============================ НАСТРОЙКИ ============================

// ---- LCD1602 ----
#define LCD_I2C 1          // 1 = экран с I2C-платой (4 провода), 0 = обычный 16-пиновый
#if LCD_I2C
  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(0x27, 16, 2);           // если экран пустой — попробуй 0x3F
#else
  #include <LiquidCrystal.h>
  LiquidCrystal lcd(40, 41, 42, 43, 44, 45);    // RS, E, D4, D5, D6, D7
#endif

// ---- Моторы (L298N). Перемычки ENA/ENB СНЯТЬ, иначе скорость не регулируется ----
const uint8_t PIN_ENA = 5,  PIN_IN1 = 7,  PIN_IN2 = 8;    // левая пара моторов (OUT1/OUT2)
const uint8_t PIN_ENB = 6,  PIN_IN3 = 9,  PIN_IN4 = 10;   // правая пара моторов (OUT3/OUT4)
const bool    INVERT_LEFT  = false;   // если левая сторона крутится назад — поставь true
const bool    INVERT_RIGHT = false;

const uint8_t SPEED_CRUISE = 170;     // 0..255
const uint8_t SPEED_SLOW   = 140;     // при "жёлтом" уровне газа едем медленнее
const uint8_t SPEED_TURN   = 190;

// Калибровка расстояния (энкодеров нет, считаем по времени):
// поставь ровер на пол, дай ему ехать 3 с, измерь рулеткой путь, раздели на 3.
float CM_PER_SEC_CRUISE = 30.0;
float CM_PER_SEC_SLOW   = 20.0;
unsigned long TURN_90_MS = 550;       // сколько мс крутиться на месте для поворота ~90°

// ---- Серво + ультразвук ----
const uint8_t PIN_SERVO = 11;            // если серво в D4 — поставь 4
const uint8_t SERVO_CENTER = 90, SERVO_LEFT = 160, SERVO_RIGHT = 20;
const uint8_t PIN_TRIG = 12, PIN_ECHO = 13;
const int     OBSTACLE_CM = 25;

// ---- Буззер, кнопка, светодиоды ----
const uint8_t PIN_BUZZER = 3;            // если буззер в D22 — поставь 22
const bool    BUZZER_ACTIVE_LOW = false;   // если пищит постоянно, а при тревоге молчит — поставь true
const uint8_t PIN_BUTTON = 2;              // кнопка на GND (необязательно): старт/стоп
#define USE_LEDS 0                         // 1 = есть светодиоды (через резисторы 220–330 Ом)
const uint8_t PIN_LED_G = 33, PIN_LED_Y = 34, PIN_LED_R = 35;   // не подключены (USE_LEDS 0)

// ---- Напряжение батареи (необязательно): делитель 10k+10k с "+" батареи на A4 ----
#define USE_BATTERY 0
const uint8_t PIN_BATTERY = A4;
const float   BATTERY_DIVIDER = 2.0;

// ---- Миссия ----
const int  STEP_CM          = 50;       // шаг между замерами
const int  STEP_CM_WARN     = 25;       // при "жёлтом" уровне меряем чаще
const int  MISSION_MAX_CM   = 1000;     // длина миссии (10 м), потом стоп
const unsigned long SAMPLE_MS      = 2500;   // стоим и нюхаем на каждой точке
const unsigned long ALARM_SIREN_MS = 4000;   // сирена перед отходом
const unsigned long RETREAT_MS     = 1500;   // отход назад из опасной зоны
const unsigned long MANUAL_PULSE_MS = 500;   // ручная команда = короткий рывок
const unsigned long WARMUP_MS      = 120000UL; // прогрев после включения (2 мин). Команда RECAL пропускает
const unsigned long CAL_MS         = 10000;  // калибровка в чистом воздухе
const unsigned long TELEMETRY_MS   = 1000;

// ---- Пороги ----
// Индекс опасности 0..150: 50 = "жёлтый", 100 = "красный".
// По ppm (MQ-4, MQ-7) и по падению сопротивления относительно чистого воздуха (все датчики).
const float REL_WARN   = 35.0;   // % падения Rs — осторожно
const float REL_DANGER = 60.0;   // % падения Rs — опасно

// ========================== КОНЕЦ НАСТРОЕК ==========================

struct Gas {
  const char* tag;      // подпись
  uint8_t pin;
  bool    hasPpm;       // считаем ли ppm по кривой из даташита
  float   cleanRatio;   // Rs/R0 в чистом воздухе (из даташита)
  float   a, b;         // ppm = a * (Rs/R0)^b
  float   ppmWarn, ppmDanger;
  // рабочие поля
  float   rs0;          // Rs в чистом воздухе (после калибровки)
  float   rsEma;        // сглаженное Rs
  float   ppm;          // оценка ppm (только hasPpm)
  float   dev;          // % падения Rs относительно чистого воздуха
  float   index;        // 0..150
  double  calSum;
  uint16_t calN;
};

// Кривые — из даташитов Hanwei (аппроксимация, как в библиотеке MQUnifiedsensor).
// Цифры ppm у MQ — оценочные! Для питча говорим "≈ ppm".
Gas gas[4] = {
  {"CH4",  A0, true,  4.4,  1012.7, -2.786, 5000, 10000},  // MQ-4: метан. 5000 ppm = 10% НКПР
  {"CO",   A1, true,  27.5, 99.042, -1.518, 25,   50},     // MQ-7: угарный газ (без цикла нагрева — ориентир)
  {"Q135", A2, false, 0, 0, 0, 0, 0},                       // MQ-135: NH3/H2S/"канализация"
  {"Q2",   A3, false, 0, 0, 0, 0, 0},                       // MQ-2: пропан/бутан/горючие
};
const uint8_t GAS_N = 4;

enum State : uint8_t {
  ST_WARMUP, ST_CAL, ST_READY, ST_DRIVE, ST_SAMPLE, ST_AVOID,
  ST_ALARM, ST_RETREAT, ST_HOLD, ST_MANUAL, ST_DONE
};
const char* STATE_CODE[] = {
  "WARM", "CAL", "READY", "DRIVE", "SAMPLE", "AVOID",
  "ALARM", "RETREAT", "HOLD", "MANUAL", "DONE"
};

Servo scanServo;

State  state = ST_WARMUP;
unsigned long stateStart = 0;
unsigned long lastTick = 0;
unsigned long tGas = 0, tLcd = 0, tTel = 0, tSonar = 0;
unsigned long beepUntil = 0;

float distCm = 0;           // пройденный путь (оценка)
float stepTargetCm = 0;     // где следующий замер
int   frontCm = 400;
uint8_t overall = 0;        // 0 ок, 1 осторожно, 2 опасно
int   lastCmdId = -1;       // для отсечки повторов по LoRa

// объезд
uint8_t avoidStep = 0;
unsigned long avoidStart = 0;
int   leftCm = 0, rightCm = 0;
bool  turnLeft = true;

// ручное управление
State manualReturn = ST_READY;
char  manualDir = 'S';

// кнопка
bool  btnPrev = HIGH;
unsigned long btnTime = 0;

String usbLine, megaLine;

// --------------------------- МОТОРЫ ---------------------------

void setSide(uint8_t en, uint8_t inA, uint8_t inB, int speed, bool invert) {
  if (invert) speed = -speed;
  if (speed > 0)      { digitalWrite(inA, HIGH); digitalWrite(inB, LOW);  }
  else if (speed < 0) { digitalWrite(inA, LOW);  digitalWrite(inB, HIGH); }
  else                { digitalWrite(inA, LOW);  digitalWrite(inB, LOW);  }
  analogWrite(en, constrain(abs(speed), 0, 255));
}
void motors(int left, int right) {
  setSide(PIN_ENA, PIN_IN1, PIN_IN2, left,  INVERT_LEFT);
  setSide(PIN_ENB, PIN_IN3, PIN_IN4, right, INVERT_RIGHT);
}
void stopMotors() { motors(0, 0); }

// --------------------------- ДАТЧИКИ ---------------------------

float readRs(uint8_t pin) {
  long sum = 0;
  for (uint8_t i = 0; i < 8; i++) sum += analogRead(pin);
  float v = (sum / 8.0) * 5.0 / 1023.0;
  v = constrain(v, 0.02, 4.98);
  return 10.0 * (5.0 - v) / v;      // RL условно 10 кОм — в отношении Rs/R0 сокращается
}

// кусочно-линейный индекс: 0..warn -> 0..50, warn..danger -> 50..100, дальше до 150
float toIndex(float v, float warn, float danger) {
  float idx;
  if (v <= 0) return 0;
  if (v < warn) idx = v / warn * 50.0;
  else          idx = 50.0 + (v - warn) / (danger - warn) * 50.0;
  return min(idx, 150.0f);
}

void updateGas() {
  float worst = 0;
  for (uint8_t i = 0; i < GAS_N; i++) {
    Gas &g = gas[i];
    float rs = readRs(g.pin);
    g.rsEma = (g.rsEma <= 0) ? rs : g.rsEma * 0.7 + rs * 0.3;

    if (state == ST_CAL) { g.calSum += g.rsEma; g.calN++; }

    if (g.rs0 > 0) {
      float r = g.rsEma / g.rs0;                    // 1.0 = чистый воздух
      g.dev = max(0.0f, (1.0f - r) * 100.0f);
      float idx = toIndex(g.dev, REL_WARN, REL_DANGER);
      if (g.hasPpm) {
        g.ppm = g.a * pow(r * g.cleanRatio, g.b);
        idx = max(idx, toIndex(g.ppm, g.ppmWarn, g.ppmDanger));
      }
      g.index = idx;
    } else {
      g.dev = 0; g.ppm = 0; g.index = 0;
    }
    worst = max(worst, g.index);
  }
  overall = (worst >= 100) ? 2 : (worst >= 50) ? 1 : 0;
}

int readFrontCm() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 25000UL);
  if (us == 0) return 400;     // ничего не видно
  return (int)(us / 58);
}

int batteryDeciVolts() {
#if USE_BATTERY
  float v = analogRead(PIN_BATTERY) * 5.0 / 1023.0 * BATTERY_DIVIDER;
  return (int)(v * 10 + 0.5);
#else
  return 0;
#endif
}

// --------------------------- ТЕЛЕМЕТРИЯ ---------------------------
// $T,STATE,dist_cm,ch4_ppm,co_ppm,q135_dev,q2_dev,i1,i2,i3,i4,overall,front_cm,pt,ack,bat_dV,warm_s

void sendTelemetry(bool point) {
  unsigned long warmLeft = 0;
  if (state == ST_WARMUP) {
    unsigned long el = millis() - stateStart;
    warmLeft = (el < WARMUP_MS) ? (WARMUP_MS - el) / 1000 : 0;
  }
  char buf[140];
  snprintf(buf, sizeof(buf),
           "$T,%s,%d,%ld,%ld,%d,%d,%d,%d,%d,%d,%u,%d,%d,%d,%d,%lu",
           STATE_CODE[state], (int)distCm,
           (long)gas[0].ppm, (long)gas[1].ppm,
           (int)gas[2].dev, (int)gas[3].dev,
           (int)gas[0].index, (int)gas[1].index, (int)gas[2].index, (int)gas[3].index,
           overall, frontCm, point ? 1 : 0, lastCmdId, batteryDeciVolts(), warmLeft);
  Serial1.println(buf);
  Serial.println(buf);
  tTel = millis();
}

// --------------------------- СОСТОЯНИЯ ---------------------------

void enter(State s) {
  state = s;
  stateStart = millis();
  if (s != ST_DRIVE && s != ST_RETREAT && s != ST_MANUAL && s != ST_AVOID) stopMotors();
  sendTelemetry(false);          // база сразу узнаёт о смене состояния
}

void beep(unsigned long ms) { beepUntil = millis() + ms; }

void startMission() {
  distCm = 0;
  stepTargetCm = STEP_CM;
  scanServo.write(SERVO_CENTER);
  enter(ST_SAMPLE);              // первый замер — прямо на входе (0 м)
}

void enterAlarm() {
  stopMotors();
  scanServo.write(SERVO_CENTER);
  enter(ST_ALARM);
  sendTelemetry(true);           // точка тревоги попадает на карту
}

void setAvoidStep(uint8_t s) {
  avoidStep = s;
  avoidStart = millis();
  switch (s) {
    case 0: stopMotors(); scanServo.write(SERVO_LEFT);  break;
    case 1: scanServo.write(SERVO_RIGHT); break;
    case 2: scanServo.write(SERVO_CENTER); break;
    case 3: if (turnLeft) motors(-SPEED_TURN, SPEED_TURN); else motors(SPEED_TURN, -SPEED_TURN); break;
    case 4: stopMotors(); break;
    case 10: motors(-SPEED_CRUISE, -SPEED_CRUISE); break;          // тупик: назад
    case 11: motors(SPEED_TURN, -SPEED_TURN); break;               // и разворот
  }
}

void runStateMachine(unsigned long now) {
  float dt = (now - lastTick) / 1000.0;
  lastTick = now;
  unsigned long inState = now - stateStart;

  switch (state) {

    case ST_WARMUP:
      if (inState >= WARMUP_MS) {
        for (uint8_t i = 0; i < GAS_N; i++) { gas[i].calSum = 0; gas[i].calN = 0; }
        enter(ST_CAL);
      }
      break;

    case ST_CAL:
      if (inState >= CAL_MS) {
        for (uint8_t i = 0; i < GAS_N; i++)
          if (gas[i].calN > 0) gas[i].rs0 = gas[i].calSum / gas[i].calN;
        beep(300);
        enter(ST_READY);
      }
      break;

    case ST_READY:
    case ST_DONE:
      break;

    case ST_DRIVE: {
      if (overall == 2) { enterAlarm(); break; }
      if (now - tSonar >= 80) {
        tSonar = now;
        frontCm = readFrontCm();
        if (frontCm < OBSTACLE_CM) { state = ST_AVOID; stateStart = now; setAvoidStep(0); sendTelemetry(false); break; }
      }
      bool slow = (overall == 1);
      motors(slow ? SPEED_SLOW : SPEED_CRUISE, slow ? SPEED_SLOW : SPEED_CRUISE);
      distCm += dt * (slow ? CM_PER_SEC_SLOW : CM_PER_SEC_CRUISE);
      if (distCm >= stepTargetCm) { stopMotors(); enter(ST_SAMPLE); }
      break;
    }

    case ST_SAMPLE:
      if (overall == 2) { enterAlarm(); break; }
      if (inState >= SAMPLE_MS) {
        sendTelemetry(true);                                   // точка на карте
        if (distCm >= MISSION_MAX_CM) { beep(1000); enter(ST_DONE); break; }
        stepTargetCm = distCm + (overall == 1 ? STEP_CM_WARN : STEP_CM);
        enter(ST_DRIVE);
      }
      break;

    case ST_AVOID: {
      if (overall == 2) { enterAlarm(); break; }
      unsigned long t = now - avoidStart;
      switch (avoidStep) {
        case 0: if (t >= 400) { leftCm = readFrontCm(); setAvoidStep(1); } break;
        case 1: if (t >= 500) { rightCm = readFrontCm(); setAvoidStep(2); } break;
        case 2:
          if (t >= 300) {
            if (max(leftCm, rightCm) < OBSTACLE_CM + 10) setAvoidStep(10);
            else { turnLeft = (leftCm >= rightCm); setAvoidStep(3); }
          }
          break;
        case 3: if (t >= TURN_90_MS) setAvoidStep(4); break;
        case 4:
          if (t >= 200) {
            frontCm = readFrontCm();
            if (frontCm < OBSTACLE_CM) setAvoidStep(0);        // всё ещё стена — смотрим снова
            else enter(ST_DRIVE);
          }
          break;
        case 10:
          distCm = max(0.0f, distCm - dt * CM_PER_SEC_CRUISE);
          if (t >= 400) setAvoidStep(11);
          break;
        case 11: if (t >= TURN_90_MS * 2) setAvoidStep(4); break;
      }
      break;
    }

    case ST_ALARM:
      if (inState >= ALARM_SIREN_MS) {
        state = ST_RETREAT; stateStart = now;
        motors(-SPEED_CRUISE, -SPEED_CRUISE);
        sendTelemetry(false);
      }
      break;

    case ST_RETREAT:
      distCm = max(0.0f, distCm - dt * CM_PER_SEC_CRUISE);
      if (inState >= RETREAT_MS) enter(ST_HOLD);
      break;

    case ST_HOLD:
      break;     // ждём человека: RESUME / STOP / ручное управление

    case ST_MANUAL:
      if (manualDir == 'F') distCm += dt * CM_PER_SEC_CRUISE;
      if (manualDir == 'B') distCm = max(0.0f, distCm - dt * CM_PER_SEC_CRUISE);
      if (inState >= MANUAL_PULSE_MS) { stopMotors(); enter(manualReturn); }
      break;
  }
}

// --------------------------- КОМАНДЫ ---------------------------

void manual(char dir) {
  if (state == ST_WARMUP || state == ST_CAL) return;
  if (state != ST_MANUAL) manualReturn = (state == ST_READY || state == ST_DONE) ? state : ST_HOLD;
  if (dir == 'F') { frontCm = readFrontCm(); if (frontCm < 15) { beep(150); return; } }
  manualDir = dir;
  state = ST_MANUAL; stateStart = millis();
  switch (dir) {
    case 'F': motors(SPEED_CRUISE, SPEED_CRUISE); break;
    case 'B': motors(-SPEED_CRUISE, -SPEED_CRUISE); break;
    case 'L': motors(-SPEED_TURN, SPEED_TURN); break;
    case 'R': motors(SPEED_TURN, -SPEED_TURN); break;
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  Serial.print(F("# CMD ")); Serial.println(cmd);

  if (cmd == "START") {
    if (state == ST_READY || state == ST_DONE || state == ST_HOLD) startMission();
  } else if (cmd == "STOP") {
    if (state == ST_DRIVE || state == ST_SAMPLE || state == ST_AVOID ||
        state == ST_ALARM || state == ST_RETREAT || state == ST_MANUAL) {
      scanServo.write(SERVO_CENTER);
      enter(ST_HOLD);
    }
  } else if (cmd == "RESUME") {
    if (state == ST_HOLD) {
      stepTargetCm = distCm + STEP_CM_WARN;
      enter(ST_DRIVE);
    }
  } else if (cmd == "RECAL") {
    if (state == ST_WARMUP || state == ST_READY || state == ST_DONE || state == ST_HOLD) {
      for (uint8_t i = 0; i < GAS_N; i++) { gas[i].calSum = 0; gas[i].calN = 0; }
      enter(ST_CAL);
    }
  } else if (cmd == "BEEP") {
    beep(1000);
  } else if (cmd == "MAN_F") manual('F');
  else if (cmd == "MAN_B") manual('B');
  else if (cmd == "MAN_L") manual('L');
  else if (cmd == "MAN_R") manual('R');
}

// строка от LilyGO: $C,<id>,<CMD>
void handleLoraLine(const String &line) {
  if (!line.startsWith("$C,")) return;
  int c2 = line.indexOf(',', 3);
  if (c2 < 0) return;
  int id = line.substring(3, c2).toInt();
  if (id == lastCmdId) return;             // база шлёт команду 3 раза — выполняем один
  lastCmdId = id;
  handleCommand(line.substring(c2 + 1));
}

void readSerials() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') { handleLoraLine(megaLine); megaLine = ""; }
    else if (c != '\r' && megaLine.length() < 60) megaLine += c;
  }
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { if (usbLine.length()) handleCommand(usbLine); usbLine = ""; }
    else if (c != '\r' && usbLine.length() < 30) usbLine += c;
  }
}

void readButton(unsigned long now) {
  bool b = digitalRead(PIN_BUTTON);
  if (b == LOW && btnPrev == HIGH && now - btnTime > 300) {
    btnTime = now;
    if (state == ST_READY || state == ST_DONE || state == ST_HOLD) startMission();
    else handleCommand("STOP");
  }
  btnPrev = b;
}

// --------------------------- ВЫВОД ---------------------------

void updateBuzzer(unsigned long now) {
  bool on = false;
  if (now < beepUntil) on = true;
  else if (state == ST_ALARM) on = (now / 150) % 2;                          // сирена
  else if (state == ST_HOLD && overall == 2) on = (now % 1000) < 100;
  else if (overall == 1 && (state == ST_DRIVE || state == ST_SAMPLE)) on = (now % 1500) < 60;
  digitalWrite(PIN_BUZZER, (on != BUZZER_ACTIVE_LOW) ? HIGH : LOW);
}

void updateLeds() {
#if USE_LEDS
  digitalWrite(PIN_LED_G, overall == 0);
  digitalWrite(PIN_LED_Y, overall == 1);
  digitalWrite(PIN_LED_R, overall == 2);
#endif
}

void printLine(uint8_t row, const char *s) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", s);
  lcd.setCursor(0, row);
  lcd.print(buf);
}

void updateLcd() {
  char l1[24], l2[24];
  // HD44780 не знает кириллицу — пишем латиницей
  if (state == ST_WARMUP) {
    unsigned long el = millis() - stateStart;
    unsigned long left = (el < WARMUP_MS) ? (WARMUP_MS - el) / 1000 : 0;
    snprintf(l1, sizeof(l1), "BARLAUSHY v1");
    snprintf(l2, sizeof(l2), "PROGREV %3lus", left);
  } else if (state == ST_CAL) {
    snprintf(l1, sizeof(l1), "KALIBROVKA...");
    snprintf(l2, sizeof(l2), "chistyi vozduh!");
  } else {
    long ch4 = min((long)gas[0].ppm, 99999L);
    long co  = min((long)gas[1].ppm, 9999L);
    snprintf(l1, sizeof(l1), "CH4:%5ldCO:%4ld", ch4, co);
    const char *lv = overall == 2 ? "XX" : overall == 1 ? "!!" : "OK";
    int d = (int)distCm;
    snprintf(l2, sizeof(l2), "%-7s%2d.%1dm %s", STATE_CODE[state], d / 100, (d / 10) % 10, lv);
  }
  printLine(0, l1);
  printLine(1, l2);
}

// --------------------------- SETUP / LOOP ---------------------------

void setup() {
  Serial.begin(115200);
  Serial1.begin(57600);          // к LilyGO (через делитель на линии TX)

  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  stopMotors();

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, BUZZER_ACTIVE_LOW ? HIGH : LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
#if USE_LEDS
  pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_Y, OUTPUT); pinMode(PIN_LED_R, OUTPUT);
#endif

  scanServo.attach(PIN_SERVO);
  scanServo.write(SERVO_CENTER);

#if LCD_I2C
  lcd.init();
  lcd.backlight();
#else
  lcd.begin(16, 2);
#endif
  printLine(0, "BARLAUSHY v1");
  printLine(1, "gas scout robot");

  beep(150);
  lastTick = millis();
  enter(ST_WARMUP);
  Serial.println(F("# Barlaushy ready. Commands: START STOP RESUME RECAL BEEP MAN_F/B/L/R"));
}

void loop() {
  unsigned long now = millis();

  readSerials();
  readButton(now);

  if (now - tGas >= 200) { tGas = now; updateGas(); }
  runStateMachine(now);

  updateBuzzer(now);
  updateLeds();

  if (now - tLcd >= 500) { tLcd = now; updateLcd(); }
  if (now - tTel >= TELEMETRY_MS) sendTelemetry(false);
}
