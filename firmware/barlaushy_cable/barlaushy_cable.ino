/*
  ============================================================================
   БАРЛАУШЫ — робот-разведчик газа.  Версия «ПО КАБЕЛЮ» (без LoRa)
   Плата: Arduino Mega 2560
  ============================================================================

   КАК ЭТО РАБОТАЕТ (простыми словами)
   ----------------------------------
   Робот подключён к ноутбуку USB-кабелем. По этому же кабелю он:
     • получает команды от пульта (СТАРТ, СТОП, стрелки…)
     • раз в секунду отправляет ноутбуку всё, что "унюхал"
   Ноутбук (пульт в Chrome) рисует карту туннеля и сам отправляет данные
   на сайт — поэтому QR-код и live-страница тоже работают.

   ЧТО РОБОТ ДЕЛАЕТ ПО ШАГАМ
   -------------------------
   1. ПРОГРЕВ (2 минуты). Датчикам MQ надо нагреться, иначе врут.
   2. КАЛИБРОВКА (10 секунд). Робот запоминает, как пахнет ЧИСТЫЙ воздух.
      Потом всё сравнивает с этим "нулём". Поэтому калибровать — только
      в чистом воздухе, без зажигалки рядом!
   3. ГОТОВ. Ждёт команду СТАРТ.
   4. МИССИЯ: едет 50 см -> останавливается -> нюхает 2,5 с -> ставит точку
      на карте -> снова едет. И так до 10 метров.
   5. Если впереди стена (ближе 25 см) — смотрит "головой" влево и вправо,
      поворачивает туда, где свободнее, и едет дальше.
   6. Если газ ОПАСНЫЙ — стоп, сирена 4 с, отъезд назад, ждёт человека.
      Человек решает: «Продолжить», «Стоп» или ведёт робота стрелками.

   КОМАНДЫ (кнопки пульта или Монитор порта 115200, "Новая строка"):
     START   — начать миссию          STOP    — остановиться
     RESUME  — продолжить после стопа  BEEP    — пискнуть ("где ты?")
     RECAL   — калибровка заново (ТОЛЬКО в чистом воздухе). Во время
               прогрева пропускает его — удобно, если нет времени ждать.
     MAN_F / MAN_B / MAN_L / MAN_R — короткий рывок вперёд/назад/влево/вправо
     TEST    — самопроверка: моторы, "голова", дальномер, пищалка, датчики

   НУЖНЫЕ БИБЛИОТЕКИ (Tools -> Manage Libraries):
     • LiquidCrystal I2C  (автор Frank de Brabander)
     • Servo — уже встроена в Arduino

   КУДА ЧТО ПОДКЛЮЧЕНО (как сейчас собран робот):
     Драйвер моторов L298N: ENA=5  IN1=7  IN2=8   (левые колёса)
                            ENB=6  IN3=9  IN4=10  (правые колёса)
     Газ:  MQ-4 (метан)=A0  MQ-7 (угарный газ)=A1  MQ-135 (аммиак/канализация)=A2
           MQ-2 (пропан/бутан)=A3
     Серво ("голова")=11   Дальномер HC-SR04: TRIG=12  ECHO=13
     Пищалка=3   Экран LCD1602: SDA=20  SCL=21
  ============================================================================
*/

#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ############################################################################
//   НАСТРОЙКИ.  Если что-то не так — меняй ТОЛЬКО здесь.
// ############################################################################

// --- Экран ---
// 0x27 — адрес экрана. Если экран светится, но букв нет — поставь 0x3F.
// (Если букв не видно вообще — покрути отвёрткой синий винтик сзади экрана.)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// --- Моторы ---
const int PIN_ENA = 5, PIN_IN1 = 7, PIN_IN2 = 8;    // левая сторона
const int PIN_ENB = 6, PIN_IN3 = 9, PIN_IN4 = 10;   // правая сторона
// Команда TEST сначала крутит ЛЕВЫЕ колёса вперёд, потом ПРАВЫЕ.
// Если какая-то сторона крутится НАЗАД — поставь для неё true.
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = false;
// Если на TEST первыми крутятся ПРАВЫЕ колёса — поставь true (поменяет стороны местами).
const bool SWAP_SIDES   = false;
// Если робот едет не прямо, а уводит в сторону — одна сторона сильнее.
// Уменьши силу СИЛЬНОЙ стороны (в процентах). Уводит ВЛЕВО -> сильнее правая
// -> RIGHT_POWER = 90 (потом 85, 80…). Уводит ВПРАВО -> уменьши LEFT_POWER.
const int LEFT_POWER  = 100;
const int RIGHT_POWER = 100;

// Скорость: 0 = стоит, 255 = максимум. Меньше ~110 моторы могут не тронуться.
const int SPEED_CRUISE = 170;   // обычная езда
const int SPEED_SLOW   = 140;   // когда газа "немного" — едем осторожнее
const int SPEED_TURN   = 190;   // повороты на месте

// Датчиков пути на колёсах нет, поэтому путь считаем по времени.
// Как настроить: поставь робота на пол, нажми ↑ 6 раз (6 рывков по 0,5 с = 3 с),
// измерь рулеткой, сколько сантиметров он проехал, и раздели на 3.
const float CM_PER_SEC      = 30.0;   // см в секунду на обычной скорости
const float CM_PER_SEC_SLOW = 20.0;   // см в секунду на медленной
// Сколько миллисекунд крутиться на месте, чтобы повернуть примерно на 90°.
const unsigned long TURN_90_MS = 550;

// --- "Голова" (серво) и дальномер ---
const int PIN_SERVO = 11;
const int HEAD_CENTER = 90, HEAD_LEFT = 160, HEAD_RIGHT = 20;   // углы в градусах
const int PIN_TRIG = 12, PIN_ECHO = 13;
const int OBSTACLE_CM = 25;            // ближе этого — считаем, что впереди стена
// Если дальномер глючит (на пульте "Впереди 0 см" и робот всё время объезжает) —
// поставь false: робот будет ездить без объезда препятствий.
const bool USE_SONAR = true;

// --- Пищалка ---
const int  PIN_BUZZER = 3;
// Если пищалка пищит ВСЁ ВРЕМЯ, а при тревоге молчит — поставь true.
const bool BUZZER_INVERTED = false;

// --- Миссия ---
const int STEP_CM        = 50;     // через сколько см делать замер
const int STEP_CM_WARN   = 25;     // если газа "немного" — замеряем чаще
const int MISSION_MAX_CM = 1000;   // длина миссии: 10 м, потом "ГОТОВО"

const unsigned long WARMUP_MS      = 120000;  // прогрев 2 минуты
const unsigned long CAL_MS         = 10000;   // калибровка 10 секунд
const unsigned long SAMPLE_MS      = 2500;    // сколько стоять и нюхать в точке
const unsigned long ALARM_MS       = 4000;    // сирена перед отъездом
const unsigned long RETREAT_MS     = 1500;    // сколько ехать назад от газа
const unsigned long MANUAL_MS      = 500;     // длина одного рывка стрелкой

// --- Пороги газа ---
// Каждый датчик получает "индекс опасности" от 0 до 150:
//     0..49 = зелёный (ОК),  50..99 = жёлтый (осторожно),  100+ = красный (ОПАСНО)
// Индекс считается двумя способами, берётся больший:
//   а) насколько изменился датчик по сравнению с чистым воздухом (в %)
const float CHANGE_WARN   = 35.0;   // изменился на 35% — жёлтый
const float CHANGE_DANGER = 60.0;   // изменился на 60% — красный
//   б) для MQ-4 и MQ-7 ещё и по примерным ppm (частиц газа на миллион):
const float CH4_WARN = 5000,  CH4_DANGER = 10000;   // метан: 10000 ppm = 20% от взрывоопасного
const float CO_WARN  = 25,    CO_DANGER  = 50;      // угарный газ: норма для рабочей зоны ~20 ppm

// ############################################################################
//   ДАЛЬШЕ — САМА ПРОГРАММА. Трогать не обязательно.
// ############################################################################

// ---------------------------------------------------------------------------
//  1. ДАТЧИКИ ГАЗА
// ---------------------------------------------------------------------------
// Как работает датчик MQ: внутри нагретая пластинка. Когда рядом газ, её
// сопротивление ПАДАЕТ. Arduino меряет напряжение, из него считаем
// сопротивление и сравниваем с тем, что было в чистом воздухе.

struct GasSensor {
  const char* name;   // короткое имя
  int   pin;          // к какой ножке подключён
  bool  hasPpm;       // умеем ли считать ppm (только MQ-4 и MQ-7)
  float cleanRatio;   // из документации датчика: значение в чистом воздухе
  float a, b;         // формула из документации: ppm = a * (отношение)^b
  float ppmWarn, ppmDanger;
  // Эти значения программа заполняет сама:
  float cleanR;       // сопротивление в чистом воздухе (после калибровки)
  float smoothR;      // текущее сопротивление (сглаженное, без скачков)
  float ppm;          // примерно сколько газа (ppm)
  float change;       // на сколько % изменился по сравнению с чистым воздухом
  float danger;       // индекс опасности 0..150
  float calSum;       // для калибровки: сумма замеров
  int   calCount;     //                  и их количество
};

GasSensor gas[4] = {
  // имя    пин  ppm?  чист.  a        b       жёлтый    красный
  {"CH4",   A0,  true, 4.4,   1012.7, -2.786,  CH4_WARN, CH4_DANGER},  // MQ-4: метан (шахта, канализация)
  {"CO",    A1,  true, 27.5,  99.042, -1.518,  CO_WARN,  CO_DANGER},   // MQ-7: угарный газ
  {"Q135",  A2,  false, 0, 0, 0, 0, 0},                                // MQ-135: аммиак, сероводород
  {"Q2",    A3,  false, 0, 0, 0, 0, 0},                                // MQ-2: пропан, бутан (зажигалка!)
};

int gasLevel = 0;   // общий уровень: 0 = ОК, 1 = осторожно, 2 = ОПАСНО

// Читаем датчик: 8 замеров подряд, берём среднее (меньше "шума")
float readResistance(int pin) {
  long sum = 0;
  for (int i = 0; i < 8; i++) sum += analogRead(pin);
  float volts = (sum / 8.0) * 5.0 / 1023.0;     // 0..1023 -> 0..5 вольт
  volts = constrain(volts, 0.02, 4.98);         // защита от деления на ноль
  return 10.0 * (5.0 - volts) / volts;          // сопротивление (в условных кОм)
}

// Переводим число в индекс 0..150 по двум порогам (жёлтый и красный)
float toDanger(float value, float warn, float danger) {
  if (value <= 0) return 0;
  float d;
  if (value < warn) d = value / warn * 50.0;                       // 0..50
  else              d = 50.0 + (value - warn) / (danger - warn) * 50.0;  // 50..100 и выше
  return min(d, 150.0f);
}

// Вызывается 5 раз в секунду: обновляем все 4 датчика
void updateGas(bool calibrating) {
  float worst = 0;
  for (int i = 0; i < 4; i++) {
    GasSensor &g = gas[i];
    float r = readResistance(g.pin);
    // сглаживание: 70% старого + 30% нового (чтобы цифры не прыгали)
    g.smoothR = (g.smoothR <= 0) ? r : g.smoothR * 0.7 + r * 0.3;

    if (calibrating) { g.calSum += g.smoothR; g.calCount++; }

    if (g.cleanR > 0) {                               // калибровка уже была
      float ratio = g.smoothR / g.cleanR;             // 1.0 = как в чистом воздухе
      g.change = max(0.0f, (1.0f - ratio) * 100.0f);  // упало на сколько %
      g.danger = toDanger(g.change, CHANGE_WARN, CHANGE_DANGER);
      if (g.hasPpm) {
        g.ppm = g.a * pow(ratio * g.cleanRatio, g.b);
        g.danger = max(g.danger, toDanger(g.ppm, g.ppmWarn, g.ppmDanger));
      }
    } else {                                          // ещё не калибровались
      g.change = 0; g.ppm = 0; g.danger = 0;
    }
    worst = max(worst, g.danger);
  }
  gasLevel = (worst >= 100) ? 2 : (worst >= 50) ? 1 : 0;
}

// ---------------------------------------------------------------------------
//  2. МОТОРЫ
// ---------------------------------------------------------------------------
// У драйвера L298N на каждую сторону 3 провода:
//   IN-провода задают НАПРАВЛЕНИЕ (вперёд/назад), EN-провод — СКОРОСТЬ.
// speed: от -255 (полный назад) до +255 (полный вперёд), 0 = стоп.

void setSide(int pinEN, int pinA, int pinB, int speed, bool invert) {
  if (invert) speed = -speed;
  if (speed > 0)      { digitalWrite(pinA, HIGH); digitalWrite(pinB, LOW);  }
  else if (speed < 0) { digitalWrite(pinA, LOW);  digitalWrite(pinB, HIGH); }
  else                { digitalWrite(pinA, LOW);  digitalWrite(pinB, LOW);  }
  analogWrite(pinEN, constrain(abs(speed), 0, 255));
}

void drive(int left, int right) {
  left  = left  * LEFT_POWER  / 100;
  right = right * RIGHT_POWER / 100;
  if (SWAP_SIDES) { int t = left; left = right; right = t; }
  setSide(PIN_ENA, PIN_IN1, PIN_IN2, left,  INVERT_LEFT);
  setSide(PIN_ENB, PIN_IN3, PIN_IN4, right, INVERT_RIGHT);
}
void stopMotors()  { drive(0, 0); }
void forward(int s){ drive(s, s); }
void backward(int s){ drive(-s, -s); }
void spinLeft()    { drive(-SPEED_TURN, SPEED_TURN); }   // левые назад, правые вперёд
void spinRight()   { drive(SPEED_TURN, -SPEED_TURN); }

// ---------------------------------------------------------------------------
//  3. ДАЛЬНОМЕР И "ГОЛОВА"
// ---------------------------------------------------------------------------
// HC-SR04 "кричит" ультразвуком и слушает эхо. Чем дольше эхо летит,
// тем дальше стена. 58 микросекунд эха = 1 см.

Servo head;

int measureDistance() {
  if (!USE_SONAR) return 400;                            // дальномер выключен в настройках
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);   // короткий "крик"
  digitalWrite(PIN_TRIG, LOW);
  unsigned long echo = pulseIn(PIN_ECHO, HIGH, 25000UL); // ждём эхо максимум 25 мс
  if (echo == 0) return 400;                             // эхо не вернулось = впереди пусто
  int cm = echo / 58;
  if (cm < 3) return 400;   // ближе 3 см HC-SR04 мерить не умеет — это помеха, а не стена
  return cm;
}

// ---------------------------------------------------------------------------
//  4. ПИЩАЛКА
// ---------------------------------------------------------------------------
unsigned long beepUntil = 0;
void beep(unsigned long ms) { beepUntil = millis() + ms; }
void buzzer(bool on) { digitalWrite(PIN_BUZZER, (on != BUZZER_INVERTED) ? HIGH : LOW); }

// ---------------------------------------------------------------------------
//  5. РЕЖИМЫ РОБОТА
// ---------------------------------------------------------------------------
// Робот всегда находится ровно в одном режиме. Имена режимов латиницей —
// их же видит пульт и сайт.
enum Mode { WARM, CAL, READY, DRIVE, SAMPLE, AVOID, ALARM, RETREAT, HOLD, MANUAL, DONE };
const char* MODE_NAME[] = { "WARM", "CAL", "READY", "DRIVE", "SAMPLE", "AVOID",
                            "ALARM", "RETREAT", "HOLD", "MANUAL", "DONE" };

Mode mode = WARM;
unsigned long modeStart = 0;     // когда вошли в текущий режим
unsigned long lastLoop = 0;

float distanceCm   = 0;          // сколько проехали от входа (примерно)
float nextSampleCm = 0;          // где следующий замер
int   frontCm      = 400;        // что видит дальномер
int   cmdCount     = 0;          // сколько команд получено (пульт видит, что дошло)

// для объезда
int  avoidStep = 0;
unsigned long avoidStepStart = 0;
int  lookLeftCm = 0, lookRightCm = 0;
bool goLeft = true;

// для ручного управления
Mode modeAfterManual = READY;
char manualDir = 'S';

void sendData(bool mapPoint);   // (описана ниже)
void lcdRestart();               // (описана ниже)

// Сменить режим
// (параметр int, а не Mode — так Arduino IDE не путается при сборке)
void setMode(int m) {
  mode = (Mode)m;
  modeStart = millis();
  if (m != DRIVE && m != RETREAT && m != MANUAL && m != AVOID) {
    stopMotors();
    lcdRestart();   // моторы "шумят" в проводах и сбивают экран — когда встали, перезапускаем его
  }
  sendData(false);                // сразу сообщаем пульту
}

void startMission() {
  distanceCm = 0;
  nextSampleCm = STEP_CM;
  head.write(HEAD_CENTER);
  setMode(SAMPLE);                // первый замер — прямо у входа (0 м)
}

void startAlarm() {
  stopMotors();
  head.write(HEAD_CENTER);
  setMode(ALARM);
  sendData(true);                 // точка тревоги — на карту
}

// Объезд препятствия — по шагам:
//  0: смотрим влево   1: смотрим вправо   2: решаем куда
//  3: поворачиваем    4: проверяем, свободно ли
//  10: тупик -> назад 11: разворот
void avoidGo(int step) {
  avoidStep = step;
  avoidStepStart = millis();
  switch (step) {
    case 0:  stopMotors(); head.write(HEAD_LEFT); break;
    case 1:  head.write(HEAD_RIGHT); break;
    case 2:  head.write(HEAD_CENTER); break;
    case 3:  if (goLeft) spinLeft(); else spinRight(); break;
    case 4:  stopMotors(); break;
    case 10: backward(SPEED_CRUISE); break;
    case 11: spinRight(); break;
  }
}

// Главная логика: вызывается много раз в секунду
void runRobot(unsigned long now) {
  float sec = (now - lastLoop) / 1000.0;   // сколько секунд прошло с прошлого раза
  lastLoop = now;
  unsigned long inMode = now - modeStart;   // сколько мс мы уже в этом режиме

  switch (mode) {

    case WARM:                               // греемся
      if (inMode >= WARMUP_MS) {
        for (int i = 0; i < 4; i++) { gas[i].calSum = 0; gas[i].calCount = 0; }
        setMode(CAL);
      }
      break;

    case CAL:                                // запоминаем чистый воздух
      if (inMode >= CAL_MS) {
        for (int i = 0; i < 4; i++)
          if (gas[i].calCount > 0) gas[i].cleanR = gas[i].calSum / gas[i].calCount;
        beep(300);
        setMode(READY);
      }
      break;

    case READY:
    case DONE:
    case HOLD:                               // ждём команду человека
      break;

    case DRIVE: {                            // едем к следующей точке
      if (gasLevel == 2) { startAlarm(); break; }
      static unsigned long lastLook = 0;
      if (now - lastLook >= 80) {            // смотрим вперёд ~12 раз в секунду
        lastLook = now;
        frontCm = measureDistance();
        if (frontCm < OBSTACLE_CM) { mode = AVOID; modeStart = now; avoidGo(0); sendData(false); break; }
      }
      bool careful = (gasLevel == 1);
      forward(careful ? SPEED_SLOW : SPEED_CRUISE);
      distanceCm += sec * (careful ? CM_PER_SEC_SLOW : CM_PER_SEC);
      if (distanceCm >= nextSampleCm) { stopMotors(); setMode(SAMPLE); }
      break;
    }

    case SAMPLE:                             // стоим и нюхаем
      if (gasLevel == 2) { startAlarm(); break; }
      if (inMode >= SAMPLE_MS) {
        sendData(true);                      // точка на карту
        if (distanceCm >= MISSION_MAX_CM) { beep(1000); setMode(DONE); break; }
        nextSampleCm = distanceCm + (gasLevel == 1 ? STEP_CM_WARN : STEP_CM);
        setMode(DRIVE);
      }
      break;

    case AVOID: {                            // объезжаем стену
      if (gasLevel == 2) { startAlarm(); break; }
      unsigned long t = now - avoidStepStart;
      switch (avoidStep) {
        case 0: if (t >= 400) { lookLeftCm  = measureDistance(); avoidGo(1); } break;
        case 1: if (t >= 500) { lookRightCm = measureDistance(); avoidGo(2); } break;
        case 2:
          if (t >= 300) {
            if (max(lookLeftCm, lookRightCm) < OBSTACLE_CM + 10) avoidGo(10);  // везде стены — тупик
            else { goLeft = (lookLeftCm >= lookRightCm); avoidGo(3); }         // туда, где свободнее
          }
          break;
        case 3: if (t >= TURN_90_MS) avoidGo(4); break;
        case 4:
          if (t >= 200) {
            frontCm = measureDistance();
            if (frontCm < OBSTACLE_CM) avoidGo(0);    // всё ещё стена — смотрим снова
            else setMode(DRIVE);                      // свободно — едем дальше
          }
          break;
        case 10:
          distanceCm = max(0.0f, distanceCm - sec * CM_PER_SEC);
          if (t >= 400) avoidGo(11);
          break;
        case 11: if (t >= TURN_90_MS * 2) avoidGo(4); break;
      }
      break;
    }

    case ALARM:                              // сирена, стоим
      if (inMode >= ALARM_MS) {
        mode = RETREAT; modeStart = now;
        backward(SPEED_CRUISE);
        sendData(false);
      }
      break;

    case RETREAT:                            // уезжаем назад от газа
      distanceCm = max(0.0f, distanceCm - sec * CM_PER_SEC);
      if (inMode >= RETREAT_MS) setMode(HOLD);
      break;

    case MANUAL:                             // короткий рывок от стрелки
      if (manualDir == 'F') distanceCm += sec * CM_PER_SEC;
      if (manualDir == 'B') distanceCm = max(0.0f, distanceCm - sec * CM_PER_SEC);
      if (inMode >= MANUAL_MS) { stopMotors(); setMode(modeAfterManual); }
      break;
  }
}

// ---------------------------------------------------------------------------
//  6. ОТПРАВКА ДАННЫХ НА НОУТБУК
// ---------------------------------------------------------------------------
// Одна строка в секунду. Пульт её разбирает. Порядок полей менять НЕЛЬЗЯ:
// $T,режим,путь_см,CH4_ppm,CO_ppm,MQ135_%,MQ2_%,инд1,инд2,инд3,инд4,уровень,
//    дальномер_см,точка_на_карте(1/0),команд_получено,батарея(0),прогрев_осталось_с
unsigned long lastSend = 0;

void sendData(bool mapPoint) {
  unsigned long warmLeft = 0;
  if (mode == WARM) {
    unsigned long passed = millis() - modeStart;
    warmLeft = (passed < WARMUP_MS) ? (WARMUP_MS - passed) / 1000 : 0;
  }
  char line[140];
  snprintf(line, sizeof(line), "$T,%s,%d,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,0,%lu",
           MODE_NAME[mode], (int)distanceCm,
           (long)gas[0].ppm, (long)gas[1].ppm, (int)gas[2].change, (int)gas[3].change,
           (int)gas[0].danger, (int)gas[1].danger, (int)gas[2].danger, (int)gas[3].danger,
           gasLevel, frontCm, mapPoint ? 1 : 0, cmdCount, warmLeft);
  Serial.println(line);
  lastSend = millis();
}

// ---------------------------------------------------------------------------
//  7. КОМАНДЫ С НОУТБУКА
// ---------------------------------------------------------------------------

void manualMove(char dir) {
  if (mode == WARM || mode == CAL) return;                 // пока греемся — не ездим
  if (mode != MANUAL) modeAfterManual = (mode == READY || mode == DONE) ? mode : HOLD;
  if (dir == 'F') { frontCm = measureDistance(); if (frontCm < 15) { beep(150); return; } }  // в стену не едем
  manualDir = dir;
  mode = MANUAL; modeStart = millis();
  if (dir == 'F') forward(SPEED_CRUISE);
  if (dir == 'B') backward(SPEED_CRUISE);
  if (dir == 'L') spinLeft();
  if (dir == 'R') spinRight();
}

void selfTest();   // (описана ниже)

void doCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.startsWith("CMD:")) cmd = cmd.substring(4);    // на всякий случай (формат старой базы)
  if (cmd.length() == 0) return;
  cmdCount++;
  Serial.print(F("# Команда: ")); Serial.println(cmd);

  if (cmd == "START") {
    if (mode == READY || mode == DONE || mode == HOLD) startMission();
    else Serial.println(F("# Сейчас нельзя: робот ещё греется/калибруется"));
  }
  else if (cmd == "STOP") {
    if (mode == DRIVE || mode == SAMPLE || mode == AVOID || mode == ALARM ||
        mode == RETREAT || mode == MANUAL) { head.write(HEAD_CENTER); setMode(HOLD); }
  }
  else if (cmd == "RESUME") {
    if (mode == HOLD) { nextSampleCm = distanceCm + STEP_CM_WARN; setMode(DRIVE); }
  }
  else if (cmd == "RECAL") {
    if (mode == WARM || mode == READY || mode == DONE || mode == HOLD) {
      for (int i = 0; i < 4; i++) { gas[i].calSum = 0; gas[i].calCount = 0; }
      setMode(CAL);
    }
  }
  else if (cmd == "BEEP")  beep(1000);
  else if (cmd == "MAN_F") manualMove('F');
  else if (cmd == "MAN_B") manualMove('B');
  else if (cmd == "MAN_L") manualMove('L');
  else if (cmd == "MAN_R") manualMove('R');
  else if (cmd == "TEST")  selfTest();
  else Serial.println(F("# Не знаю такую команду"));
}

// Собираем буквы с кабеля, пока не придёт "Enter" — тогда выполняем
String typed = "";
void readCable() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { doCommand(typed); typed = ""; }
    else if (c != '\r' && typed.length() < 30) typed += c;
  }
}

// ---------------------------------------------------------------------------
//  8. САМОПРОВЕРКА (команда TEST). Поставь робота так, чтобы колёса
//     не касались стола, или держи его в руках!
// ---------------------------------------------------------------------------
void selfTest() {
  if (mode != WARM && mode != READY && mode != DONE && mode != HOLD) {
    Serial.println(F("# TEST только когда робот стоит")); return;
  }
  stopMotors();
  Serial.println(F("# TEST 1/5: ЛЕВЫЕ колёса ВПЕРЁД"));
  drive(SPEED_CRUISE, 0); delay(800); stopMotors(); delay(400);
  Serial.println(F("# TEST 2/5: ПРАВЫЕ колёса ВПЕРЁД"));
  drive(0, SPEED_CRUISE); delay(800); stopMotors(); delay(400);
  Serial.println(F("# TEST 3/5: голова влево-вправо-прямо"));
  head.write(HEAD_LEFT);  delay(600);
  head.write(HEAD_RIGHT); delay(600);
  head.write(HEAD_CENTER); delay(400);
  Serial.print(F("# TEST 4/5: дальномер видит ")); Serial.print(measureDistance()); Serial.println(F(" см (поднеси ладонь — число должно уменьшиться)"));
  Serial.println(F("# TEST 5/5: пищалка"));
  buzzer(true); delay(300); buzzer(false);
  for (int i = 0; i < 4; i++) {
    Serial.print(F("# датчик ")); Serial.print(gas[i].name);
    Serial.print(F(" сырое значение = ")); Serial.print(analogRead(gas[i].pin));
    Serial.println(F(" (норма 50..700; 0 или 1023 = провод не подключён)"));
  }
  Serial.println(F("# TEST готов"));
  lastLoop = millis();
}

// ---------------------------------------------------------------------------
//  9. ЭКРАН И ПИЩАЛКА
// ---------------------------------------------------------------------------
// Экран не знает русских букв — пишем латиницей.
void lcdLine(int row, const char* text) {
  char buf[17];
  snprintf(buf, sizeof(buf), "%-16s", text);   // дополняем пробелами до 16 символов
  lcd.setCursor(0, row);
  lcd.print(buf);
}

// Моторы при работе дают электрические помехи, экран от них может начать
// показывать "кракозябры". Перезапуск экрана (0,1 с) всё исправляет.
void lcdRestart() {
  lcd.init();
  lcd.backlight();
}

void updateScreen() {
  char top[24], bottom[24];
  if (mode == WARM) {
    unsigned long passed = millis() - modeStart;
    unsigned long left = (passed < WARMUP_MS) ? (WARMUP_MS - passed) / 1000 : 0;
    snprintf(top, sizeof(top), "BARLAUSHY");
    snprintf(bottom, sizeof(bottom), "PROGREV %3lus", left);
  } else if (mode == CAL) {
    snprintf(top, sizeof(top), "KALIBROVKA...");
    snprintf(bottom, sizeof(bottom), "chistyi vozduh!");
  } else {
    snprintf(top, sizeof(top), "CH4:%5ldCO:%4ld", min((long)gas[0].ppm, 99999L), min((long)gas[1].ppm, 9999L));
    const char* level = (gasLevel == 2) ? "XX" : (gasLevel == 1) ? "!!" : "OK";
    int d = (int)distanceCm;
    snprintf(bottom, sizeof(bottom), "%-7s%2d.%1dm %s", MODE_NAME[mode], d / 100, (d / 10) % 10, level);
  }
  lcdLine(0, top);
  lcdLine(1, bottom);
}

void updateBuzzer(unsigned long now) {
  bool on = false;
  if (now < beepUntil)                    on = true;                    // короткий писк по команде
  else if (mode == ALARM)                 on = (now / 150) % 2;         // сирена
  else if (mode == HOLD && gasLevel == 2) on = (now % 1000) < 100;      // газ всё ещё рядом
  else if (gasLevel == 1 && (mode == DRIVE || mode == SAMPLE)) on = (now % 1500) < 60;  // "тик" осторожно
  buzzer(on);
}

// ---------------------------------------------------------------------------
//  10. СТАРТ И ГЛАВНЫЙ ЦИКЛ
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);                       // скорость кабеля — как в пульте

  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENB, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  stopMotors();

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  buzzer(false);

  head.attach(PIN_SERVO);
  head.write(HEAD_CENTER);

  lcd.init();
  lcd.backlight();
  lcdLine(0, "BARLAUSHY");
  lcdLine(1, "gas scout robot");

  beep(150);                                  // "пик" = программа запустилась
  lastLoop = millis();
  setMode(WARM);
  Serial.println(F("# Barlaushy (kabel) gotov. Komandy: START STOP RESUME RECAL BEEP MAN_F/B/L/R TEST"));
}

void loop() {
  unsigned long now = millis();

  readCable();                                       // 1. есть ли команда?

  static unsigned long lastGas = 0;
  if (now - lastGas >= 200) {                        // 2. нюхаем 5 раз в секунду
    lastGas = now;
    updateGas(mode == CAL);
  }

  runRobot(now);                                     // 3. решаем, что делать
  updateBuzzer(now);                                 // 4. пищалка

  static unsigned long lastScreen = 0;
  if (now - lastScreen >= 500) { lastScreen = now; updateScreen(); }   // 5. экран 2 раза в секунду

  if (now - lastSend >= 1000) sendData(false);       // 6. данные на ноутбук раз в секунду
}
