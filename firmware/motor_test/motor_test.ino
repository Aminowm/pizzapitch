/*
  ПРОВЕРКА МОТОРОВ — самая простая программа, без пульта и без датчиков.
  Залей в Mega, отключи USB-кабель от Mac (или оставь — не важно),
  включи батарею, ПОДНИМИ робота, чтобы колёса висели в воздухе.

  По кругу повторяется:
    пик       -> ЛЕВЫЕ колёса вперёд 2 с на ПОЛНОЙ скорости
    пик-пик   -> ПРАВЫЕ колёса вперёд 2 с
    пик-пик-пик -> ВСЕ колёса назад 2 с
  Пищалка пищит = программа работает. Если пищит, а колёса стоят —
  проблема в питании моторов или в проводах L298N, а не в коде.
*/

const int ENA = 5, IN1 = 7, IN2 = 8;    // левая сторона (по проводам)
const int ENB = 6, IN3 = 9, IN4 = 10;   // правая сторона
const int BUZZER = 3;

void beeps(int n) {
  for (int i = 0; i < n; i++) { digitalWrite(BUZZER, HIGH); delay(120); digitalWrite(BUZZER, LOW); delay(150); }
}

void sideA(int dir) {   // dir: 1 вперёд, -1 назад, 0 стоп
  digitalWrite(IN1, dir > 0); digitalWrite(IN2, dir < 0); analogWrite(ENA, dir ? 255 : 0);
}
void sideB(int dir) {
  digitalWrite(IN3, dir > 0); digitalWrite(IN4, dir < 0); analogWrite(ENB, dir ? 255 : 0);
}

void setup() {
  int pins[] = {ENA, IN1, IN2, ENB, IN3, IN4, BUZZER};
  for (int p : pins) pinMode(p, OUTPUT);
}

void loop() {
  beeps(1); sideA(1);              delay(2000); sideA(0); delay(1000);
  beeps(2); sideB(1);              delay(2000); sideB(0); delay(1000);
  beeps(3); sideA(-1); sideB(-1);  delay(2000); sideA(0); sideB(0); delay(2000);
}
