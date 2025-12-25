#include <QTRSensors.h>

// QTR çizgi sensörü - (QTR sensor)
QTRSensors cizgiSensoru;
unsigned int sensorDegerleri[8];

// Motor pinleri - (Motor Pins)
#define PWMSAG 3
#define SAGIN2 9
#define SAGIN1 8

#define SOLIN1 7
#define SOLIN2 6
#define PWMSOL 5

// LED ve Engel sensörü - (LED and obstacle sensor)
#define LED 13
#define ENGEL_SENSORU 10

// PID ayarları - (PID settings)
float Kp = 0.015;
float Ki = 0.0005;
float Kd = 0.045;
int oncekiHata = 0;
int hataToplam = 0;

// ---------------- Motor Fonksiyonları (Motor Functions) ----------------
void solMotorIleri(int hiz) {
  digitalWrite(SOLIN1, HIGH);
  digitalWrite(SOLIN2, LOW);
  analogWrite(PWMSOL, hiz);
}

void sagMotorIleri(int hiz) {
  digitalWrite(SAGIN1, HIGH);
  digitalWrite(SAGIN2, LOW);
  analogWrite(PWMSAG, hiz);
}

void motorDur() {
  analogWrite(PWMSOL, 0);
  analogWrite(PWMSAG, 0);
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(9600);

  // Motor pin modları
  pinMode(SAGIN1, OUTPUT);
  pinMode(SAGIN2, OUTPUT);
  pinMode(SOLIN1, OUTPUT);
  pinMode(SOLIN2, OUTPUT);
  pinMode(PWMSOL, OUTPUT);
  pinMode(PWMSAG, OUTPUT);

  pinMode(LED, OUTPUT);
  pinMode(ENGEL_SENSORU, INPUT);

  // QTR sensör pinleri (A7 sol, A0 sağ) - (QTR sensor pins (left A7, right A0))
  cizgiSensoru.setTypeAnalog();
  cizgiSensoru.setSensorPins((const byte[]){A7, A6, A5, A4, A3, A2, A1, A0}, 8);

  delay(1000);

  // Kalibrasyon - sağ-sol tarama - (Calibration - left and right scanning)
  digitalWrite(LED, HIGH);
  for (int i = 0; i < 150; i++) {
    cizgiSensoru.calibrate();
    solMotorIleri(60); sagMotorIleri(0);
    delay(10);
  }
  for (int i = 0; i < 150; i++) {
    cizgiSensoru.calibrate();
    solMotorIleri(0); sagMotorIleri(60);
    delay(10);
  }
  motorDur();
  digitalWrite(LED, LOW);

  // Kalibrasyon sonrası LED yanıp söner (the led blinks after the calibration)
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED, HIGH); delay(150);
    digitalWrite(LED, LOW); delay(150);
  }
}

// -------------------- Loop --------------------
void loop() {
  // Engel kontrolü
  if (digitalRead(ENGEL_SENSORU) == LOW) {
    motorDur();
    digitalWrite(LED, HIGH);
    return;
  }
  digitalWrite(LED, LOW);

  // Çizgi pozisyonu - (Lİne Position)
  unsigned int pozisyon = cizgiSensoru.readLineWhite(sensorDegerleri); // 0 (sağ) - 7000 (sol)
  int hata = 3500 - pozisyon;

  // PID hesaplama - (PID calculation)
  float P = Kp * hata;
  hataToplam += hata;
  float I = Ki * hataToplam;
  float D = Kd * (hata - oncekiHata);
  oncekiHata = hata;
  int PID_cikis = P + I + D;

  // Dinamik hız (Dynamic Speed)
  int hiz = map(abs(hata), 0, 2000, 70, 40); // Düzde hızlı, virajda yavaş

  int solPWM = hiz - PID_cikis;
  int sagPWM = hiz + PID_cikis;

  solPWM = constrain(solPWM, 0, hiz);
  sagPWM = constrain(sagPWM, 0, hiz);

  solMotorIleri(solPWM);
  sagMotorIleri(sagPWM);

  delay(10);
}
