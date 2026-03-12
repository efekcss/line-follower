
#include <QTRSensors.h>

// ─── MOTOR PİNLERİ (L298N) ───────────────────────────────────────
#define ENA  5    // Sol motor PWM (Timer pini olmalı)
#define IN1  4    // Sol motor yön
#define IN2  3    // Sol motor yön

#define ENB  6    // Sağ motor PWM (Timer pini olmalı)
#define IN3  7    // Sağ motor yön
#define IN4  8    // Sağ motor yön

// ─── QTR-8A SENSÖR AYARLARI ──────────────────────────────────────
#define SENSOR_COUNT 8
QTRSensors qtr;
uint16_t sensorDegerleri[SENSOR_COUNT];

// ─── PID PARAMETRELERİ ────────────────────────────────────────────
// 5000RPM motor için optimize edilmiş başlangıç değerleri
// 12V L298N → Gerçek PWM çıkışı yaklaşık 10.5-11V olur (düşüş var)
float Kp = 0.07;
float Kd = 0.8;    // Kp × 15
float Ki = 0.0001;    // Yarışmada SIFIR tut

// ─── HIZ AYARLARI ────────────────────────────────────────────────
// L298N + 12V 5000RPM → Başlangıçta 70-80% PWM ile çalış
// 255 = tam gaz, yarışmada 180-210 bandından başla
int BASE_HIZ    = 55;   // Düz yol ana hızı (0-255)
int MAX_HIZ     = 110;   // Maksimum hız limiti
int MIN_HIZ     = -40;     // Minimum (negatif = aktif fren)

// ─── DEĞİŞKENLER ─────────────────────────────────────────────────
int oncekiHata  = 0;
float hataBirikiim = 0;
int hedefPozisyon;      // QTR-8A için: 3500 (orta)

// ─────────────────────────────────────────────────────────────────
void setup() {
  // Motor pin modu
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // QTR-8A sensör konfigürasyonu
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SENSOR_COUNT);

  // ── KALİBRASYON ──────────────────────────────────────────────
  // Robot pist üzerinde ileri-geri dönerken 400 örnek alır
  delay(500);
  for (int i = 0; i < 400; i++) {
    // Kalibrasyon sırasında robotu yavaşça sağa-sola sür
    if (i < 100 || i >= 300) {
      solMotor(80, true);   // Sağa dön
      sagMotor(80, false);
    } else {
      solMotor(80, false);  // Sola dön
      sagMotor(80, true);
    }
    qtr.calibrate();
    delay(5);
  }
  motorDur();

  // 8 sensör için hedef merkez: (0+7000)/2 = 3500
  hedefPozisyon = 3500;
  delay(1000); // Başlamadan önce hazır ol
}

// ─────────────────────────────────────────────────────────────────
void loop() {
  // 1. Sensör oku (kalibrasyon sonrası 0-7000 arası değer)
  int pozisyon = qtr.readLineBlack(sensorDegerleri);

  // 2. Hata hesapla (0 = tam merkezde)
  int hata = pozisyon - hedefPozisyon;  // -3500 ile +3500 arası

  // 3. PID hesapla
  hataBirikiim += hata;
  // Anti-windup: integral'in kaymasını önle
  hataBirikiim = constrain(hataBirikiim, -3500, 3500);

  float pid = (Kp * hata) + (Ki * hataBirikiim) + (Kd * (hata - oncekiHata));
  oncekiHata = hata;

  // 4. ADAPTİF HIZ: Hata büyüdükçe ana hızı düşür
  // Virajtayken yavaşla, düz yolda tam gaz
  int adaptifHiz = BASE_HIZ - (int)(abs(hata) * 0.025);
  adaptifHiz = constrain(adaptifHiz, 80, MAX_HIZ);

  // 5. Motor hızlarını hesapla
  int solHiz  = adaptifHiz + (int)pid;
  int sagHiz  = adaptifHiz - (int)pid;

  // 6. Motorları sürmeden önce limit uygula
  solHiz = constrain(solHiz, -MAX_HIZ, MAX_HIZ);
  sagHiz = constrain(sagHiz, -MAX_HIZ, MAX_HIZ);

  // 7. Motorları çalıştır (negatif değer = aktif fren/geri)
  motorSur(solHiz, sagHiz);
}

// ─── MOTOR KONTROL FONKSİYONLARI ─────────────────────────────────

// Ana sürme fonksiyonu: negatif PWM → L298N aktif fren
void motorSur(int sol, int sag) {
  // SOL MOTOR
  if (sol > 0) {
    // İleri
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, sol);
  } else if (sol < 0) {
    // AKTİF FREN (ters PWM ile hard braking)
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, -sol);  // sol negatif, -sol pozitif
  } else {
    // Tam fren (kısa devre yöntemi)
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, 0);
  }

  // SAĞ MOTOR
  if (sag > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, sag);
  } else if (sag < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, -sag);
  } else {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, 0);
  }
}

// Yardımcı fonksiyonlar
void solMotor(int pwm, bool ileri) {
  digitalWrite(IN1, ileri ? HIGH : LOW);
  digitalWrite(IN2, ileri ? LOW : HIGH);
  analogWrite(ENA, pwm);
}

void sagMotor(int pwm, bool ileri) {
  digitalWrite(IN3, ileri ? HIGH : LOW);
  digitalWrite(IN4, ileri ? LOW : HIGH);
  analogWrite(ENB, pwm);
}

void motorDur() {
  // Her iki motoru tam fren yap
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH); analogWrite(ENA, 0);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH); analogWrite(ENB, 0);
}