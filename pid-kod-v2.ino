#include <QTRSensors.h>

// --- MOTOR PİN TANIMLAMALARI ---
const int ENA = 10; // PWMsag
const int IN1 = 9;  
const int IN2 = 8;  

const int ENB = 5;  //PWMsol
const int IN3 = 7;  
const int IN4 = 6;  

// --- SENSÖR PİNLERİ ---
const int MZ80_PIN = 2; 
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// --- MOTOR ÖZEL AYARLARI ---
// temel hızı düşük tutup PID'nin çizgiyi yakalamasına izin vermeliyiz.

int temelHiz = 120;      // Başlangıç için güvenli hız (0-255)
int maxHiz = 200;        // L298N'i korumak ve savrulmamak için limit
float Kp = 0.08;         // Motorlar hızlı olduğu için Kp'yi biraz düşürdük
float Kd = 1.2;          
int sonHata = 0;

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(MZ80_PIN, INPUT);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);

  // MZ80 ile Başlatma (Kapak Algılama)
  while(digitalRead(MZ80_PIN) == LOW) {
    delay(10); 
  }

  // Kalibrasyon (5 saniye boyunca robotu çizgi üzerinde gezdirin)
  for (uint16_t i = 0; i < 250; i++) {
    qtr.calibrate();
    delay(20);
  }
  
  motorDurdur();
  delay(500); // Fırlamadan önce kısa bir bekleme
}

void loop() {
  // Engel Kontrolü (10cm ayarlı MZ80) bunu elle mz80 üzerinden ayarlamalıyız yoksa mesafe hatalı çalışır
  //MZ80 in yere paralel olması önemli açı farkı hataya sebep olabilir
  if (digitalRead(MZ80_PIN) == LOW) {
    aniFren(); 
    while(digitalRead(MZ80_PIN) == LOW); 
  }

  uint16_t pozisyon = qtr.readLineBlack(sensorValues);
  int hata = pozisyon - 3500;

  // PID Hesaplama
  //hız ayarı da bu kısımdadır pid hızdan bağımsız değildir

  int motorHizDegisimi = (Kp * hata) + (Kd * (hata - sonHata));
  sonHata = hata;

  int solMotorHizi = temelHiz + motorHizDegisimi;
  int sagMotorHizi = temelHiz - motorHizDegisimi;

  // 5000 RPM Motorlar için Hassas Sınırlama
  solMotorHizi = constrain(solMotorHizi, 0, maxHiz);
  sagMotorHizi = constrain(sagMotorHizi, 0, maxHiz);

  hareketEt(solMotorHizi, sagMotorHizi);
}

 // hareket fnksiyonları burada tanmladım
void hareketEt(int solHiz, int sagHiz) {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, solHiz);
  analogWrite(ENB, sagHiz);
}

void aniFren() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);
}

void motorDurdur() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}