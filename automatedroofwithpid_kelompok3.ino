#include <Servo.h>

// Pin definitions
#define RAIN_SENSOR_PIN A1
#define TRIG_PIN 8
#define ECHO_PIN 9
#define SERVO_PIN 10

Servo myServo;

// Threshold values
const int rainThreshold = 1000;
const float stopDistance = 3.0; // cm, objek terlalu dekat
const float slowDistance = 7.0; // cm, memperlambat
const float barrierDistance = 4.5; // cm, jarak ke penghalang

int servoPosition = 90; // Posisi awal servo (setengah terbuka)
float targetPosition = 90; // Posisi target untuk pergerakan halus

// PID constants
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

// PID variables
float previousError = 0;
float integral = 0;

void setup() {
  Serial.begin(9600);
  myServo.attach(SERVO_PIN);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  myServo.write(servoPosition); // Memulai dengan atap setengah terbuka
}

void loop() {
  int rainValue = analogRead(RAIN_SENSOR_PIN);
  float distance = readDistance();

  // Cek kondisi hujan
  bool isRaining = (rainValue < rainThreshold);

  if (isRaining) {
    if (servoPosition < 180) {
      if (distance < stopDistance) {
        // Hentikan jika objek terlalu dekat
        targetPosition = servoPosition;
      } else if (distance < barrierDistance) {
        // Perlambat saat mendekati penghalang
        targetPosition = constrain(targetPosition + 0.2, servoPosition, 180);
      } else {
        // Menutup atap secara halus
        targetPosition = constrain(targetPosition + 1, servoPosition, 180);
      }
    }
  } else {
    if (servoPosition >= 90) {
      if (distance < stopDistance) {
        // Hentikan jika objek terlalu dekat
        targetPosition = servoPosition;
      } else {
        // Membuka atap secara halus
        targetPosition = constrain(targetPosition - 1, 90, servoPosition);
      }
    }
  }

  // PID control
  float error = targetPosition - servoPosition;
  integral += error;
  float derivative = error - previousError;
  float output = Kp * error + Ki * integral + Kd * derivative;
  previousError = error;

  // Update servo position
  servoPosition = constrain(servoPosition + output, 0, 180);
  myServo.write(servoPosition);

  // Output nilai untuk Serial Plotter
  Serial.print(rainValue);
  Serial.print(" ");
  Serial.print(distance);
  Serial.print(" ");
  Serial.println(servoPosition);

  delay(30); // Delay kecil untuk menghindari pergerakan terlalu cepat
}

// Fungsi untuk membaca jarak dari sensor ultrasonik
float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  float duration = pulseIn(ECHO_PIN, HIGH);
  float distance = (duration * 0.034) / 2; // Konversi ke cm
  return distance;
}

// Fungsi untuk menggerakkan servo dengan halus (tidak diperlukan lagi dengan PID)
// int smoothMove(int currentPos, float targetPos, float smoothFactor) {
//   return currentPos + (targetPos - currentPos) * smoothFactor;
// }
