# **Automated Roof**
Automated Roof adalah sebuah sistem atap otomatis yang dapat beradaptasi dengan kondisi cuaca. Sistem ini menggunakan servo motor sebagai aktuator utama dan dilengkapi dengan sensor ultrasonik untuk mengukur jarak serta sensor air hujan untuk mendeteksi hujan. Dengan mengintegrasikan kendali PID (Proportional, Integral, Derivative), sistem ini mampu menggerakkan atap dengan presisi tinggi, memastikan atap berhenti pada posisi yang aman dan optimal. Implementasi PID controller memungkinkan penyesuaian real-time terhadap pergerakan atap, meningkatkan stabilitas dan keandalan sistem dalam kondisi cuaca yang beragam. Analisis dan pengujian yang dilakukan menunjukkan bahwa sistem ini berhasil meningkatkan kenyamanan dan efisiensi energi dalam bangunan, serta memberikan perlindungan dari kondisi cuaca ekstrem. Hasil proyek ini diharapkan dapat mendorong adopsi lebih luas teknologi otomatisasi dalam sektor perumahan dan bangunan.

## Anggota Kelompok
* Axel David 1103210017
* Farhan Rizki Fauzi 1103210064
* Muhammad Musyaffakul Harisna 110321003131
* Ilham Khodar Trijaya 110321003084
* Haldi Alfiansyach 1103213196

> **[CLO 4]** Memiliki kemampuan untuk menganalisis sistem kendali loop tertutup pada kondisi transien dan steady state untuk melihat performansinya.
> **[CLO 5]** Memiliki kemampuan merancang sistem kendali motor DC.
<!--
> **Capaian CLO 4:**
> 1.Mahasiswa mampu menjelaskan konsep kendali umpan balik.
> 2.Mahasiswa mampu merancang sistem kendali PID.
> 3.Mahasiswa mampu mengevaluasi dan optimasi sistem kendali PID.
> **Capaian CLO 5:**
> 1.Mahasiswa mampu mendapatkan fungsi transfer sistem dari mekanisme transfer daya.
> 2.Mampu menjelaskan cara kerja dan karakteristik motor listrik, khususnya motor DC.
> 3.Mampu menganalisis hubungan antara torsi dan kecepatan motor.
> 4.Mahasiswa mampu mendemonstrasikan sistem mekanisme transfer daya sederhana menggunakan gear dan motor DC.
> 
-->


## Pendahuluan
### Latar Belakang

Dalam beberapa dekade terakhir, perkembangan teknologi otomatisasi telah mengalami kemajuan pesat dan memberikan dampak signifikan pada berbagai aspek kehidupan manusia. Salah satu area yang mendapatkan perhatian khusus adalah otomasi dalam sektor perumahan dan bangunan. Dengan meningkatnya kebutuhan akan kenyamanan, efisiensi energi, dan perlindungan lingkungan, inovasi dalam sistem bangunan otomatis menjadi sangat relevan dan penting.

Sistem atap otomatis merupakan salah satu solusi inovatif yang bertujuan untuk memberikan kemudahan dalam mengelola kondisi bangunan, terutama dalam hal perlindungan dari cuaca dan pengaturan suhu dalam ruangan. Di lingkungan tropis dan sub-tropis, kondisi cuaca yang berubah-ubah, seperti hujan tiba-tiba atau sinar matahari yang terik, dapat mempengaruhi kenyamanan dan keamanan di dalam bangunan. Oleh karena itu, penggunaan atap yang dapat bergerak secara otomatis untuk menyesuaikan dengan kondisi cuaca menjadi solusi yang sangat diperlukan. Sistem ini menggunakan servo sebagai aktuator utama, yang dipilih karena kemampuan gerak presisi dan kemudahan kontrol melalui pemrograman. Dengan sensor ultrasonik, sistem dapat memastikan bahwa atap bergerak dengan akurat dan berhenti pada posisi yang tepat, mencegah potensi kerusakan atau bahaya.

Lebih lanjut, sistem atap otomatis ini juga dilengkapi dengan sensor air hujan. Sensor air hujan memungkinkan atap menutup secara otomatis saat mendeteksi adanya air hujan, melindungi bagian dalam bangunan dari kerusakan akibat air.

### Tujuan

Tujuan dari pengembangan sistem atap otomatis ini adalah untuk meningkatkan kenyamanan pengguna dengan menyediakan mekanisme yang memungkinkan atap untuk bergerak maju dan mundur sesuai kebutuhan, sehingga dapat menyesuaikan dengan kondisi cuaca. Dengan adanya sistem ini, diharapkan pengguna dapat menikmati kenyamanan optimal di dalam bangunan tanpa harus secara manual menyesuaikan atap. Selain itu, sistem ini bertujuan untuk menjamin keamanan dan keandalan operasional atap dengan menggunakan sensor ultrasonik yang memastikan atap berhenti pada jarak yang aman, menghindari potensi kerusakan atau bahaya yang mungkin terjadi jika atap bergerak terlalu jauh.

Lebih lanjut, tujuan lain dari sistem ini adalah untuk melindungi bangunan dari cuaca ekstrem, khususnya hujan, dengan mengintegrasikan sensor air hujan yang dapat mendeteksi adanya hujan dan menutup atap secara otomatis. Hal ini akan melindungi bagian dalam bangunan dari kerusakan akibat air. Selain itu, pengembangan sistem atap otomatis ini diharapkan dapat mempromosikan penggunaan teknologi otomatisasi yang lebih luas dalam sektor perumahan dan bangunan, dengan menciptakan solusi yang cerdas dan terintegrasi untuk manajemen bangunan modern. Dengan sistem otomatis yang mampu menanggapi kondisi lingkungan secara real-time, diharapkan dapat meminimalisir biaya pemeliharaan yang disebabkan oleh kerusakan akibat cuaca atau penggunaan energi yang berlebihan. 


## Rancangan Sistem Kendali Loop Tertutup PID
Teori yang mendukung/ metode-metode yang digunakan
<br>
![rancangan COSE.drawio](https://hackmd.io/_uploads/B1sUSitS0.png)


Dari gambar Diagram Close Loop diatas berikut merupakan beberapa penjelasan dari setiap bagian:
*  Set Point
    *   Rain Sensor: Mendeteksi tetesan air hujan.
    *   Ultrasonic Sensor (Jarak): Mengukur jarak objek di depan atap.


* Comparator
    * Threshold Hujan: Membandingkan nilai Rain Sensor dengan ambang batas deteksi hujan.
    * Threshold Jarak: Membandingkan nilai jarak dengan ambArg batas jarak minimum dan maksimum
    * 


* Controller
    * Logika Kontrol: Memproses hasil perbanding dan mengambil keputusan untuk menggerakkan servo motor.
 
* Aktuator
    * Servo Motor: Menggerakkan atap sesuai sinyal dari controller.

* Proses
    * Atap Otomatis: Sistem fisik yang membuka atau menutup berdasarkan perintah dari servo motor.

*  Sensor
    * HC-SR04 : Mengukur dan mendeteksi jarak objek didepan atap

* Feedback
    *  Feedback dari posisi servo dan jarak objek dikembalikan ke input sensor untuk memastikan bahwa controller dapat menyesuaikan pergerakan servo secara real-time berdasarkan perubahan lingkungan.
    

## Mekanik Motor DC/Aktuator
Dalam proyek sistem atap otomatis dengan menggunakan servo motor, implementasi PID controller sangat krusial untuk menjaga posisi atap sesuai dengan setpoint yang diinginkan. PID controller terintegrasi dalam mikrocontroller berfungsi untuk mengontrol pergerakan servo motor berdasarkan feedback yang diberikan oleh sensor ultrasonik mengenai posisi aktual atap. Parameter PID, yaitu Proporsional (P), Integral (I), dan Derivatif (D), disesuaikan untuk mengoptimalkan respons sistem terhadap perubahan posisi atap, sehingga sistem dapat dengan cepat menyesuaikan pergerakan atap dan menghentikan servo motor tepat pada posisi yang ditentukan.

Mikrocontroller bertanggung jawab dalam membaca dan menganalisis data dari sensor ultrasonik, yang mengukur jarak antara atap dan sensor. Informasi ini kemudian digunakan untuk menghitung error antara posisi aktual dan setpoint atap. Dengan menggunakan feedback loop dari sensor, PID controller secara kontinu menghasilkan sinyal kontrol yang sesuai, mengirimkannya ke servo motor melalui sinyal PWM untuk menggerakkan atap. Dengan cara ini, implementasi PID controller tidak hanya meningkatkan presisi dalam pengaturan posisi atap, tetapi juga membantu dalam menjaga kestabilan sistem secara keseluruhan, memberikan solusi yang efisien dan handal dalam manajemen atap otomatis pada berbagai kondisi cuaca dan lingkungan.

## Analisis Transient Respon

#### Initial Value
* Raindrops Sensor = ~1019
![image](https://hackmd.io/_uploads/BysTD65BA.png)
![image](https://hackmd.io/_uploads/Byh4tpcHA.png)
![image](https://hackmd.io/_uploads/HJ6LFa5HA.png)
![image](https://hackmd.io/_uploads/ryt5K69SR.png)
![image](https://hackmd.io/_uploads/SkUHc6cSA.png)

* Ultrasonic Sensor = ~11.17
![image](https://hackmd.io/_uploads/r16i9aqHR.png)
![image](https://hackmd.io/_uploads/rJhAsp9BR.png)
![image](https://hackmd.io/_uploads/rkY12a5B0.png)
![image](https://hackmd.io/_uploads/rJMx26qBC.png)
![image](https://hackmd.io/_uploads/rkH-2T9SR.png)

* Servo Motor = ~90
![image](https://hackmd.io/_uploads/ByUthp9SA.png)
![image](https://hackmd.io/_uploads/SkY-pa5HA.png)
![image](https://hackmd.io/_uploads/H1tzpaqSC.png)
![image](https://hackmd.io/_uploads/BkemppqHR.png)
![image](https://hackmd.io/_uploads/Bk64TpcrR.png)

### Percobaan Dengan PID Kp = Ki = Kd = 0
![image](https://hackmd.io/_uploads/rkgyexorC.png)

* Raindrops Sensor
    * Delay Time = |96.7192 - 96.7294| =  0.0102 s
    * Rise Time = |96.7192 - 96.7366| = 0.01648 s
    * Peak Time = |96.7192 - 97.8997| = 0.01978 s
    * Settling Time = |96.7808 - 97.6818| = 0.901 s
    ![image](https://hackmd.io/_uploads/Sk5mxxsr0.png)
    ![image](https://hackmd.io/_uploads/Byc4eeoBR.png)
    ![image](https://hackmd.io/_uploads/B1VLgxsS0.png)
    ![image](https://hackmd.io/_uploads/By3dgejS0.png)
    ![image](https://hackmd.io/_uploads/rJC5lljrC.png)
    ![image](https://hackmd.io/_uploads/HkSplxsB0.png)

    
* Ultrasonic Sensor
    * ![image](https://hackmd.io/_uploads/rJEI3yjBA.png)


* Servo Motor
    * ![image](https://hackmd.io/_uploads/S19AoksSR.png)
    * ![image](https://hackmd.io/_uploads/Hkb0s1oBR.png)



### Percobaan Dengan PID Kp = 0.05, Ki = 1.2, Kd = 0.01
![image](https://hackmd.io/_uploads/SyvKV6qSR.png)

* Raindrops Sensor
    * Delay Time = |97.6773 - 97.6892| =  0.0119 s
    * Rise Time = |97.6773 - 97.6969| = 0.0196 s
    * Peak Time = |97.6773 - 97.7017| = 0.0244 s
    * Settling Time = |97.7215 - 98.5613| = 0.8398 s
    ![image](https://hackmd.io/_uploads/Sy053AqH0.png)
    ![image](https://hackmd.io/_uploads/H1c1aRcBR.png)
    ![image](https://hackmd.io/_uploads/HJUalkiS0.png)
    ![image](https://hackmd.io/_uploads/rJMXaRqBR.png)
    ![image](https://hackmd.io/_uploads/BJS-pC9BR.png)
    ![image](https://hackmd.io/_uploads/HkNzCRqHC.png)
    ![image](https://hackmd.io/_uploads/H1JrRAqH0.png)
    
* Ultrasonic Sensor
    * ![image](https://hackmd.io/_uploads/rJEI3yjBA.png)


* Servo Motor
    * ![image](https://hackmd.io/_uploads/S19AoksSR.png)
    * ![image](https://hackmd.io/_uploads/Hkb0s1oBR.png)

    


    










<!-- 
Panduan Analisis (CLO4)
1 Plot respon transient Kondisi nilai KP, KI, dan KD yang seperti apa yang membuat sistem paling cepat stabil
2 Plot respon transient Kondisi nilai KP, KI, dan KD yang seperti apa yang membuat sistem memiliki error paling besar
3 Bandingkan set point dengan output hitung berapa besar steady state error (SSE)
4 Mana yang paling rentan menyebabkan sistem berosilasi? 
5 (Catatan: dikatakan berosilasi jika berulang kali bolak-balik melewati set point untuk mencapai set point yang diinginkan sampai berada di titik stabil.
6 Bagaimana respons transien sistem tersebut dari masing-masing kategori kecepatan yang diberikan? Hitung dengan menggunakan stopwatch/dari melihat serialplot dan berikan penjelasan dari masing-masing waktu berikut ini:
1.Delay time (Td)
2.Rise time (Tr)
3.Peak time (Tp)
4.Settling time (Ts)
 -->


## Analisis Mekanika
<!--
Panduan Analisis Mekanika (CLO 5)
-->
### Pengukuran Mekanik Gear/Motor/Servo/Stepper
Kami menggunakan motor servo Tower Pro SG90 untuk percobaan ini, yang dilengkapi dengan empat gear berbeda dengan pitch diameter yang sama, yaitu 1 cm, dan circular pitch sebesar 0,01 cm. Gear driver memiliki 32 gigi, sedangkan driven gear memiliki pilihan gigi sebanyak 39, 40, dan 48. Sebagai contoh, kami memilih rasio menggunakan gear driver 32 gigi dan driven gear 48 gigi.

Dalam penggunaan ini, rasio gear adalah 32:48 atau dapat disederhanakan menjadi 2:3. Jika gear kecil dipilih sebagai driver dan diputar sebesar 360 derajat, gear besar akan bergerak sejauh 240 derajat. Sebaliknya, jika gear besar dipilih sebagai driver dan diputar sebesar 360 derajat, gear kecil akan berputar sejauh 540 derajat.

Menurut pendapat kami, kami memilih gear besar sebagai driver dalam percobaan ini karena membutuhkan torsi yang lebih besar. Namun, keputusan ini dapat bervariasi tergantung pada kebutuhan sistem yang bersangkutan. Jika sebuah sistem lebih mengutamakan kecepatan daripada torsi, maka gear kecil mungkin lebih cocok untuk dipilih sebagai driver.

### Stabilitas dan Kontrol Presisi dengan PID Controller
Implementasi PID controller sangat penting untuk menjaga stabilitas dan presisi dalam menggerakkan atap. PID controller berfungsi untuk mengatur sinyal PWM yang dikirim ke servo motor berdasarkan umpan balik dari sensor ultrasonik. Ini memastikan bahwa atap berhenti tepat pada posisi yang diinginkan tanpa overshoot atau osilasi berlebihan.
1. Parameter PID
    * P (Proportional): Mengurangi error besar dengan menyesuaikan output secara proporsional terhadap error.
    * I (Integral): Mengatasi error kumulatif dari masa lalu, mengoreksi kesalahan yang berulang.
    * D (Derivative): Menanggapi perubahan cepat dalam error, meningkatkan stabilitas sistem dengan mengurangi osilasi.

2. Feedback Loop
    * Sensor ultrasonik mengukur jarak antara atap dan sensor, memberikan umpan balik real-time ke mikrocontroller.
    * PID controller memproses informasi ini dan mengatur sinyal PWM yang mengontrol posisi servo motor
    
### Desain Mekanik dan Pengaruh Beban
Desain mekanik sistem ini harus mempertimbangkan beban dinamis seperti angin dan hujan yang dapat mempengaruhi kinerja atap. Rel atau engsel harus dilumasi dengan baik untuk mengurangi gesekan dan memastikan gerakan yang halus. Pemilihan material yang tepat untuk atap, seperti aluminium atau plastik tahan cuaca, juga penting untuk mengurangi beban dan meningkatkan efisiensi sistem.
1. Distribusi Beban
    * Desain atap harus memastikan distribusi beban yang merata untuk menghindari keausan tidak merata dan memastikan umur panjang komponen.

2. Beban Dinamis
    * Sistem harus mampu mengatasi beban dinamis yang tidak terduga. Gear dengan torsi yang lebih besar membantu memastikan bahwa sistem tetap berfungsi meskipun menghadapi beban eksternal seperti angin kencang atau hujan deras.
<!--
Nyalakan motor listrik tanpa beban.
1.Hubungkan beban dengan motor (sesuai kasus project). Apakah motor masih dapat berputar?
2.Tambahkan lagi beban secara bertahap hingga motor berhenti berputar. Lakukan analisis hubungan antara torsi dan kecepatan dari setiap beban yang ditambah hingga motor berhenti berputar.
3.Jika beban terus ditambahkan dan menyebabkan motor berhenti berputar, apakah beban tersebut tetap dapat diputar dengan menggunakan gear? Jelaskan! 
4.Bagaimana mengatur komposisi gear agar beban tersebut dapat berputar? Mengapa beban menjadi dapat berputar setelah ditambahkan rangkaian gear?

Jelaskan hal-hal apa saja yang terjadi saat beban motor meningkat, khususnya dilihat dari:
1.kecepatan, 
2.CEMF, 
3.arus (current), 
4.torsi.
-->
## Hasil dan Saran
### Hasil
Proyek sistem atap otomatis ini berhasil dirancang dan diuji dengan menggunakan PID controller untuk mengendalikan servo motor. Sistem ini dilengkapi dengan sensor ultrasonik, dan sensor air hujan untuk mendeteksi kondisi lingkungan dan menggerakkan atap sesuai dengan kondisi cuaca.
![WhatsApp Image 2024-06-15 at 10.35.21_6b676b38](https://hackmd.io/_uploads/BJhbhFqSC.jpg)
<br>


### Saran
Untuk pengembangan lebih lanjut, kami menyarankan beberapa hal. Pertama, penggunaan material atap yang lebih ringan namun tahan lama dapat mengurangi beban pada servo motor dan meningkatkan efisiensi sistem secara keseluruhan. Kedua, integrasi lebih lanjut dengan sistem smart home lainnya, seperti pengaturan suhu, dapat memberikan manfaat tambahan dalam hal kenyamanan dan efisiensi energi. Ketiga, pengujian dan kalibrasi lebih lanjut terhadap parameter PID diperlukan untuk mengatasi berbagai kondisi cuaca ekstrem dan memastikan performa optimal sepanjang waktu. Terakhir, evaluasi berkala terhadap komponen mekanis dan elektronik perlu dilakukan untuk memastikan sistem tetap berfungsi dengan baik dan mencegah kerusakan yang tidak diinginkan.
## Lampiran 
### Video Demo
<iframe width="720" height="315" src="https://www.youtube.com/embed/bVW52yf3U_Q?si=pxUOC4o_neDm_v1w" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

### Source Code
[Repository Link](https://github.com/szyxxx/Arduino-Automated-Roof)
```
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
float Kp = 0.02;
float Ki = 0.1;
float Kd = 0.00001;

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
        targetPosition = constrain(targetPosition - 1, 90, servoPosition);
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

```


## Referensi
[Automatic Sliding Door](https://www.youtube.com/watch?v=WFBnl30g7fo&pp=ygUeYXV0b21hdGljIHNsaWRpbmcgZG9vciBhcmR1aW5v)
[Rain Sensor Arduino](https://www.youtube.com/watch?v=H62xzxI-4A0&pp=ygUTcmFpbiBzZW5zb3IgYXJkdWlubw%3D%3D)

## Rubrik Penilaian

| Penilaian Indikator Ketercapaian CLO | Bobot |
| -------- | -------- |
| Mahasiswa mampu mengevaluasi dan optimasi sistem kendali PID (soal CLO 4).     | 50 %     |
|Mahasiswa mampu mendemonstrasikan sistem mekanisme transfer daya sederhana menggunakan gear dan motor DC (soal CLO 5)|50%|

### Kriteria Nilai
| 4 | 3 | 2 | 1 | 0 |
| ----- | ----- | ------ | ----- | ---- |
|      CLO 4  |       |      | | |
Mampu menjelaskan konsep kendali umpan balik, merancang sistem kendali PID, hingga mengevaluasi dan optimasi sistem kendali PID.|	Mampu menjelaskan konsep kendali umpan balik, merancang, dan mengevaluasi sistem kendali PID.|Mampu menjelaskan konsep kendali umpan balik dan merancang sistem kendali PID.|Mampu menjelaskan konsep kendali umpan balik dan PID, tetapi kesulitan dalam merancang dan mengevaluasi sistem kendali PID.|	Kesulitan dalam menjelaskan konsep kendali umpan balik dan PID.|
| CLO 5 |     | |  |
Mampu mendapatkan fungsi transfer sistem dari mekanisme transfer daya, menganalisis hubungan antara torsi dan kecepatan motor, menjelaskan cara kerja dan karakteristik motor listrik, serta mendemonstrasikan sistem mekanisme transfer daya sederhana menggunakan gear dan motor DC.|	Mampu mendapatkan fungsi transfer sistem dari mekanisme transfer daya, menganalisis hubungan antara torsi dan kecepatan motor, menjelaskan cara kerja dan karakteristik motor listrik, tetapi kesulitan dalam mendemonstrasikan sistem mekanisme transfer daya sederhana menggunakan gear dan motor DC.	|Mampu mendapatkan fungsi transfer sistem dari mekanisme transfer daya, menganalisis hubungan antara torsi dan kecepatan motor, tetapi kesulitan dalam menjelaskan cara kerja dan karakteristik motor listrik.	| Mampu mendapatkan fungsi transfer sistem dari mekanisme transfer daya dan menganalisis hubungan antara torsi dan kecepatan motor.	|Tidak dapat menentukan satu langkah pun untuk menjelaskan mengenai mekanisme transfer daya.|





