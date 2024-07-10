#include <Arduino.h>
#include <TB6612_ESP32.h>
#include <BluetoothSerial.h>


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Motor control pins for TB6612FNG || Pinouts
#define lms 32  //vm    pwma
#define lmb 33  //vcc   ain2
#define lmf 25  //gnd   ain1
#define stb 26  //ao1   stb
#define rmf 14  //ao2   bin1
#define rmb 27  //bo2   bin2
#define rms 12  //bo1   pwmb // last 2 are gnd
// Motor declaration
const int offsetA = 1;
const int offsetB = 1;
Motor motor1 = Motor(rmf, rmb, rms, offsetA, stb, 5000, 8, 2);  //right -> A
Motor motor2 = Motor(lmf, lmb, lms, offsetB, stb, 5000, 8, 1);  //left -> B

BluetoothSerial SerialBT;

// IR sensor pins & variables
const int irSensorPins[] = { 16, 17, 5, 18, 19 };

int s[5];
float avg, setpoint = 3;
float sensor, sum = 0;
short base[5] = { 1, 2, 4, 8, 16 };
short position[5] = { 1, 2, 3, 4, 5 };
int threshold = 512;

float Kp = 90;
float Ki = 0.15;
float Kd = 81;

uint8_t multiP = 1;
uint8_t multiI = 1;
uint8_t multiD = 1;
uint8_t Kpfinal;
uint8_t Kifinal;
uint8_t Kdfinal;
float Pvalue;
float Ivalue;
float Dvalue;

boolean onoff = false;

int val, cnt = 0, v[3];

uint16_t position2;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 200;

// sharp turn
char turn = 's';
int tsp = 250;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("esp32", false);
  Serial.println("Bluetooth Started! Ready to pair...");
}

void loop() {
  // if (SerialBT.available()) {
  //   while (SerialBT.available() == 0)
  //     ;
  //   valuesread();
  //   processing();
  // }
  // if (onoff == true) {
  semi_pid();
  // } else if (onoff == false) {
  //   motor1.brake();
  //   motor2.brake();
  // }
}

void reading() {
  sensor = 0;
  sum = 0;
  for (int i = 0; i <= 4; i++) {
    s[i] = digitalRead(irSensorPins[i]);
    (s[i] == 0) ? s[i] = 1 : s[i] = 0;
    // Serial.print(String(s[i]) + " ");
    sensor += s[i] * base[i];
    sum += s[i];
  }
  // Serial.println();
}

void PID_reading() {
  sensor = 0;
  sum = 0;
  for (byte i = 0; i <= 4; i++) {
    s[i] = digitalRead(irSensorPins[i]);
    (s[i] == 0) ? s[i] = 1 : s[i] = 0;
    sensor += s[i] * position[i];
    sum += s[i];
  }
  // avg out to find error
  if (sum) avg = sensor / sum;
}

void semi_pid() {
  delay(1000);
  // Manual control 1
  reading();

  if (sum == 0) {
    if (turn != 's') {
      delay(40);
      (turn == 'r') ? right(motor2, motor1, tsp) : left(motor2, motor1, tsp);
      // while (!(s[1] + s[2] + s[3])) reading();
      while (!s[2]) reading();
      turn = 's';
    }
  }

  // PID control
  PID_reading();
  error = setpoint - avg;
  PID_Linefollow(error);

  // // Manual control 2
  if (!s[0] && (s[3] + s[4])) {
    delay(40);
    turn = 'r';
  }
  if (!s[4] && (s[0] + s[1])) {
    delay(40);
    turn = 'l';
  }

  if (sum == 5) {
    delay(10);
    reading();
    if (sum == 5) {
      brake(motor1, motor2);
      while (sum == 5) reading();
    } else if (sum == 0) turn = 'r';
  }
}

void PID_Linefollow(int error) {
  P = error;
  I = I + error;
  D = error - previousError;

  Pvalue = (Kp * P);
  Ivalue = (Ki * I) / 100;
  Dvalue = (Kd * D);

  float PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  // Serial.print("P = " + String(Pvalue) + " I = " + String(Ivalue) + " D = " + String(Dvalue));
  // Serial.println();

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  // Speed limit
  if (lsp > 255)
    lsp = 255;
  if (lsp < -255)
    lsp = -255;
  if (rsp > 255)
    rsp = 255;
  if (rsp < -255)
    rsp = -255;

  motor1.drive(rsp);
  motor2.drive(lsp);
}

//The  Arduino knows that for each instruction it will receive 2 bytes.
// void valuesread() {
//   val = SerialBT.read();
//   cnt++;
//   v[cnt] = val;
//   if (cnt == 2)
//     cnt = 0;
// }

// // //In this void the 2 read values are assigned.
// void processing() {
//   int a = v[1];
//   if (a == 1) {
//     Kp = v[2];
//   }
//   if (a == 2) {
//     multiP = v[2];
//   }
//   if (a == 3) {
//     Ki = v[2];
//   }
//   if (a == 4) {
//     multiI = v[2];
//   }
//   if (a == 5) {
//     Kd = v[2];
//   }
//   if (a == 6) {
//     multiD = v[2];
//   }
//   if (a == 7) {
//     onoff = v[2];
//   }
// }