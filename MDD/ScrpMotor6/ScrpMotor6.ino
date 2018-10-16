#include <ScrpSlave.h>
#include <Utility.h>
#include <EEPROM.h>

void changeID(byte new_id) {
  EEPROM.write(0, new_id);
}

constexpr int Motor[3][3] = {
  {5, 6, 12}, 
  {10, 11, 13}, 
  {9, 3, 2}
};
constexpr long BaudRate = 115200, RedePin = 4;

ScrpSlave slave(RedePin, EEPROM.read(0), changeID);

constexpr int Spin = 30;
constexpr int DelaySolenoid = 250, DelayLoad = 500, DelayArm = 1000, DelayHand = 550;
int delayShoot = 310;
constexpr int LimitFall = A3, LimitCatch = A2, LimitArm = A1, Solenoid[2] = {10, 11}, Arm = 3, Hand = 9, Magnet = 7;
unsigned long now = millis();
unsigned long prevUp = now, prevDown = now, prevArm = now, prevHand = now, prevShoot = now;
boolean loadable = false;
boolean flagFB = false, flagHand = false;
boolean shootable = true, order = false;
boolean nowCatch = false, prevCatch = false;
int phase = 6;

void setup() {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 2; ++j) {
      pinMode(Motor[i][j], OUTPUT);
      setPWMFrequency(Motor[i][j], PWM_DIV64);
    }
    pinMode(Motor[i][2], OUTPUT);
  }
  pinMode(RedePin, OUTPUT);
  pinMode(Magnet, OUTPUT);
  Serial.begin(BaudRate);
  slave.addCMD(2, driveMtr1);
  slave.addCMD(3, driveMtr2);
  slave.addCMD(4, driveMtr3);
  slave.addCMD(10, checker);
  slave.addCMD(11, calibration);
}

void loop() {
  slave.check();
  now = millis();

  // Act
  switch (phase) {
    case 0:
      driveMtr(Spin, 0);
      break;

    case 1:
      driveMtr(0, 0);
      digitalWrite(Arm, 0);
      digitalWrite(Magnet, 1);
      break;

    case 2:
      driveMtr(Spin, 0);
      break;

    case 3:
      driveMtr(0, 0);
      digitalWrite(Hand, 0);
      break;

    case 4:
      driveMtr(Spin, 0);
      break;

    case 5:
      driveMtr(0, 0);
      digitalWrite(Solenoid[flagFB], 1);
      digitalWrite(Magnet, 0);
      break;

    case 6:
      digitalWrite(Solenoid[flagFB], 0);
      break;
  }

  // Judge
  if (phase == 0 && digitalRead(LimitArm)) {
    prevArm = now;
    phase = 1;
  }
  else if (phase == 2 && digitalRead(LimitCatch)) {
    prevHand = now;
    phase = 3;
  }
  else if (phase == 1 && now - prevArm > DelayArm) {
    phase = 2;
  }
  else if (phase == 3 && now - prevHand > DelayHand) {
    phase = 4;
  }
  else if (phase == 4 && digitalRead(LimitFall)) {
    prevUp = now;
    loadable = false;
    shootable = true;
    phase = 5;
  }
  else if (phase == 5 && now - prevUp > DelaySolenoid) {
    prevDown = now;
    phase = 6;
  }
  else if (phase == 6 && loadable && now - prevDown > DelayLoad) {
    phase = 0;
    flagFB = !flagFB;
  }

  if (order) {
    digitalWrite(Arm, 1);
    shootable = false;
    loadable = true;
    order = false;
    prevShoot = now;
    flagHand = true;
  }
  else if (now - prevShoot > delayShoot && flagHand) {
    digitalWrite(Hand, 1);
    flagHand = false;
  }
}

inline boolean driveMtr(int rx_data, int num) {
  rx_data = constrain(rx_data, -240, 240);
  if (!rx_data) {
    digitalWrite(Motor[num][0], LOW);
    digitalWrite(Motor[num][1], LOW);
    digitalWrite(Motor[num][2], LOW);
  }
  else if (0 < rx_data) {
    analogWrite(Motor[num][0], rx_data);
    digitalWrite(Motor[num][1], LOW);
    digitalWrite(Motor[num][2], HIGH);
  }
  else {
    digitalWrite(Motor[num][0], LOW);
    analogWrite(Motor[num][1], -rx_data);
    digitalWrite(Motor[num][2], HIGH);
  }
  return true;
}

boolean driveMtr1(int rx_data, int& tx_data) {
  return driveMtr(rx_data, 0);
}

boolean driveMtr2(int rx_data, int& tx_data) {
  return driveMtr(rx_data, 1);
}

boolean driveMtr3(int rx_data, int& tx_data) {
  return driveMtr(rx_data, 2);
}

boolean checker(int rx_data, int& tx_data) {
  if (rx_data != 0) {
    order = shootable;
  }
  else {
    order = false;
  }
  if (order) {
    delayShoot = rx_data;
  }
  return shootable;
}

boolean calibration(int rx_data, int& tx_data) {
  digitalWrite(Hand, 1);
  delay(rx_data);
  digitalWrite(Arm, 1);
  delay(rx_data);
  digitalWrite(Arm, 0);
  delay(rx_data);
  digitalWrite(Hand, 0);
  return true;
}
