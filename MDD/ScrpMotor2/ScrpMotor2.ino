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
constexpr int URTRIG = 9;
constexpr int ReadPin = A0;

ScrpSlave slave(RedePin, EEPROM.read(0), changeID);

void setup() {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 2; ++j) {
      pinMode(Motor[i][j], OUTPUT);
      setPWMFrequency(Motor[i][j], PWM_DIV64);
    }
    pinMode(Motor[i][2], OUTPUT);
  }
  Serial.begin(BaudRate);
  slave.addCMD(2, driveMtr1);
  slave.addCMD(3, driveMtr2);
  slave.addCMD(4, driveMtr3);
  slave.addCMD(20, ultraDist);
  slave.addCMD(255, safeOperation);

  pinMode(URTRIG, OUTPUT);
  digitalWrite(URTRIG, HIGH);
}

int dist = 0;
int sensorValue = 0;
unsigned long prev_time = millis();
void loop() {
  slave.check();
  if (millis() - prev_time > 100) {
    digitalWrite(URTRIG, LOW);
    digitalWrite(URTRIG, HIGH);
    sensorValue = analogRead(ReadPin);
    if (sensorValue <= 10) {
      dist = 512;
    } else {
      dist = sensorValue * 0.718;
    }
    prev_time = millis();
  }
}

boolean safeOperation(int rx_data, int& tx_data) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      digitalWrite(Motor[i][j], LOW);
    }
  }
  return true;
}

inline boolean driveMtr(int rx_data, int num) {
  rx_data = constrain(rx_data, -240, 240);
  if (!rx_data) {
    digitalWrite(Motor[num][0], LOW);
    digitalWrite(Motor[num][1], LOW);
    digitalWrite(Motor[num][2], LOW);
  }
  else if (0 < rx_data) {
    digitalWrite(Motor[num][0], rx_data);
    analogWrite(Motor[num][1], LOW);
    digitalWrite(Motor[num][2], HIGH);
  }
  else {
    digitalWrite(Motor[num][0], LOW);
    digitalWrite(Motor[num][1], -rx_data);
    digitalWrite(Motor[num][2], HIGH);
  }
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

boolean ultraDist(int rx_dara, int& tx_data) {
  tx_data = dist;
  return true;
}

