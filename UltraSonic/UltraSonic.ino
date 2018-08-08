
#include <ScrpSlave.h>
#include <Utility.h>
#include <EEPROM.h>
#include "config.h"

void changeID(byte new_id) {
  EEPROM.write(0, new_id);
}

ScrpSlave slave(REDE_PIN, EEPROM.read(0), changeID);

void setup() {
  pinMode(REDE_PIN, OUTPUT);
  pinMode(MTR1_FWD, OUTPUT);
  pinMode(MTR1_RVS, OUTPUT);
  pinMode(MTR2_FWD, OUTPUT);
  pinMode(MTR2_RVS, OUTPUT);
  pinMode(MTR1_LED, OUTPUT);
  pinMode(MTR2_LED, OUTPUT);
  pinMode(MTR3_LED, OUTPUT);
  setPWMFrequency(MTR1_FWD, PWM_DIV8); //7812.5Hz
  setPWMFrequency(MTR1_RVS, PWM_DIV8);
  setPWMFrequency(MTR2_FWD, PWM_DIV8); //3906.25Hz
  setPWMFrequency(MTR2_RVS, PWM_DIV8);
  setPWMFrequency(MTR3_FWD, PWM_DIV8);
  setPWMFrequency(MTR3_RVS, PWM_DIV8);
  Serial.begin(BAUDRATE);
  slave.addCMD(2, driveMtr1);
  slave.addCMD(3, driveMtr2);
  slave.addCMD(4, driveMtr3);
  slave.addCMD(20, ultraDist);
  slave.addCMD(255, safeOperation);

  pinMode(URTRIG,OUTPUT);
  digitalWrite(URTRIG,HIGH);
  pinMode(URECHO, INPUT);
}

int sensorMode = 0;
int sensorValue = 0;
int dist = 0;
unsigned long prev_time = millis();
void loop() {
  slave.check();
  if(millis() - prev_time > 100){
    digitalWrite(URTRIG, LOW);
    digitalWrite(URTRIG, HIGH);
    if(sensorMode == 0){
      unsigned long lowLevelTime = pulseIn(URECHO, LOW);
      if(lowLevelTime>=45000){
        dist = 512;
      }
      else{
        dist = lowLevelTime / 50;
      }
    }
    else{
      sensorValue = analogRead(READPIN);
      if(sensorValue <= 10){
        dist = 512;
      }else{
        dist = sensorValue * 0.718;
      }
    }
    prev_time = millis();
  }
}

boolean safeOperation(int rx_data, int& tx_data) {
  digitalWrite(MTR1_FWD, LOW);
  digitalWrite(MTR1_RVS, LOW);
  digitalWrite(MTR2_FWD, LOW);
  digitalWrite(MTR2_RVS, LOW);
  digitalWrite(MTR3_FWD, LOW);
  digitalWrite(MTR3_RVS, LOW);
  digitalWrite(MTR1_LED, LOW);
  digitalWrite(MTR2_LED, LOW);
  digitalWrite(MTR3_LED, LOW);
  return true;
}

boolean driveMtr1(int rx_data, int& tx_data) {
  rx_data = constrain(rx_data, -250, 250);
  if (!rx_data) {
    digitalWrite(MTR1_FWD, LOW);
    digitalWrite(MTR1_RVS, LOW);
    digitalWrite(MTR1_LED, LOW);
  } else if (0 < rx_data) {
    digitalWrite(MTR1_RVS, LOW);
    analogWrite(MTR1_FWD, rx_data);
    digitalWrite(MTR1_LED, HIGH);
  } else {
    digitalWrite(MTR1_FWD, LOW);
    analogWrite(MTR1_RVS, -rx_data);
    digitalWrite(MTR1_LED, HIGH);
  }
  return true;
}

boolean driveMtr2(int rx_data, int& tx_data) {
  rx_data = constrain(rx_data, -250, 250);
  if (!rx_data) {
    digitalWrite(MTR2_FWD, LOW);
    digitalWrite(MTR2_RVS, LOW);
    digitalWrite(MTR2_LED, LOW);
  } else if (0 < rx_data) { //-250
    digitalWrite(MTR2_RVS, LOW);
    analogWrite(MTR2_FWD, rx_data);
    digitalWrite(MTR2_LED, HIGH);
  } else {
    digitalWrite(MTR2_FWD, LOW);
    analogWrite(MTR2_RVS, -rx_data);
    digitalWrite(MTR2_LED, HIGH);
  }
  return true;
}

boolean driveMtr3(int rx_data, int& tx_data) {
  rx_data = constrain(rx_data, -250, 250);
  if (!rx_data) {
    digitalWrite(MTR3_FWD, LOW);
    digitalWrite(MTR3_RVS, LOW);
    digitalWrite(MTR3_LED, LOW);
  } else if (0 < rx_data) {
    digitalWrite(MTR3_RVS, LOW);
    analogWrite(MTR3_FWD, rx_data);
    digitalWrite(MTR3_LED, HIGH);
  } else {
    digitalWrite(MTR3_FWD, LOW);
    analogWrite(MTR3_RVS, -rx_data);
    digitalWrite(MTR3_LED, HIGH);
  }
  return true;
}

boolean ultraDist(int rx_dara, int& tx_data) {
  tx_data = dist;
  return true;
}

