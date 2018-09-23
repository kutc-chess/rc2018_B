#include <ScrpSlave.h>
#include <Utility.h>
#include <EEPROM.h>
#include "config.h"

void changeID(byte new_id) {
  EEPROM.write(0, new_id);
}

//const number
ScrpSlave slave(REDE_PIN, EEPROM.read(0), changeID);

void setup() {
  pinMode(REDE_PIN, OUTPUT);
  pinMode(MTR1_FWD, OUTPUT);
  pinMode(MTR1_RVS, OUTPUT);
  pinMode(MTR2_FWD, OUTPUT);
  pinMode(MTR2_RVS, OUTPUT);
  pinMode(MTR3_FWD, OUTPUT);
  pinMode(MTR3_RVS, OUTPUT);
  pinMode(MTR1_LED, OUTPUT);
  pinMode(MTR2_LED, OUTPUT);
  pinMode(MTR3_LED, OUTPUT);
  setPWMFrequency(MTR1_FWD, PWM_DIV64); //7812.5Hz
  setPWMFrequency(MTR1_RVS, PWM_DIV64);
  setPWMFrequency(MTR2_FWD, PWM_DIV64); //3906.25Hz
  setPWMFrequency(MTR2_RVS, PWM_DIV64);
  setPWMFrequency(MTR3_FWD, PWM_DIV64);
  setPWMFrequency(MTR3_RVS, PWM_DIV64);
  Serial.begin(BAUDRATE);
  slave.addCMD(2, driveMtr1);
  slave.addCMD(3, driveMtr2);
  slave.addCMD(4, driveMtr3);
  slave.addCMD(10, checker);
}

constexpr int Spin = 30;
constexpr int DelaySolenoid = 250, DelayLoad = 500, DelayArm = 1000, DelayHand = 250, DelayShoot = 310;
constexpr int LimitFall = A2, LimitCatch = A3, LimitArm = A0, Solenoid[2] = {10, 11}, Arm = 3, Hand = 9;
unsigned long now = millis();
unsigned long prevUp = now, prevDown = now, prevArm = now, prevHand = now, prevShoot = now;
boolean loadable = false;
boolean flagFB= false, flagHand = false;
boolean shootable = true, order = false;
boolean nowCatch = false, prevCatch = false;
int phase = 6;

void loop() {
  slave.check();
  now = millis();

  // Act
  switch(phase){
    case 0:
    driveMtr(Spin);
    break;
    
    case 1:
    driveMtr(0);
    digitalWrite(Arm, 0);
    break;
    
    case 2:
    driveMtr(Spin);
    break;
    
    case 3:
    driveMtr(0);
    digitalWrite(Hand, 0);
    break;
    
    case 4:
    driveMtr(Spin);
    break;
    
    case 5:
    driveMtr(0);
    digitalWrite(Solenoid[flagFB], 1);
    break;
    
    case 6:
    digitalWrite(Solenoid[flagFB], 0);
    break;
  }

  // Judge
  if(phase == 0 && digitalRead(LimitArm)){
    prevArm = now;
    phase = 1;
  }
  else if(phase == 2 && digitalRead(LimitCatch)){
    prevHand = now;
    phase = 3;
  }
  else if(phase == 1 && now - prevArm > DelayArm){
    phase = 2;
  }
  else if(phase == 3 && now- prevHand > DelayHand){
    phase = 4;
  }
  else if(phase == 4 && digitalRead(LimitFall)){
    prevUp = now;
    loadable = false;
    shootable = true;
    phase = 5;
  }
  else if(phase == 5 && now - prevUp > DelaySolenoid){
    prevDown = now;
    phase = 6;
  }
  else if(phase == 6 && loadable && now - prevDown > DelayLoad){
    phase = 0;
    flagFB = !flagFB;
  }

  if(order){
    digitalWrite(Arm, 1);
    shootable = false;
    loadable = true;
    order = false;
    prevShoot = now;
    flagHand = true;
  }
  else if(now - prevShoot > DelayShoot && flagHand){
    digitalWrite(Hand, 1);
    flagHand = false;
  }

}

boolean checker(int rx_data, int& tx_data){
  order = shootable & rx_data;
  return shootable;
}

boolean driveMtr(int rx_data) {
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
