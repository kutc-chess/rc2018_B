#include <ScrpSlave.h>
#include <Utility.h>
#include <EEPROM.h>
#include "config.h"
#define Limit_catch A1
#define Limit_fall A0
#define Arm 3
#define Hand 9
#define SolenoidF 10
#define SolenoidB 11
#define slit 7
#define Motor 200

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
  pinMode(MTR3_FWD, OUTPUT);
  pinMode(MTR3_RVS, OUTPUT);
  pinMode(MTR1_LED, OUTPUT);
  pinMode(MTR2_LED, OUTPUT);
  pinMode(MTR3_LED, OUTPUT);
  setPWMFrequency(MTR1_FWD, PWM_DIV32); //7812.5Hz
  setPWMFrequency(MTR1_RVS, PWM_DIV32);
  setPWMFrequency(MTR2_FWD, PWM_DIV32); //3906.25Hz
  setPWMFrequency(MTR2_RVS, PWM_DIV32);
  setPWMFrequency(MTR3_FWD, PWM_DIV32);
  setPWMFrequency(MTR3_RVS, PWM_DIV32);
  Serial.begin(BAUDRATE);
  slave.addCMD(2, driveMtr1);
  slave.addCMD(3, driveMtr2);
  slave.addCMD(4, driveMtr3);
  slave.addCMD(255, safeOperation);
  slave.addCMD(10,acceptance);
}
boolean flag_fallable = false; 
boolean flag_shotable = true;
boolean flag_arm_fallable = false;
boolean flag_seize = false;
boolean flag_order = false;
boolean flag_FB = false;
boolean flag_ch = false;
unsigned long prev_solenoid = millis();
unsigned long prev_motor = millis();
unsigned long prev_func = millis();
unsigned long now;


void loop() {
  now = millis();
  //Start Fall
  if(digitalRead(Limit_fall) && flag_fallable){
    prev_solenoid = now;
    solenoid(flag_ch);
    flag_ch = !flag_ch;
    flag_shotable = false;
  }
  //Finish Fall
  //Start Load
  if(now - prev_solenoid > 500 && !flag_order){
    driveMtr(Motor);
    if(digitalRead, Limit_catch){
     if(digitalRead, !Limit_catch){
      driveMtr(0);
     }
    }
    flag_arm_fallable = true;
  }
  //Start Arm Fall
  if(flag_arm_fallable){
    digitalWrite(Arm, 0);
    flag_fallable = false;
    driveMtr(Motor);
    if(digitalRead(Limit_catch)){
      if(digitalRead(!Limit_catch)){
        driveMtr(0); 
      }
    }
    digitalWrite(Hand, 0);
    flag_fallable = false;
    flag_shotable = true;
    prev_motor = millis();
  }
  //Finish Arm Fall
  if(now - prev_motor > 5000){
    driveMtr(Motor);
  }
  if(digitalRead(Limit_fall)){
    driveMtr(0);
  }
  //Finish Load
  if(Limit_fall && !flag_fallable){
    flag_fallable = true;
  }
  if(flag_order){
    digitalWrite(Arm, 1);
    flag_shotable = false;
    if(slit){
      digitalWrite(Hand, 1);
      flag_order = false;
    }
  }
  slave.check();
}

boolean acceptance(int rx_data,int& tx_data){
  tx_data = flag_shotable;
  flag_order = flag_shotable;
  return true;
}

void solenoid(boolean level){
  if(flag_FB){
    digitalWrite(SolenoidF, level);
    flag_FB = false;
  }
  else{
    digitalWrite(SolenoidB, level);
    flag_FB = true;
  }
}
/*
  prev_func = millis();
  if(millis() - prev_func > 5000){
    if(flag){
      digitalWrite(!SolenoidF, 0);
    }
    else{
      digitalWrite(!SolenoidB, 0);
    }
    flag = !flag
    */
void driveMtr(int rx_data) {
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
