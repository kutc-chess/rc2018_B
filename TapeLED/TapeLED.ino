#include <Adafruit_NeoPixel.h>
#include <ScrpSlave.h>
#include <Utility.h>
#include <EEPROM.h>

void changeID(byte new_id) {
  EEPROM.write(0, new_id);
}

//const number

constexpr int Motor[3][3] = {
  {5, 6, 12}, 
  {10, 11, 13}, 
  {9, 3, 2}
};
constexpr int Tape[2] = {10, 11};
constexpr int LEDs = 15;
constexpr int BAUDRATE = 115200, REDE_PIN = 4;

// Adafruit_NeoPixel leftPixels = Adafruit_NeoPixel(LEDs, 10, NEO_GRB + NEO_KHZ800);
// Adafruit_NeoPixel rightPixels = Adafruit_NeoPixel(LEDs, 11, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel Pixels[2] = {
  Adafruit_NeoPixel(LEDs, 10, NEO_GRB + NEO_KHZ800), // Left
  Adafruit_NeoPixel(LEDs, 11, NEO_GRB + NEO_KHZ800)  // Right
};
/*
uint32_t red24 = Pixels[0].Color(0xFF, 0, 0);
uint32_t redNormal = Pixels[0].Color(0xFF, 0x69, 0xB4);
uint32_t blue24 = Pixels[0].Color(0, 0, 0xFF);
uint32_t blueNormal = Pixels[0].Color(0x7F, 0xFF, 0xD4);
*/
uint32_t colors[2][2] = {
  {Pixels[0].Color(0xFF, 0, 0), Pixels[0].Color(0xFF, 0x69, 0xB4)}, // 2.4 & Other in RedZone
  {Pixels[0].Color(0, 0, 0xFF), Pixels[0].Color(0x7F, 0xFF, 0xD4)}  // 2.4 & Other in BlueZone
};

boolean zone, arm, table, doing;
int stage = 0, interval = 500, stepLED = 0;

unsigned int stageEnd = 0;

ScrpSlave slave(REDE_PIN, EEPROM.read(0), changeID);

void setup() {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 2; ++j) {
      pinMode(Motor[i][j], OUTPUT);
      setPWMFrequency(Motor[i][j], PWM_DIV64);
    }
    pinMode(Motor[i][2], OUTPUT);
  }
  pinMode(REDE_PIN, OUTPUT);
  Serial.begin(BAUDRATE);
  slave.addCMD(2, driveMtr1);
  slave.addCMD(3, driveMtr2);
  slave.addCMD(4, driveMtr3);
  slave.addCMD(30, changeLEDFlag);
  slave.addCMD(255, safeOperation);
  Pixels[0].begin();
  Pixels[1].begin();
  Pixels[0].show(); // Set all LED to OFF
  Pixels[1].show();
}

void loop() {
  slave.check();
  if (doing) {
    if (stage < 1) {
      onLED();
    }
    if (stage < 2 && millis() - stageEnd >= 5000) {
      offLED(0);
    }
  }
  // digitalWrite(9, HIGH);
}

void onLED() {
  static unsigned long preLED = 0;
  static int stepLED = 0;
  if (millis() - preLED >= interval) {
    Pixels[arm].setPixelColor(stepLED, colors[zone][table]);
    Pixels[arm].show();
    preLED = millis();
    stepLED++;
  }
  if (stepLED >= LEDs) {
    stageEnd = millis();
    stage++;
    preLED = 0;
    stepLED = 0;
  }
}

void offLED(int pattern){
  static unsigned long preLED = 0;
  static int stepLED = 0;
  if (pattern == 0) { // All off
    for (int i = 0; i < LEDs; ++i){
      Pixels[arm].setPixelColor(i, Pixels[arm].Color(0, 0, 0));
    }
    Pixels[arm].show();
    doing = 0;
    stage = 0;
  }else{ // Wipe off
    if (millis() - preLED >= interval) {
      Pixels[arm].setPixelColor(stepLED, Pixels[arm].Color(0, 0, 0));
      Pixels[arm].show();
      preLED = millis();
      stepLED++;
    }
    if (stepLED >= LEDs) {
      preLED = 0;
      stepLED = 0;
      doing = 0;
      stage = 0;
    }
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

boolean changeLEDFlag(int rx_data, int& tx_data) {
  // RED(1) or BLUE(0)?
  zone = 0b1 & rx_data;
  // LEFT(1) or RIGHT(0)?
  arm = 0b10 & rx_data;
  // 2.4mTable(1) or others(0)?
  table = 0b100 & rx_data;
  // Start shine? (Yes:1, No:0)
  doing = 0b1000 & rx_data;
  return true;
}

boolean changeLEDInterval(int rx_data, int& tx_data) {
  // change LEDwipe interval(ms)
  interval = rx_data / 15;
  return true;
}

