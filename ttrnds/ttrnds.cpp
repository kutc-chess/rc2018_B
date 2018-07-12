//立ルンですのプログラム, MainLoop from 77
#include "/home/pi/PigpioMS/PigpioMS.hpp"
#include "/home/pi/RasPiDS3/RasPiDS3.hpp"
#include "/home/pi/Sensor/GY521/GY521.hpp"
#include <algorithm>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#define ROOT3 (1.7320508)

using namespace std;
using namespace RPDS3;
using namespace RPMS;
using namespace RPGY521;

int main(void) {
  MotorSerial ms;
  DualShock3 Controller;
  // MDD通信セットアップ
  try {
    ms.init();
  } catch (const char *str) {
    return -1;
  }

  //----------etc.----------
  // Check LED
  constexpr int BCheck = 13;
  gpioSetMode(BCheck, OUTPUT);
  //  constexpr int MAX = 250;

  //---------Time----------
  struct timespec now, prev;
  long double delta;

  //----------Guess Point----------
  // bia UltraSonic
  // Origin Point = Center of Robot Square
  constexpr int measureX0 = 300, measureY0 = -300;
  int nowX = 0, nowY = 0;
  constexpr int firstX = 654, firstrY = 1454;
  constexpr double firstDeg = -5.5;

  //----------Movement----------
  // froont = 12
  // OutPut
  int sixWheel, twoWheel, tenWheel;
  double slowWheel;
  constexpr int MaxWheel = 255;
  // Input bia Robot
  int vXR, vYR, moment;
  // Input bia Field
  double velocityF;
  constexpr double angleF = M_PI / 6 - firstDeg / 180 * M_PI;
  // Lock Angle in PID
  // Result: yaw, Goal: yawLock, Control: moment
  double yaw = 0, yawDiff, yawPrev = firstDeg;
  constexpr double YawLock = firstDeg;
  constexpr double yawProp = 7.5, yawInt = 0, yawDeff = 0;

  //----------Calibration----------
  UPDATELOOP(Controller,
             !(Controller.button(RIGHT) && Controller.button(SQUARE))) {}
  GY521 gyro;
  gyro.start();
  gyro.resetYaw(YawLock);

  cout << "Main Start" << endl;
  gpioWrite(BCheck, 1);

  // MainLoop
  UPDATELOOP(Controller,
             !(Controller.button(START) && Controller.button(CROSS))) {
    //----------Sensar----------
    // GY521
    yaw = gyro.getYaw();
    // time
    now = prev;
    clock_gettime(CLOCK_REALTIME, &now);
    delta = now.tv_sec - prev.tv_sec +
            (long double)(now.tv_nsec - prev.tv_nsec) / 1000000000;

    //----------Guess Point----------
    nowX = ms.send(2, 20, 0) + measureX0;
    nowY = ms.send(3, 20, 0) + measureY0;

    //----------Movement----------
    // Input bia Field
    if (nowX < 2000) {
      velocityF = clamp((double)nowX - firstX, MaxWheel / 2 * ROOT3, 0.0);
    } else if (nowY > 0) {
      velocityF = 0;
    } else {
    }

    // Change Robot frome Field
    vXR = velocityF * cos(angleF - yaw * M_PI / 180);
    vYR = velocityF * sin(angleF - yaw * M_PI / 180);
    yawDiff = yaw - yawPrev;
    moment = yawProp * yawDiff;
    yawPrev = yaw;

    // Nomal
    twoWheel = -vXR / 2 + ROOT3 * vYR / 2 + moment;
    sixWheel = vXR + moment;
    tenWheel = -vXR / 2 - ROOT3 / 2 * vYR + moment;

    // Regulation Max
    slowWheel = 1.0;
    if (twoWheel / MaxWheel > slowWheel) {
      slowWheel = MaxWheel / twoWheel;
    }
    if (sixWheel / MaxWheel > slowWheel) {
      slowWheel = MaxWheel / sixWheel;
    }
    if (tenWheel / MaxWheel > slowWheel) {
      slowWheel = MaxWheel / tenWheel;
    }

    //----------Finish----------
    // Move Boost
    if (Controller.button(L1)) {
      slowWheel *= 0.2;
    } else if (!Controller.button(R1)) {
      slowWheel *= 0.5;
    }

    // Output
    cout << "two" << twoWheel * slowWheel;
    cout << "six" << sixWheel * slowWheel;
    cout << "ten" << tenWheel * slowWheel;
    cout << endl;

    ms.send(1, 2, 250 * twoWheel * slowWheel);
    ms.send(2, 2, 250 * sixWheel * slowWheel);
    ms.send(3, 2, 250 * tenWheel * slowWheel);

    //----------Emergency----------
    if (Controller.press(SELECT)) {
      UPDATELOOP(Controller, !Controller.press(SELECT)) {
        ms.send(255, 255, 0);
        if (Controller.button(START) && Controller.button(CROSS)) {
          // FinishSequence
          ms.send(255, 255, 0);
          gpioWrite(BCheck, 0);
          return -1;
        }
      }
    }
  }
  cout << "Main Finish" << endl;
  ms.send(255, 255, 0);
  gpioWrite(BCheck, 0);
  return 0;
}
