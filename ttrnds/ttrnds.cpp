//立ルンですのプログラム, MainLoop from 77
#include "/home/pi/PigpioMS/PigpioMS.hpp"
#include "/home/pi/RasPiDS3/RasPiDS3.hpp"
#include "/home/pi/Sensor/GY521/GY521.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#define ROOT2 (1.41421356)
#define ROOT3 (1.7320508)
#define M_PI_3 (M_PI / 3)
#define M_PI_6 (M_PI / 6)

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
  gpioSetMode(BCheck, PI_OUTPUT);
  //  constexpr int MAX = 250;

  //---------Time----------
  struct timespec now, prev;
  long double delta;

  //----------Guess Point----------
  // bia UltraSonic
  // Origin Point = Center of Robot Square
  constexpr int measureX0 = 400, measureY0 = 300;
  int nowX = 0, nowY = 0;
  constexpr int firstX = 654, firstrY = 1454;
  constexpr double firstDeg = -5;
  constexpr double UltraReg = 1.05;

  //----------Movement----------
  // OutPut
  int sixWheel, twoWheel, tenWheel;
  double slowWheel;
  constexpr int MaxWheel = 255;
  // Input Robot View
  double angleR, moment;
  // Input Field View
  double velocityF, angleF = M_PI / 6 - firstDeg / 180 * M_PI;
  // Lock Angle bia PID
  // Result: yaw, Goal: yawLock, Control: moment
  double yaw = 0, yawDiff, yawPrev = firstDeg;
  constexpr double YawLock = firstDeg;
  constexpr double yawProp = 7.5, yawInt = 0, yawDeff = 0;

  //----------Calibration----------
  UPDATELOOP(Controller,
             !(Controller.button(RIGHT) && Controller.button(SQUARE))) {}
  GY521 gyro;
  gyro.start();
  gyro.resetYaw(firstDeg);

  cout << "Main Start" << endl;
  gpioWrite(BCheck, 1);
  clock_gettime(CLOCK_REALTIME, &prev);

  // MainLoop
  UPDATELOOP(Controller,
             !(Controller.button(START) && Controller.button(CROSS))) {
    //----------Sensar----------
    // GY521
    yaw = gyro.getYaw();
    // time
    prev = now;
    clock_gettime(CLOCK_REALTIME, &now);
    delta = now.tv_sec - prev.tv_sec +
            (long double)(now.tv_nsec - prev.tv_nsec) / 1000000000;

    //----------Movement----------
    /*
    // Input Field View
    if (nowY > 345) {
      velocityF = 50 * 2 / ROOT3;
      angleF = M_PI / 6 - firstDeg / 180 * M_PI;
    } else if (!(nowX - 2560 < 5 && nowX - 2560 > -5) &&
               !(nowY - 10 < 5 && nowX - 10 > -5)) {
      double velocityFX = (2560 - nowX) * (50) / 360;
      double velocityFY = (nowY - 10) * 10 / 340;
      velocityF = hypot(velocityFX, velocityFY) / ROOT2 * 2 / ROOT3;
      angleF = atan2(velocityFY, velocityFX);
    } else {
      velocityF = 0;
    }

    // Change Field to Robot
    yawDiff = yaw - yawPrev;
    moment = yawProp * yawDiff;
    yawPrev = yaw;

    // Nomal
    angleR = angleF - yaw * M_PI / 180;
    twoWheel = velocityF * cos(angleR - M_PI_3) + moment;
    sixWheel = velocityF * cos(angleR) + moment;
    tenWheel = velocityF * cos(angleR + M_PI_3) + moment;
    */
    int stickX = Controller.stick(LEFT_X);
    int stickY = -Controller.stick(LEFT_Y);
    angleF = atan2(stickY, stickX);
    velocityF = hypot(stickX, stickY) * (fabs(0.58 * cos(2 * angleF)) + 1.4);
    if (velocityF > 250) {
      velocityF = 250;
    }
    angleR = angleF - yaw * M_PI / 180;
    moment = -(Controller.stick(LEFT_T) - Controller.stick(RIGHT_T));

    twoWheel = velocityF * -sin(angleR - M_PI_6) + moment;
    sixWheel = velocityF * -cos(angleR) + moment;
    tenWheel = velocityF * sin(angleR + M_PI_6) + moment;

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
    /*
   cout << "two" << twoWheel * slowWheel;
   cout << "six" << sixWheel * slowWheel;
   cout << "ten" << tenWheel * slowWheel;
   cout << endl;
   */
    ms.send(1, 2, twoWheel * slowWheel);
    ms.send(2, 2, sixWheel * slowWheel);
    ms.send(3, 2, -tenWheel * slowWheel);

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
