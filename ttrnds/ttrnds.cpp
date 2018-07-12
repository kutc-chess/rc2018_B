//立てルンですのプログラム, MainLoop from 116
#include "/home/pi/PigpioMS/PigpioMS.hpp"
#include "/home/pi/RasPiDS3/RasPiDS3.hpp"
#include "/home/pi/Sensor/GY521/GY521.hpp"
#include <atomic>
#include <cstdio>
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

  //----------Movement----------
  // froont = 12
  // OutPut
  double sixWheel, twoWheel, tenWheel;
  constexpr int MaxWheel = 255;
  // Input bia field
  double velocityF, radF, slowWheel;
  // Input bia Robot
  int vXR, vYR, moment;
  double radR;
  // Lock Angle in PID
  // Result: yaw, Goal: yawLock, Control: moment
  double yaw = 0;
  /*
  bool flagLock = false;
  double yawLock, yawDev;
  constexpr double yawProp = 7.5;
  */

  //----------Guess Point----------
  // bia UltraSonic
  // Origin Point = Center of Robot Square
  constexpr int measureX0[2] = {100, 100};
  constexpr int measureY0[2] = {100, 100};
  int distX = 0, distY = 0;
  int nowX = 110, nowY = 110;

  //----------Calibration----------
  //静止状態を作る SQUAREとRIGHTボタンが押されるまで待機
  UPDATELOOP(Controller,
             !(Controller.button(RIGHT) && Controller.button(SQUARE))) {}
  // GY521 gyro;
  // gyro.start();

  cout << "Main Start" << endl;
  gpioWrite(BCheck, 1);

  UPDATELOOP(Controller,
             !(Controller.button(START) && Controller.button(CROSS))) {
    //----------Sensar----------
    // GY521
    if (Controller.button(RIGHT) && Controller.button(SQUARE)) {
      // gyro.resetYaw(0);
    }
    // yaw = gyro.getYaw();
    // time
    now = prev;
    clock_gettime(CLOCK_REALTIME, &now);
    delta = now.tv_sec - prev.tv_sec +
            (long double)(now.tv_nsec - prev.tv_nsec) / 1000000000;

    //----------Movement----------
    moment = -(Controller.stick(LEFT_T) - Controller.stick(RIGHT_T));
    int stickX = Controller.stick(RIGHT_X);
    int stickY = -Controller.stick(RIGHT_Y);
    // Transport Polar Coordinate
    radF = atan2(stickY, stickX);
    if (radF >= 2 * M_PI) {
      radF -= 2 * M_PI;
    } else if (radF < 0) {
      radF += 2 * M_PI;
    }
    velocityF = hypot(stickX, stickY) * (fabs(0.58 * cos(2 * radF)) + 1.4);
    radR = radF - (yaw / 180 * M_PI);
    vXR = velocityF * cos(radR);
    vYR = velocityF * sin(radR);

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
    if (Controller.button(DOWN)) {
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
  ms.send(3, 100, 0);
  gpioWrite(BCheck, 0);
  return 0;
}
