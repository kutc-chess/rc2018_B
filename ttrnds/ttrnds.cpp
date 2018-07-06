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
  gpioSetMode(BCheck, OUTPUT);
  constexpr int MAX = 250;

  //---------Time----------
  struct timespec now, prev;
  long double delta;

  //----------Movement----------
  // Nomal
  // 12時が正面
  double sixWheel, twoWheel, tenWheel;
  double rad, slowWheel;
  double yaw, yawOffset = 0;
  int stickX, stickY, slant, moment;
  // Lock Angle in PID
  // Result: yaw, Goal: yawLock, Control: moment
  /*
  bool flagLock = false;
  double yawLock, yawDev;
  constexpr double yawProp = 7.5;
  */

  //----------Calibration----------
  //静止状態を作る SQUAREとRIGHTボタンが押されるまで待機
  UPDATELOOP(Controller,
             !(Controller.button(RIGHT) && Controller.button(SQUARE))) {}
  GY521 gyro;
  gyro.start();

  cout << "Main Start" << endl;
  gpioWrite(BCheck, 1);

  UPDATELOOP(Controller,
             !(Controller.button(START) && Controller.button(CROSS))) {
    //----------Sensar----------
    // GY521
    if (Controller.button(RIGHT) && Controller.button(SQUARE)) {
      gyro.resetYaw(0);
    }
    yaw = gyro.getYaw();
    now = prev;
    clock_gettime(CLOCK_REALTIME, &now);
    delta = now.tv_sec - prev.tv_sec +
            (long double)(now.tv_nsec - prev.tv_nsec) / 1000000000;

    /*
    if (Controller.press(UP)) {
      flagLock = !flagLock;
    }
    if (flagLock) {
      yawDev = yaw - yawLock;
      if (yawDev > 180) {
        yawDev -= 360;
      }
      moment = yawProp * yawDev;
    } else {
      moment = -(Controller.stick(LEFT_T) - Controller.stick(RIGHT_T));
      yawLock = yaw;
    }
    */

    moment = -(Controller.stick(LEFT_T) - Controller.stick(RIGHT_T));

    // Input
    stickX = Controller.stick(RIGHT_X);
    stickY = -Controller.stick(RIGHT_Y);
    if (!stickX && !stickY) {
      stickX = Controller.stick(LEFT_X);
      stickY = -Controller.stick(LEFT_Y);
      // Transport Polar Coordinate
      // 8方位
      rad = atan2(stickY, stickX) - (yaw / 180 * M_PI);
      if (rad >= 2 * M_PI) {
        rad -= 2 * M_PI;
      } else if (rad < 0) {
        rad += 2 * M_PI;
      }

      if (0 <= rad && rad < M_PI_4 / 2) {
        rad = 0;
      } else if (M_PI_4 / 2 <= rad && rad < M_PI_4 / 2 * 3) {
        rad = M_PI_4;
      } else if (M_PI_4 / 2 * 3 <= rad && rad < M_PI_4 / 2 * 5) {
        rad = M_PI_2;
      } else if (M_PI_4 / 2 * 5 <= rad && rad < M_PI_4 / 2 * 7) {
        rad = M_PI_4 * 3;
      } else if (M_PI_4 / 2 * 7 <= rad && rad < M_PI_4 / 2 * 9) {
        rad = M_PI;
      } else if (M_PI_4 / 2 * 9 <= rad && rad < M_PI_4 / 2 * 11) {
        rad = M_PI_4 * 5;
      } else if (M_PI_4 / 2 * 11 <= rad && rad < M_PI_4 / 2 * 13) {
        rad = M_PI_2 * 3;
      } else if (M_PI_4 / 2 * 13 <= rad && rad < M_PI_4 / 2 * 15) {
        rad = M_PI_4 * 7;
      } else if (M_PI_4 / 2 * 15 <= rad && rad < M_PI * 2) {
        rad = 0;
      }
      rad += yawOffset / 180 * M_PI;
    } else {
      // Transport Polar Coordinate
      rad = atan2(stickY, stickX) - (yaw / 180 * M_PI) + yawOffset / 180 * M_PI;
      if (rad >= 2 * M_PI) {
        rad -= 2 * M_PI;
      } else if (rad < 0) {
        rad += 2 * M_PI;
      }
    }
    slant = hypot(stickX, stickY) * (fabs(0.58 * cos(2 * rad)) + 1.4);
    if (slant > 250) {
      slant = 250;
    }

    //----------Don't Change the order----------
    // Move Parallel
    twoWheel = sixWheel = tenWheel = (double)slant / 250;
    twoWheel *= -sin(rad - M_PI / 6);
    sixWheel *= -cos(rad);
    tenWheel *= sin(rad + M_PI / 6);

    // Move Rotation
    twoWheel += (double)moment / 255;
    sixWheel += (double)moment / 255;
    tenWheel += (double)moment / 255;

    // Regulation Max
    slowWheel = 1.0;
    if (twoWheel > slowWheel) {
      slowWheel = twoWheel;
    }
    if (sixWheel > slowWheel) {
      slowWheel = sixWheel;
    }
    if (tenWheel > slowWheel) {
      slowWheel = tenWheel;
    }
    slowWheel = 1.0 / slowWheel;

    //----------Finish----------
    // Move Boost
    if (Controller.button(DOWN)) {
      slowWheel *= 0.2;
    } else if (!Controller.button(R1)) {
      slowWheel *= 0.5;
    }

    // Output
    cout << "two" << ms.send(1, 2, 250 * twoWheel * slowWheel);
    cout << "six" << ms.send(2, 2, 250 * sixWheel * slowWheel);
    cout << "ten" << ms.send(3, 2, 250 * tenWheel * slowWheel);
    cout << endl;

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
