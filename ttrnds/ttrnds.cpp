// 立ルンですのプログラム, MainLoop from 77
#include "/home/pi/PigpioMS/PigpioMS.hpp"
#include "/home/pi/RasPiDS3/RasPiDS3.hpp"
#include "/home/pi/Sensor/GY521/GY521.hpp"
#include "/home/pi/Sensor/RotaryInc/RotaryInc.hpp"
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

inline double wheel_Func(double rad);

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
  constexpr int Max = 250;

  //---------Time----------
  struct timespec now, prev;
  long double delta;

  //----------IncRotary----------
  constexpr int Range = 500 * 2;
  rotaryInc rotary[3] = {rotaryInc(17, 27, true), rotaryInc(22, 10, true),
                         rotaryInc(9, 11, true)};
  int wheelIn[3];
  double constexpr WheelCirc = 101.6 * M_PI;

  //----------Guess Point----------
  constexpr int firstX = 654, firstrY = 1454;
  constexpr double firstDeg = -5;
  /*
  // bia UltraSonic
  // Origin Point = Center of Robot Square
  constexpr int measureX0 = 400, measureY0 = 300;
  int nowX = 0, nowY = 0;
  constexpr double UltraReg = 1.05;
  */

  //----------Movement----------
  // OutPut
  int wheelOut[3];
  constexpr double wheelDeg[3] = {M_PI_3, 0, -M_PI_3};
  double wheelSlow;
  // Wheel Speed bia PID with Control Accel
  // Result: Speed, Goal: Goal, Control; Out(define before)
  int wheelSpeed[3], wheelGoal[3], wheelDiff[3], wheelPrev[3];
  constexpr double wheelProp = 1, wheelInt = 0, wheelDeff = 0;
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
    //----------Sensor----------
    // time
    prev = now;
    clock_gettime(CLOCK_REALTIME, &now);
    delta = now.tv_sec - prev.tv_sec +
            (long double)(now.tv_nsec - prev.tv_nsec) / 1000000000;

    // GY521
    if (Controller.button(RIGHT) && Controller.button(SQUARE)) {
      gyro.resetYaw(firstDeg);
    }
    yaw = gyro.getYaw();

    // RotaryInc
    for (int i = 0; i < 3; ++i) {
      wheelIn[i] = rotary[i].get();
    }

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
    out[wheel(two)] = velocityF * cos(angleR - M_PI_3) + moment;
    out[wheel(six)] = velocityF * cos(angleR) + moment;
    out[wheel(ten)] = velocityF * cos(angleR + M_PI_3) + moment;
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

    for (int i = 0; i < 3; ++i) {
      wheelSpeed[i] = wheelIn[i] / Range * WheelCirc / delta;
      wheelOut[i] = velocityF * wheel_Func(angleR + wheelDeg[i]) + moment;
    }

    // Regulation Max
    wheelSlow = 1.0;
    for (int i = 0; i < 3; ++i) {
      if (wheelOut[i] > Max) {
        wheelSlow = Max / wheelOut[i];
      }
    }

    // Move Boost
    if (Controller.button(L1)) {
      wheelSlow *= 0.2;
    } else if (!Controller.button(R1)) {
      wheelSlow *= 0.5;
    }

    // Output
    /*
    for (int i = 0; i < 3; ++i) {
      cout << 4 * i + 2 << ":" << (int)(wheelOut[i] * wheelSlow);
    }
    cout << endl;
    */
    for (int i = 0; i < 3; ++i) {
      ms.send(i + 1, 2, wheelOut[i] * wheelSlow);
    }

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

inline double wheel_Func(double rad) {
  while (rad < 0) {
    rad += 2 * M_PI;
  }
  while (rad >= 2 * M_PI) {
    rad -= 2 * M_PI;
  }

  if (0 <= rad && rad < M_PI_6) {
    return 1;
  } else if (M_PI_6 <= rad && rad < 5 * M_PI_6) {
    return (M_PI_2 - rad) * 3 / M_PI;
  } else if (5 * M_PI_6 <= rad && rad < 7 * M_PI_6) {
    return -1;
  } else if (7 * M_PI_6 <= rad && rad < 11 * M_PI_6) {
    return (rad - 3 * M_PI_2) * 3 / M_PI;
  } else if (11 * M_PI_6 <= rad && rad < 12 * M_PI_6) {
    return 1;
  }
  return 0;
}
