//立ルンですのプログラム, MainLoop from 103
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
  DualShock3 Controller;
  gpioInitialise();
  MotorSerial ms;
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
  constexpr int Max = 240;

  //---------Time----------
  struct timespec now, prev, start;
  long double delta, time;

  //----------IncRotary----------
  constexpr int Range = 500 * 2;
  rotaryInc rotary[3] = {rotaryInc(17, 27, true), rotaryInc(22, 10, true),
                         rotaryInc(9, 11, true)};
  int wheelIn[3] = {0, 0, 0};
  int wheelInPrev[3] = {0, 0, 0};
  double constexpr WheelCirc = 101.6 * M_PI;

  //----------Guess Point----------
  constexpr int firstX = 710, firstY = 1630;
  constexpr double firstDeg = 0;
  //Goal
  constexpr int goalX = 1210, goalY = 3630;
  // bia UltraSonic
  // Origin Point = Centerof Robot Square
  constexpr int measureX0 = 400, measureY0 = 300;
  double nowPoint[3] = {0, 0, 0};
  constexpr double UltraReg = 1.05;
  constexpr double MatrixPoint[3][3] = {{-1.0 / 3.0, 2.0 / 3.0, -1.0 / 3.0}, {1.0 / ROOT3, 0, -1.0 / ROOT3}, {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}};

  //----------Movement----------
  // OutPut
  constexpr int SpeedMax = 245;     // Voltage = 16 [V], PWM = 230
  constexpr int SpeedMin = 25;       // Voltage = 16 [V], PWM = 230
  constexpr double SpeedRate = 1.0; // Voltage = 16 [V], PWM = 230
  // constexpr int SpeedMax = 1700; // Voltage = 8 [V], PWM = 230
  // constexpr int SpeedMin = 1700; // Voltage = 8 [V], PWM = 230
  // constexpr double SpeedRate = 1700; // Voltage = 8 [V], PWM = 230
  int wheelOut[3];
  constexpr double wheelDeg[3] = {M_PI_3, -M_PI, -M_PI_3};
  double wheelSlow;
  int wheelSpeed[3], wheelGoal[3], wheelDelta[3], wheelPrev[3];
  // WheelSpeed bia PID with Control Accel
  // Result:Speed[mm / s], Goal : Goal[mm / s] Control; Out[PWM](define before)
  constexpr double wheelProp = 1, wheelInt = 0, wheelDeff = 0;
  // Input Robot View, velocityR = velocityF
  double angleR, moment;
  // Input Field View
  double velocityF, angleF = M_PI / 6 - firstDeg / 180 * M_PI;
  // Lock Angle bia PID
  // Result: yaw, Goal: yawLock, Control: moment(define before), [mm/s]
  double yaw, yawDelta, yawPrev;
  constexpr double YawGoal = firstDeg;
  constexpr double yawProp = 10, yawInt = 0, yawDeff = 0;
  //constexpr double yawProp = 22.8, yawInt = 62.4, yawDeff = 2.0;
  //----------Calibration----------
  UPDATELOOP(Controller,
             !(Controller.button(RIGHT) && Controller.button(SQUARE))) {}
  GY521 gyro;
  gyro.start();
  gyro.resetYaw(firstDeg);

  cout << "Main Start" << endl;
  gpioWrite(BCheck, 1);
  clock_gettime(CLOCK_REALTIME, &prev);
  start = prev;

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
      wheelInPrev[i] = wheelIn[i];
      wheelIn[i] = rotary[i].get();
      wheelSpeed[i] =
          (double)(wheelIn[i] - wheelInPrev[i]) / Range * WheelCirc / delta;
    }

    //----------Movement----------
    //Guess Field
    for(int i = 0; i < 3; ++i){
      for(int j = 0; j < 3; ++j){
        nowPoint[i] += MatrixPoint[i][j] * (double)(wheelIn[j] - wheelInPrev[j]) * WheelCirc / (double)Range;
      }
      cout << nowPoint[i] << ", ";
    }
    cout << endl;

    // Input Field View
    velocityF = hypot(goalY - nowPoint[1] - firstY, goalX - nowPoint[0] - firstX) * 10;
    angleF = atan2(goalY - nowPoint[1] - firstY, goalX - nowPoint[0] - firstX);
    cout << velocityF << ", " << angleF << endl;
    if(velocityF > 240){
      velocityF = 240;
    }

    // Change Field to Robot
    yawDelta = yaw - yawPrev;
    moment = yawProp * yawDelta;
    yawPrev = yaw;

    // Input
    int stickX = Controller.stick(LEFT_X);
    int stickY = -Controller.stick(LEFT_Y);
    // angleF = atan2(stickY, stickX);
    // velocityF = hypot(stickX, stickY) * (fabs(0.58 * cos(2 * angleF)) + 1.4);
    if (velocityF > 250) {
      velocityF = 250;
    }
    angleR = angleF - yaw * M_PI / 180;

    // moment = -(Controller.stick(LEFT_T) - Controller.stick(RIGHT_T)) * 0.5;
    // moment frome Lock Angle bia PID
    // if(moment == 0){
      yawPrev = yawDelta;
      yawDelta = YawGoal - yaw;
      if (yawDelta > 180) {
        yawDelta -= 360;
      } else if (yawDelta <= -180) {
        yawDelta += 360;
      }
      moment = yawProp * yawDelta + yawInt * yawDelta * delta +
              yawDeff * (yawDelta - yawPrev) / delta;
      moment *= -1;
    // }
    // moment from stick
    if (moment > 125) {
      moment = 125;
    } else if (moment < -125) {
      moment = -125;
    }

    // wheelGoal
    int dummyMax = SpeedMax;
    for (int i = 0; i < 3; ++i) {
      wheelGoal[i] = velocityF * wheel_Func(angleR + wheelDeg[i]) + moment;
      if (abs(wheelGoal[i]) > dummyMax) {
        dummyMax = abs(wheelGoal[i]);
      }
    }
    wheelSlow = SpeedMax / (double)dummyMax;
    if (Controller.button(L1)) {
      wheelSlow *= 0.2;
    } else if (!Controller.button(R1)) {
      wheelSlow *= 0.5;
    }

    for (int i = 0; i < 3; ++i) {
      wheelGoal[i] *= wheelSlow;
      if (0 < wheelGoal[i] && wheelGoal[i] < SpeedMin) {
        wheelGoal[i] = 0;
      } else if (0 > wheelGoal[i] && wheelGoal[i] > -SpeedMin) {
        wheelGoal[i] = 0;
      }
    }

    // 2018/7/19 without PID
    // wheelOut & PID
    /*
    for (int i = 0; i < 3; ++i) {
      wheelGoal[i] *= wheelSlow;
      wheelPrev[i] = wheelDelta[i];
      wheelSpeed[i] = (wheelIn[i] - wheelInPrev[i]) / Range * WheelCirc / delta;
      wheelDelta[i] = wheelGoal[i] - wheelSpeed[i];
      wheelOut[i] = wheelProp * wheelDelta[i] +
                    wheelInt * wheelDelta[i] * delta +
                    wheelDeff * (wheelDelta[i] - wheelPrev[i]) / delta;
    }
    */
    // wheelGoal change wheelOut
    for (int i = 0; i < 3; ++i) {
      wheelOut[i] = wheelGoal[i] * SpeedRate;
    }

    /*
    // Output
    for (int i = 0; i < 3; ++i) {
      cout << 4 * i + 2 << ":" << (int)wheelOut[i] << endl;
    }
    */

    for (int i = 0; i < 3; ++i) {
      ms.send(i + 1, 2, wheelOut[i]);
    }

    //----------Emergency----------
    if (Controller.press(SELECT)) {
      UPDATELOOP(Controller, !Controller.press(SELECT)) {
        ms.send(255, 255, 0);
        if (Controller.button(START) && Controller.button(CROSS)) {
          // FinishSequence
          ms.send(255, 255, 0);
          gpioWrite(BCheck, 0);
          cout << "Main Finish" << endl;
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
