//立ルンです, MainLoop from 112
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
inline double check(double a, double b, double c);

int main(void) {
  DualShock3 Controller;
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

  //---------Time----------
  struct timespec now, prev;
  long double delta, start = 0;

  //----------Guess Point----------
  // Origin Point = Centerof Robot Square
  constexpr int firstX = 710, firstY = 1470 + 89;
  constexpr double firstDeg = 0;
  double nowPoint[3] = {firstX, firstY, firstDeg};
  double deltaX, deltaY, deltaL, deltaA;
  constexpr double MatrixPoint[3][3] = {{-2.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0},
                                        {0, 1.0 / ROOT3, -1.0 / ROOT3},
                                        {-1.0 / 3.0, -1.0 / 3.0, -1.0 / 3.0}};

  // bia UltraSonic
  // constexpr int measureX0 = 400, measureY0 = 300;
  // constexpr double UltraReg = 1.05;

  //----------Plan Root----------
  // Goal
  constexpr int TwoTableX = 3000, TwoTableY = 3500, TwoTableR = 1280,
                TwoTableAngle = 30;
  int twoTableDeg = 0;
  int goalX = firstX, goalY = firstY;
  bool flagNear = false;

  // bia Field View, also yawGoal = moment
  double velocityF, angleF;

  //----------Movement----------
  // OutPut
  constexpr int WheelID[3] = {1, 2, 3};
  constexpr int SpeedMax = 240;
  constexpr int SpeedMin = 7;
  constexpr double WheelDeg[3] = {0, M_PI_3 * 2, -M_PI_3 * 2};
  double wheelSlow;
  double wheelOut[3] = {};

  // Input Robot View
  double velocityR = 0, angleR, moment;
  constexpr int ErrorMin = 15, ErrorSpead = 20;
  constexpr double ErrorReg =
      (SpeedMax - ErrorSpead) / log(SpeedMax - ErrorMin + 1);

  // Option
  // WheelSpeed Control from  Accel
  // Result: wheelOut[PWM], Goal: wheelGoal[PWM], Control: wheelOut[PWM]
  constexpr double AccelMax = 50, Jerk = 200;
  int wheelGoal[3];
  double wheelAccel[3] = {};
  long double accelTime[3][3] = {{}, {}, {}}, accelStart = 0;
  int accelPolar[3] = {};

  // Lock Angle bia PID
  // Result: yaw[degree], Goal: yawLock[degree], Control: moment(define
  // before)[PWM]
  double yaw, yawDelta, yawPrev;
  double yawGoal = firstDeg;
  constexpr double yawProp = 10, yawInt = 0, yawDeff = 0;
  // constexpr double yawProp = 22.8, yawInt = 62.4, yawDeff = 2.0;

  //----------Calibration----------
  UPDATELOOP(Controller,
             !(Controller.button(RIGHT) && Controller.button(SQUARE))) {}
  //----------Gyro----------
  GY521 gyro(0x68, 2, 1000, 1.0);
  gyro.start(firstDeg);

  //----------IncRotary----------
  constexpr int Range = 500 * 2;
  constexpr double WheelCirc = 101.6 * M_PI;
  rotaryInc rotary[3] = {rotaryInc(27, 17, true), rotaryInc(11, 9, true),
                         rotaryInc(10, 22, true)};
  int wheelIn[3] = {};
  int wheelInPrev[3] = {};
  double wheelSpeed[3] = {};

  cout << "Main Start" << endl;
  gpioWrite(BCheck, 1);
  clock_gettime(CLOCK_REALTIME, &now);

  // MainLoop
  UPDATELOOP(Controller,
             !(Controller.button(START) && Controller.button(CROSS))) {
    //----------Sensor----------
    // time
    prev = now;
    clock_gettime(CLOCK_REALTIME, &now);
    delta = now.tv_sec - prev.tv_sec +
            (long double)(now.tv_nsec - prev.tv_nsec) / 1000000000;
    start += delta;

    // GY521
    gyro.updata();
    yaw = gyro.yaw;

    // RotaryInc
    for (int i = 0; i < 3; ++i) {
      wheelInPrev[i] = wheelIn[i];
      wheelIn[i] = rotary[i].get();
      wheelSpeed[i] =
          (double)(wheelIn[i] - wheelInPrev[i]) * WheelCirc / (double)Range;
      // cout << wheelIn[i] << ",";
    }
    // cout << endl;

    //----------Reset----------
    if (Controller.button(RIGHT) && Controller.button(SQUARE)) {
      gyro.resetYaw(firstDeg);
      nowPoint[0] = goalX = firstX;
      nowPoint[1] = goalY = firstY;
      twoTableDeg = 0;
    }

    //-----------Guess Field----------
    deltaX = deltaY = 0;
    for (int i = 0; i < 3; ++i) {
      deltaX += MatrixPoint[0][i] * wheelSpeed[i];
      deltaY += MatrixPoint[1][i] * wheelSpeed[i];
    }
    deltaL = hypot(deltaX, deltaY);
    deltaA = atan2(deltaY, deltaX);
    nowPoint[0] -= deltaL * cos(deltaA + yaw * M_PI / 180);
    nowPoint[1] -= deltaL * sin(deltaA + yaw * M_PI / 180);

    //----------Plan Root----------
    if (Controller.press(CIRCLE)) {
      yawGoal = twoTableDeg;
      goalX = TwoTableX + TwoTableR * sin(yawGoal * M_PI / 180);
      goalY = TwoTableY - TwoTableR * cos(yawGoal * M_PI / 180);
      twoTableDeg = (twoTableDeg + TwoTableAngle) % 360;
    } else if (Controller.press(UP)) {
      goalX = firstX;
      goalY = firstY;
      yawGoal = firstDeg;
    }

    velocityF = hypot(goalY - nowPoint[1], goalX - nowPoint[0]);
    angleF = atan2(goalY - nowPoint[1], goalX - nowPoint[0]);

    //----------Movement----------
    // Input
    /*---------- Manual
    int stickX = Controller.stick(LEFT_X);
    int stickY = -Controller.stick(LEFT_Y);
    angleF = atan2(stickY, stickX);
    velocityF = hypot(stickX, stickY) * (fabs(0.58 * cos(2 * angleF)) + 1.4);
    velocityR = velocityF;
    ----------*/
    if (velocityF < ErrorMin && velocityF > -ErrorMin) {
      velocityR = 0;
      flagNear = true;
    } else if (velocityF < SpeedMax) {
      velocityR = SpeedMax - ErrorReg * log(SpeedMax - velocityF + 1);
      flagNear = true;
    } else {
      velocityR = SpeedMax;
      flagNear = false;
    }

    angleR = angleF - yaw * M_PI / 180;
    /*----------Manual
    moment = -(Controller.stick(LEFT_T) - Controller.stick(RIGHT_T)) * 0.5;
    if(moment == 0){
      yawPrev = yawDelta;
      yawDelta = yawGoal - yaw;
      if (yawDelta > 180) {
        yawDelta -= 360;
      } else if (yawDelta <= -180) {
        yawDelta += 360;
      }
      moment = yawProp * yawDelta + yawInt * yawDelta * delta +
              yawDeff * (yawDelta - yawPrev) / delta;
      moment *= -1;
    }
    ----------*/
    // moment from Lock Angle bia PID
    // if (!flagGoal) {
    yawPrev = yawDelta;
    yawDelta = yawGoal - yaw;
    if (yawDelta > 180) {
      yawDelta -= 360;
    } else if (yawDelta <= -180) {
      yawDelta += 360;
    }
    moment = yawProp * yawDelta + yawInt * yawDelta * delta +
             yawDeff * (yawDelta - yawPrev) / delta;
    moment *= -1;

    if (moment > 50) {
      moment = 50;
    } else if (moment < -50) {
      moment = -50;
    }

    // WheelOut
    int dummyMax = SpeedMax;
    for (int i = 0; i < 3; ++i) {
      wheelGoal[i] = velocityR * wheel_Func(angleR + WheelDeg[i]) + moment;
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

    // WheelSpeed Control from  Accel
    if (flagNear) {
      for (int i = 0; i < 3; ++i) {
        wheelOut[i] = wheelGoal[i];
      }
    } else {
      accelStart = start;
      for (int i = 0; i < 3; ++i) {
        accelTime[i][0] = (AccelMax - wheelAccel[i]) / Jerk;
        accelTime[i][1] = (fabs((double)wheelGoal[i] - wheelOut[i]) -
                           (AccelMax + wheelAccel[i]) / 2 * accelTime[i][0] -
                           AccelMax * AccelMax / 2 / Jerk) /
                          AccelMax;
        accelTime[i][2] = AccelMax / Jerk;
        if (accelTime[i][1] < 0) {
          accelTime[i][1] *= -1;
          accelTime[i][0] +=
              check((AccelMax - wheelAccel[i]) / (2 * accelTime[i][0]),
                    AccelMax, -AccelMax * accelTime[i][1] / 2);
          accelTime[i][2] += check(AccelMax / (2 * accelTime[i][2]), AccelMax,
                                   -AccelMax * accelTime[i][1] / 2);
          accelTime[i][2] += accelTime[i][0];
          accelTime[i][1] = accelTime[i][0];
        } else {
          accelTime[i][1] += accelTime[i][0];
          accelTime[i][2] += accelTime[i][1];
        }
        if (wheelGoal[i] > wheelOut[i]) {
          accelPolar[i] = 1;
        } else {
          accelPolar[i] = -1;
        }
      }

      for (int i = 0; i < 3; ++i) {
        if (start - accelStart < accelTime[i][0]) {
          wheelAccel[i] += Jerk * delta;
          wheelOut[i] += accelPolar[i] * wheelAccel[i] * delta;
        } else if (start - accelStart < accelTime[i][1]) {
          wheelOut[i] += accelPolar[i] * wheelAccel[i] * delta;
        } else if (start - accelStart <= accelTime[i][2]) {
          wheelAccel[i] -= Jerk * delta;
          wheelOut[i] += accelPolar[i] * wheelAccel[i] * delta;
        }
      }
    }

    // Output
    // Data
    cout << start << ", ";
    for (int i = 0; i < 3; ++i) {
      cout << (int)wheelOut[i] << ", ";
    }
    cout << nowPoint[0] << ", " << nowPoint[1] << ", " << yaw << ", ";
    cout << velocityF << ", " << velocityR << ", " << angleR;
    cout << endl;
    /*
    cout << nowPoint[0] << ", " << nowPoint[1] << ", " << yaw << endl;
    for (int i = 0; i < 3; ++i) {
      cout << wheelIn[i] << ",";
    }
    */

    for (int i = 0; i < 3; ++i) {
      ms.send(WheelID[i], 2, wheelOut[i]);
    }

    //----------Emergency----------
    if (Controller.press(SELECT)) {
      UPDATELOOP(Controller, !Controller.press(SELECT)) {
        ms.send(255, 255, 0);
        ms.send(1, 2, 0);
        if (Controller.button(START) && Controller.button(CROSS)) {
          // FinishSequence
          ms.send(255, 255, 0);
          ms.send(1, 2, 0);
          gpioWrite(BCheck, 0);
          cout << "Main Finish" << endl;
          return -1;
        }
      }
    }
  }
  cout << "Main Finish" << endl;
  ms.send(255, 255, 0);
  ms.send(1, 2, 0);
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

inline double check(double a, double b, double c) {
  return (-b + sqrt(pow(b, 2) + 4 * a * c)) / (2 * a);
}
