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
  struct timespec now, prev;
  long double delta, time = 0;

  //----------IncRotary----------
  constexpr int Range = 500 * 2;
  rotaryInc rotary[3] = {rotaryInc(17, 27, true), rotaryInc(22, 10, true),
                         rotaryInc(9, 11, true)};
  int wheelIn[3] = {0, 0, 0};
  int wheelInPrev[3] = {0, 0, 0};

  //----------Guess Point----------
  // Origin Point = Centerof Robot Square
  constexpr int firstX = 740, firstY = 1520 + 89;
  constexpr double firstDeg = 0;
  double nowPoint[3] = {firstX, firstY, 0};
  double diffXY[2] = {}, diffV, diffR;
  constexpr double MatrixPoint[3][3] = {{-1.0 / 3.0, 2.0 / 3.0, -1.0 / 3.0}, {1.0 / ROOT3, 0, -1.0 / ROOT3}, {1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0}};
  double constexpr WheelCirc = 101.6 * M_PI;
  
  // bia UltraSonic
  // constexpr int measureX0 = 400, measureY0 = 300;
  // constexpr double UltraReg = 1.05;
 
  //----------Plan Root----------
  //Goal
  constexpr int TwoTableX = 3000, TwoTableY = 3500, TwoTableR = 1000, TwoTableAngle = 20;
  int twoTableDeg = 0;
  int goalX = TwoTableX + TwoTableR * sin(twoTableDeg), goalY = TwoTableY - TwoTableR * cos(twoTableDeg);
  // bia Field View, also yawGoal
  double velocityF, angleF;

  //----------Movement----------
  // OutPut
  constexpr int SpeedMax = 240;
  constexpr int SpeedMin = 25; 
  int wheelOut[3];
  constexpr double wheelDeg[3] = {M_PI_3, -M_PI, -M_PI_3};
  double wheelSlow;
  int wheelSpeed[3], wheelGoal[3], wheelDelta[3], wheelPrev[3];
  // WheelSpeed Control from  Accel
  // Result:Speed[mm / s], Goal : Goal[mm / s] Control; Out[PWM](define before)
  // constexpr double wheelProp = 1, wheelInt = 0, wheelDeff = 0;

  // Input Robot View
  double velocityR, angleR, moment;

  // Lock Angle bia PID
  // Result: yaw, Goal: yawLock, Control: moment(define before), [mm/s]
  double yaw, yawDelta, yawPrev;
  double yawGoal = firstDeg;
  constexpr double yawProp = 10, yawInt = 0, yawDeff = 0;
  //constexpr double yawProp = 22.8, yawInt = 62.4, yawDeff = 2.0;

  //----------Calibration----------
  UPDATELOOP(Controller,
             !(Controller.button(RIGHT) && Controller.button(SQUARE))) {}
  GY521 gyro(0x68, 2, 1000, 1.0);
  gyro.start(firstDeg);

  cout << "Main Start" << endl;
  gpioWrite(BCheck, 1);
  clock_gettime(CLOCK_REALTIME, &prev);

  // MainLoop
  UPDATELOOP(Controller,
             !(Controller.button(START) && Controller.button(CROSS))) {
    //----------Sensor----------
    // Reset
    if (Controller.button(RIGHT) && Controller.button(SQUARE)) {
      gyro.resetYaw(firstDeg);
      nowPoint[0] = firstX;
      nowPoint[1] = firstY;
      twoTableDeg = 0;
    }
    // time
    prev = now;
    clock_gettime(CLOCK_REALTIME, &now);
    delta = now.tv_sec - prev.tv_sec +
            (long double)(now.tv_nsec - prev.tv_nsec) / 1000000000;
    time += delta;

    // GY521
    gyro.updata();
    yaw = gyro.yaw;
    
    // RotaryInc
    for (int i = 0; i < 3; ++i) {
      wheelInPrev[i] = wheelIn[i];
      wheelIn[i] = rotary[i].get();
      cout << wheelIn[i] << ", ";
      wheelSpeed[i] =
          (double)(wheelIn[i] - wheelInPrev[i]) / Range * WheelCirc / delta;
    }
    cout << endl;

    //-----------Guess Field----------
    diffXY[0] = diffXY[1] = 0;
    for(int i = 0; i < 2; ++i){
      for(int j = 0; j < 3; ++j){
        diffXY[i] += MatrixPoint[i][j] * (double)(wheelIn[j] - wheelInPrev[j]) * WheelCirc / (double)Range;
      }
    }
    diffV = hypot(diffXY[0], diffXY[1]);
    diffR = atan2(diffXY[1], diffXY[0]);
    nowPoint[0] += diffV * cos(diffR + yaw * M_PI / 180);
    nowPoint[1] += diffV * sin(diffR + yaw * M_PI / 180);
    cout << nowPoint[0] << ", " << nowPoint[1]<< ", " << yaw << endl;

    //----------Plan Root----------
    if(Controller.press(CIRCLE)){
      twoTableDeg = (twoTableDeg + TwoTableAngle) % 360;
    }
    goalX = TwoTableX + TwoTableR * sin(yawGoal * M_PI / 180);
    goalY = TwoTableY - TwoTableR * cos(yawGoal * M_PI / 180);
    yawGoal = twoTableDeg;

    velocityF = hypot(goalY - nowPoint[1], goalX - nowPoint[0]);
    angleF = atan2(goalY - nowPoint[1], goalX - nowPoint[0]);

    //----------Movement----------
    // Input
    /*
    int stickX = Controller.stick(LEFT_X);
    int stickY = -Controller.stick(LEFT_Y);
    angleF = atan2(stickY, stickX);
    velocityF = hypot(stickX, stickY) * (fabs(0.58 * cos(2 * angleF)) + 1.4);
    */
    velocityR = velocityF * 5;
    if(velocityR > SpeedMax){
      velocityR = SpeedMax;
    }
    angleR = angleF - yaw * M_PI / 180;

    /*
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
    */
    // moment frome Lock Angle bia PID
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

    if (moment > 125) {
      moment = 125;
    } else if (moment < -125) {
      moment = -125;
    }

    // wheelGoal
    int dummyMax = SpeedMax;
    for (int i = 0; i < 3; ++i) {
      wheelGoal[i] = velocityR * wheel_Func(angleR + wheelDeg[i]) + moment;
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
      wheelOut[i] = wheelGoal[i];
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
