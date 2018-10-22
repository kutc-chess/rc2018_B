//立ルンです, Ma/nLoop from 112
#include "/home/pi/PigpioMS/PigpioMS.hpp"
#include "/home/pi/RasPiDS3/RasPiDS3.hpp"
#include "/home/pi/Sensor/GY521/GY521.hpp"
#include "/home/pi/Sensor/RotaryInc/RotaryInc.hpp"
#include <cmath>
#include <iostream>
#include <pigpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <vector>
#define ROOT3 (1.7320508)
#define M_PI_3 (M_PI / 3)
#define M_PI_6 (M_PI / 6)

using namespace std;
using namespace RPDS3;
using namespace RPMS;
using namespace RPGY521;

struct pointinfo {
  int x;
  int y;
  int yaw;
  bool shoot;
  int table;
  int ultra;
};

inline double wheel_Func(double rad);
inline double check(double a, double b, double c);
inline bool yaw_check(int goal, int now);
constexpr double ErrorYaw = 3;
inline void finish();

int main(void) {
  MotorSerial ms;
  // MDD通信セットアップ
  try {
    ms.init();
  } catch (const char *str) {
    return -1;
  }

  //----------etc.----------
  int restart = 0;
  // Check LED
  constexpr int BCheck = 13;
  gpioSetMode(BCheck, PI_OUTPUT);
  constexpr int Finish = 13;
  gpioSetMode(Finish, PI_INPUT);
  gpioSetPullUpDown(CheckRed, PI_PUD_DOWN);

  constexpr int CheckRed = 26, CheckBlue = 8;
  gpioSetMode(CheckRed, PI_INPUT);
  gpioSetPullUpDown(CheckRed, PI_PUD_DOWN);
  gpioSetMode(CheckBlue, PI_INPUT);
  gpioSetPullUpDown(CheckBlue, PI_PUD_DOWN);

  constexpr int ZoneRed = 20, ZoneBlue = 5;
  bool flagZone;
  gpioSetMode(ZoneRed, PI_OUTPUT);
  gpioWrite(ZoneRed, 0);
  gpioSetMode(ZoneBlue, PI_OUTPUT);
  gpioWrite(ZoneBlue, 0);

  constexpr int LEDCal = 7, LEDReset = 6;
  gpioSetMode(LEDCal, PI_OUTPUT);
  gpioWrite(LEDCal, 0);
  gpioSetMode(LEDReset, PI_OUTPUT);
  gpioWrite(LEDReset, 0);

  constexpr int CheckCal = 19, CheckReset = 16;
  gpioSetMode(CheckCal, PI_INPUT);
  gpioSetPullUpDown(CheckCal, PI_PUD_DOWN);
  gpioSetMode(CheckReset, PI_INPUT);
  gpioSetPullUpDown(CheckReset, PI_PUD_DOWN);

  int phase = 0;

  //---------Time----------
  struct timespec now, prev;
  long double delta, start = 0;

  //----------Shoot---------
  constexpr int ShootR = 311, ShootL = 307;
  constexpr int ShootRID = 5, ShootLID = 6;
  int shineR = 31, shineL = 29;
  bool flagShootR = false;
  ms.send(1, 31, ShootR);
  ms.send(1, 31, ShootL);
  ms.send(1, 32, 400);
  ms.send(1, 32, 400);

  // bia UltraSonic
  constexpr int MeasureCal = 7;
  constexpr int MeasureID[3] = {1, 2, 3};
  constexpr int MeasureL0 = 535, MeasureR0 = 535, MeasureY0 = 535;
  constexpr int UltraSpead = 100;
  constexpr int UltraSideR = 535 + 200 + 400;
  // constexpr double UltraReg = 1.05;
  vector<int> measure[3];
  double measureL = 0, measureR = 0, measureY = 0;

  //----------Plan Root----------
  vector<struct pointinfo> PointTable;
  int pointCount = 0;
  struct pointinfo goal;

  // TwoTable
  constexpr int TwoTableX = 3000, TwoTableY = 3500, TwoTableR = 1285,
                TwoTableDiv = 12;

  // MoveTable
  constexpr int MoveTableY[3] = {5500, 6500, 7500};
  int MoveTableX[3] = {5500, 5500, 5500};
  constexpr int MoveTableR[3] = {1200, 1200, 1200};

  // Home
  constexpr int HomeMin = 20, HomeSpead = 15, HomeSpeadMax = 120, HomeMax = 400;
  constexpr double HomeReg =
      (HomeSpeadMax - HomeSpead) / log(HomeMax - HomeMin + 1);
  bool flagHome = false;

  // bia Field View, also yawGoal = moment
  double velocityF, angleF;

  //----------Movement----------
  // OutPut
  constexpr int WheelID[3] = {1, 2, 3};
  constexpr int SpeedMax = 220;
  constexpr int SpeedMin = 10;
  constexpr int MomentMax = 25;
  constexpr double WheelDeg[3] = {0, M_PI_3 * 2, -M_PI_3 * 2};
  double wheelSlow;
  double wheelGoal[3] = {};
  constexpr int ErrorMin = 20, ErrorSpead = 15, ErrorMax = 700;
  constexpr double ErrorReg =
      (SpeedMax - ErrorSpead) / log(ErrorMax - ErrorMin + 1);

  // Input Robot View
  double velocityR = 0, angleR, moment;

  // Option
  // WheelSpeed Control from  Accel
  // Result: velocityOut[PWM], Goal: velocityGoal[PWM],
  // Control:velocityOut[PWM]
  constexpr double AccelMax = 150, Jerk = 205;
  double velocityGoal[2] = {}, velocityAccel[2] = {}, velocityOut[2] = {};
  long double accelTime[2][3] = {{}, {}}, accelStart = 0;
  int accelPolar[2] = {};
  bool flagNear = false;
  long double stop = 0;
  constexpr double StopTime = 0.5;

  // Lock Angle bia PID
  // Result: yaw[degree], Goal: yawLock[degree], Control: moment(define
  // before)[PWM]
  double yaw, yawDelta, yawPrev;
  double yawGoal;
  constexpr double yawProp = 7.2, yawInt = 51.42, yawDeff = 0.252;
  // constexpr double yawProp = 10, yawInt = 185.4, yawDeff = 0.672;

  //----------Calibration----------
  gpioWrite(LEDCal, 1);
  while (1) {
    if (gpioRead(CheckCal) && (gpioRead(CheckRed) || gpioRead(CheckBlue))) {
      break;
    }
  }
  gpioWrite(LEDCal, 0);
  sleep(1);
  //----------Gyro----------
  GY521 gyro(0x68, 2, 1000, 1.02);

  //----------IncRotary----------
  constexpr int Range = 256 * 2;
  constexpr double WheelCirc = 101.6 * M_PI;
  rotaryInc rotary[3] = {rotaryInc(27, 17, true), rotaryInc(11, 9, true),
                         rotaryInc(10, 22, true)};
  int wheelIn[3] = {};
  int wheelInPrev[3] = {};
  double wheelSpeed[3] = {};

  //----------Zone Check----------
  gpioWrite(ZoneRed, 1);
  gpioWrite(ZoneBlue, 1);
  while (1) {
    if (gpioRead(CheckRed)) {
      flagZone = 1;
      cout << "Red" << endl;
      --shineR;
      --shineL;
      break;
    } else if (gpioRead(CheckBlue)) {
      flagZone = -1;
      cout << "Blue" << endl;
      break;
    }
  }
  gpioWrite(ZoneRed, 0);
  gpioWrite(ZoneBlue, 0);

  //----------Guess Point----------
  // Origin Point = Centerof Robot Square
  constexpr int firstX = 490, firstY = 1530 + 89;
  constexpr double firstDeg = -90;
  double nowPoint[3] = {(double)(firstX * flagZone), firstY, firstDeg};
  double deltaX, deltaY, deltaL, deltaA;
  constexpr double MatrixPoint[3][3] = {{-2.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0},
                                        {0, 1.0 / ROOT3, -1.0 / ROOT3},
                                        {-1.0 / 3.0, -1.0 / 3.0, -1.0 / 3.0}};

  //----------Plan Root----------
  struct pointinfo dummyPoint;
  for (int i = 0; i < TwoTableDiv - 2; ++i) {
    dummyPoint.yaw = 270 + i * 360 / TwoTableDiv % 360;
    dummyPoint.x =
        TwoTableX + TwoTableR * sin(dummyPoint.yaw * M_PI / 180) * flagZone;
    dummyPoint.y = TwoTableY - TwoTableR * cos(dummyPoint.yaw * M_PI / 180);
    dummyPoint.table = 1;
    dummyPoint.ultra = 0;

    if ((i / 2) % 2 == 0) {
      dummyPoint.shoot = true;
    } else {
      dummyPoint.shoot = false;
    }

    if (i == 0) {
      dummyPoint.table = 0;
    }

    if (dummyPoint.yaw % 90 == 0) {
      struct pointinfo dummyUltra = dummyPoint;
      dummyUltra.x =
          (TwoTableX + UltraSideR * sin(dummyUltra.yaw * M_PI / 180)) *
          flagZone;
      dummyUltra.y = TwoTableY - UltraSideR * cos(dummyUltra.yaw * M_PI / 180);
      dummyUltra.ultra = 1;
      dummyUltra.shoot = false;
      PointTable.push_back(dummyUltra);

      dummyPoint.ultra = 2;
    }
    PointTable.push_back(dummyPoint);
  }

  dummyPoint = {1000 * flagZone, MoveTableY[0] - 1000, 180, false, 0, 0};
  PointTable.push_back(dummyPoint);
  /*
  dummyPoint = {(MoveTableX[0] - MoveTableR[0]) * flagZone,
                MoveTableY[0],
                -90 + 180 * (flagZone - 1) / -2,
                true,
                2,
                2};
  PointTable.push_back(dummyPoint);
  dummyPoint = {1000 * flagZone,
                MoveTableY[0] - 1000,
                -90 + 180 * (flagZone - 1) / -2,
                false,
                0,
                0};
  PointTable.push_back(dummyPoint);
  */

  dummyPoint = {firstX * flagZone, firstY - 500, (int)firstDeg, false, 5, 0};
  PointTable.push_back(dummyPoint);

  sleep(1);
  if (flagZone == 1) {
    gpioWrite(ZoneRed, 1);
    while (!gpioRead(CheckRed)) {
    }
  } else if (flagZone == -1) {
    gpioWrite(ZoneBlue, 1);
    while (!gpioRead(CheckBlue)) {
    }
  }
  ms.send(ShootRID, 10, ShootR);
  ms.send(ShootLID, 10, ShootL);

  cout << "Main Start" << endl;
  gpioWrite(BCheck, 1);
  clock_gettime(CLOCK_REALTIME, &now);
  gyro.start(firstDeg);
  phase = 0;

  // MainLoop
  while (1) {
    //----------Sensor----------
    // time
    prev = now;
    clock_gettime(CLOCK_REALTIME, &now);
    delta = now.tv_sec - prev.tv_sec +
            (long double)(now.tv_nsec - prev.tv_nsec) / 1000000000;
    start += delta;

    // GY521
    gyro.updata();
    yaw = gyro.yaw * flagZone;

    // UltraSonic
    for (int i = 0; i < 3; ++i) {
      int dummyM = ms.send(MeasureID[i], 20, 1);
      if (dummyM > 10) {
        measure[i].push_back(dummyM);
      }
      if (measure[i].size() == MeasureCal) {
        int dummySum = 0;
        for (auto x : measure[i]) {
          dummySum += x;
        }
        switch (i) {
        case 0:
          measureY = (double)dummySum / MeasureCal * 10 + MeasureY0;
          break;
        case 1:
          measureL = (double)dummySum / MeasureCal * 10 + MeasureL0;
          break;
        case 2:
          measureR = (double)dummySum / MeasureCal * 10 + MeasureR0;
          break;
        }
        measure[i].clear();
      }
    }

    // RotaryInc
    for (int i = 0; i < 3; ++i) {
      wheelInPrev[i] = wheelIn[i];
      wheelIn[i] = rotary[i].get();
      wheelSpeed[i] =
          (double)(wheelIn[i] - wheelInPrev[i]) * WheelCirc / (double)Range;
    }

    //----------Reset----------
    if (gpioRead(CheckReset)) {
      gyro.resetYaw(firstDeg);
      nowPoint[0] = firstX * flagZone;
      nowPoint[1] = firstY;
      phase = 0;
      pointCount = 0;
    }

    switch (phase) {
    case 0: {
      if ((ms.send(ShootRID, 10, 0) == 2) && (ms.send(ShootLID, 10, 0) == 2)) {
        goal = PointTable.at(pointCount);
        yawGoal = goal.yaw;
        phase = 1;
        if ((int)PointTable.size() != pointCount + 1) {
          ++pointCount;
        } else if (gpioRead(Finish)) {
          goto Finish;
        }
      }

      break;
    }

    case 1: {
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

      // bia UltraSonic
      switch (goal.ultra) {
      case 1: {
        int dummyY = measureY + 400;
        if (yaw_check(goal.yaw, yaw) && flagNear) {
          int index = (goal.yaw / 90) % 2;
          if (dummyY < measureR && dummyY < measureL) {
            nowPoint[index] = goal.x * !index * flagZone + goal.y * index;
          } else if (dummyY < measureR) {
            nowPoint[index] += UltraSpead * delta * (!index * flagZone + index);
          } else if (dummyY < measureL) {
            nowPoint[index] -= UltraSpead * delta * (!index * flagZone + index;)
          }
        }
      }
      case 2: {
        switch (goal.table) {
        case 1: {
          int dummyY = measureY + 400;
          int index = (goal.yaw / 90) % 2;
          int signe = goal.yaw / 180 - !(goal.yaw / 180);
          if (yaw_check(goal.yaw, yaw)) {
            if (nowPoint[index] >
                    TwoTableX * !index * flagZone + TwoTableY * index - 350 &&
                nowPoint[index] <
                    TwoTableX * !index * flagZone + TwoTableY * index + 350) {
              nowPoint[!index] =
                  (TwoTableX - signe * dummyY) * index * flagZone +
                  (TwoTableY + signe * dummyY) * !index;
            }
          }
          break;
        }
        case 2: {
          int dummyY = measureY + 250;
          if (yaw_check(goal.yaw, yaw)) {
            if (nowPoint[1] > TwoTableY - 200 &&
                nowPoint[1] < TwoTableY + 200) {
              nowPoint[0] = (MoveTableX - dummyY) * flagZone;
            }
          }
          break;
        }
        }
      }
      case 5: {
        break;
      }
      }

      velocityF = hypot(goal.y - nowPoint[1], goal.x - nowPoint[0]);
      angleF = atan2(goal.y - nowPoint[1], goal.x - nowPoint[0]);

      //----------Movement----------
      // Input
      // WheelSpeed Control from  Accel
      if (velocityF < ErrorMin && velocityF > -ErrorMin) {
        velocityF = 0;
        flagNear = true;
      } else if (velocityF < ErrorMax) {
        velocityF = SpeedMax - ErrorReg * log(ErrorMax - velocityF + 1);
        flagNear = true;
        stop = start;
      } else {
        velocityF = SpeedMax;
        flagNear = false;
        stop = start;
      }

      if (goal.table == 5) {
        if (velocityF < HomeMin && velocityF > -HomeMin) {
          velocityF = 0;
          flagNear = true;
        } else if (velocityF < HomeMax) {
          velocityF = HomeSpeadMax - HomeReg * log(HomeMax - velocityF + 1);
          flagNear = true;
          stop = start;
        } else {
          velocityF = HomeSpeadMax;
          flagNear = false;
          stop = start;
        }
      }

      if (flagNear) {
        velocityOut[0] = velocityF * cos(angleF);
        velocityOut[1] = velocityF * sin(angleF);
        velocityR = velocityF;
        angleR = angleF - yaw * M_PI / 180;
      } else {
        velocityGoal[0] = velocityF * cos(angleF);
        velocityGoal[1] = velocityF * sin(angleF);
        accelStart = start;
        for (int i = 0; i < 2; ++i) {
          accelTime[i][0] = (AccelMax - velocityAccel[i]) / Jerk;
          accelTime[i][1] =
              (fabs((double)velocityGoal[i] - velocityOut[i]) -
               (AccelMax + velocityAccel[i]) / 2 * accelTime[i][0] -
               AccelMax * AccelMax / 2 / Jerk) /
              AccelMax;
          accelTime[i][2] = AccelMax / Jerk;
          if (accelTime[i][1] < 0) {
            accelTime[i][1] *= -1;
            accelTime[i][0] +=
                check((AccelMax - velocityAccel[i]) / (2 * accelTime[i][0]),
                      AccelMax, -AccelMax * accelTime[i][1] / 2);
            accelTime[i][2] += check(AccelMax / (2 * accelTime[i][2]), AccelMax,
                                     -AccelMax * accelTime[i][1] / 2);
            accelTime[i][2] += accelTime[i][0];
            accelTime[i][1] = accelTime[i][0];
          } else {
            accelTime[i][1] += accelTime[i][0];
            accelTime[i][2] += accelTime[i][1];
          }
          if (velocityGoal[i] > velocityOut[i]) {
            accelPolar[i] = 1;
          } else {
            accelPolar[i] = -1;
          }

          if (start - accelStart < accelTime[i][0]) {
            velocityAccel[i] += Jerk * delta;
            velocityOut[i] += accelPolar[i] * velocityAccel[i] * delta;
          } else if (start - accelStart < accelTime[i][1]) {
            velocityOut[i] += accelPolar[i] * velocityAccel[i] * delta;
          } else if (start - accelStart <= accelTime[i][2]) {
            velocityAccel[i] -= Jerk * delta;
            velocityOut[i] += accelPolar[i] * velocityAccel[i] * delta;
          }
        }
        velocityR = hypot(velocityOut[0], velocityOut[1]);
        angleR = atan2(velocityOut[1], velocityOut[0]) - yaw * M_PI / 180;
      }

      // moment from Lock Angle bia PID
      switch (goal.table) {
      case 1:
        yawGoal = atan2(TwoTableY - nowPoint[1],
                        (-TwoTableX - nowPoint[0]) * flagZone) *
                  180 / M_PI;
        break;
      }
      yawPrev = yawDelta;
      yawDelta = yawGoal - yaw;
      if (yawDelta > 180) {
        yawDelta -= 360;
      } else if (yawDelta <= -180) {
        yawDelta += 360;
      }
      moment = yawProp * yawDelta + yawInt * yawDelta * delta +
               yawDeff * (yawDelta - yawPrev) / delta;
      moment *= -1 * flagZone;

      if (moment > MomentMax) {
        moment = MomentMax;
      } else if (moment < -MomentMax) {
        moment = -MomentMax;
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

      for (int i = 0; i < 3; ++i) {
        wheelGoal[i] *= wheelSlow;
        if (0 < wheelGoal[i] && wheelGoal[i] < SpeedMin) {
          wheelGoal[i] = 0;
        } else if (0 > wheelGoal[i] && wheelGoal[i] > -SpeedMin) {
          wheelGoal[i] = 0;
        }
      }

      // Output
      // Data
      cout << start << ", ";
      cout << nowPoint[0] << ", " << nowPoint[1] << ", " << yaw;
      /*
      for (int i = 0; i < 3; ++i) {
        cout << (int)wheelGoal[i] << ", ";
      }
      // cout << velocityF << ", " << velocityR << ", " << angleR;
      // cout << velocityF << ", " << velocityR << ", " << angleR;
      cout << nowPoint[0] << ", " << nowPoint[1] << ", " << yaw << endl;
      for (int i = 0; i < 3; ++i) {
        cout << wheelIn[i] << ",";
      }
      */
      cout << endl;

      for (int i = 0; i < 3; ++i) {
        ms.send(WheelID[i], 2, wheelGoal[i]);
      }

      if (start - stop > StopTime) {
        phase = 2;
      }
      break;
    }
    case 2: {
      // Shoot
      if (goal.shoot) {
        if (ms.send(ShootLID, 10, 0) == 2 && !flagShootR) {
          ms.send(ShootRID, 10, ShootR);
          ms.send(1, 30, shineR);
          flagShootR = true;
        } else if (ms.send(ShootRID, 10, 0) == 2) {
          ms.send(ShootLID, 10, ShootL);
          ms.send(1, 30, shineL);
          flagShootR = false;
          phase = 0;
        }
      } else {
        if (goal.ultra == 1) {
          nowPoint[0] -= 90 * fabs(cos(goal.yaw * M_PI / 180));
          nowPoint[1] += 90 * fabs(sin(goal.yaw * M_PI / 180));
        }
        phase = 0;
      }
      break;
    }
    }
  }
finish:
  cout << "Main Finish" << endl;
  ms.send(255, 255, 0);
  gpioWrite(BCheck, 0);
  gpioWrite(ZoneRed, 0);
  gpioWrite(ZoneBlue, 0);
  gpioWrite(LEDCal, 0);
  gpioWrite(LEDReset, 0);
  return restart;
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

inline bool yaw_check(int goal, int now) {
  int small = goal + 360 - ErrorYaw;
  int large = goal + 360 + ErrorYaw;
  now = (now + 360) % 360 + 360;
  if (now > small && now < large) {
    return 1;
  }
  return 0;
}
