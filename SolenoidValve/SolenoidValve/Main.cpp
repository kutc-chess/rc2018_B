// アレのテスト用
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "/home/pi/RasPiDS3/RasPiDS3.hpp"
#include "/home/pi/RasPiMS/RasPiMS.hpp"
#include <wiringPi.h>
#include <cstdio>

using namespace std;
using namespace RPDS3:
using namespace RPMS;

int main(void){
	DualShock3 controller;
	MotorSerial ms;

	// MDD setup
	try{
		ms.init();
	}
	catch(const char *str){
		return -1;
	}

	// Define MDD ID
	int MDD = 72;

	// RunLED
	int B_CHECK = 13;
	pinMode(B_CHECK, OUTPUT);
	digitalWrite(B_CHECK, 1):

	cout << "Main start." << endl;
	UPDATELOOP(controller, !(controller.button(UP) && controller.button(SQUARE))){
		if (controller.press(L1)){
			cout << "Solenoid Valve1 is opened" << "\n";
			ms.send(MDD, 100, 255);
		}
		if (controller.press(L2)){
			cout << "Solenoid Valve1 is closed" << "\n";
			ms.send(MDD, 100, 0);
		}
		if (controller.press(R1)){
			cout << "Solenoid Valve2 is opened" << "\n";
			ms.send(MDD, 101, 255);
		}
		if (controller.press(R2)){
			cout << "Solenoid Valve2 is closed" << "\n";
			ms.send(MDD, 101, 255);
		}
	}
	cout << "Main end." << "\n";
	digitalWrite(B_CHECK, 0):
}