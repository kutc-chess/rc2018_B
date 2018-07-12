#include <iostream>
#include <unistd.h>
#include <wiringPi.h>
#include "/home/pi/RasPiDS3/RasPiDS3.hpp"
#include "/home/pi/RasPiMS/RasPiMS.hpp"
using namespace std;
using namespace RPDS3;
using namespace RPMS;

int main(){
    DualShock3 controller;
    MotorSerial ms;

    // Setup
    try{
        ms.init();
    }
    catch(const char *str){
        return -1;
    }

    // RunLED
    int B_CHECK = 13;
    pinMode(B_CHECK, OUTPUT);
    digitalWrite(B_CHECK, 1);
    
    UPDATELOOP(controller, !(controller.button(UP) && controller.button(SQUARE))){
        cout << ms.send(2, 20, 255) << '\n';
        usleep(100000);
    }
    ms.send(255, 255, 0);
    digitalWrite(B_CHECK, 0);
    return 0;
}
