#pragma once
#include "Arduino.h"

class ScrpSlave{
public:
    ScrpSlave(byte rede_pin, byte my_id, void (*changeID)(byte id));
    void addCMD(byte cmd, boolean (*proc)(int rx_data, int& tx_data));
    boolean check(void);
private:
    byte my_id;
    byte rede_pin;
    boolean (*procs[256])(int rx_data, int& tx_data);
    void (*changeID)(byte id);
};
