#include "MAXM86161.h"
#include <Wire.h>

//constructor
MAXM86161::MAXM86161(){}

//initialization
bool MAXM86161::begin(){}

// write data
void MAXM86161::writeRegister(uint8_t address, uint8_t data){
    Wire.beginTransmission(MAXM86161_I2C_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    Wire.endTrasmission();
}