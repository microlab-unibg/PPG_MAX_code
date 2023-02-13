#include "MAXM86161.h"
#include <Wire.h>

//constructor
MAXM86161::MAXM86161(){}

//PPG configuration
void MAXM86161::PPG_config(){
    //initialization ppg configuration 1
     writeRegister(MAXM86161_REG_PPG_CONFIG1,
                                  ((MAXM86161_PPG_CFG_ALC_EN  << MAXM86161_PPG_CFG_ALC)
                                    | (MAXM86161_PPG_CFG_OFFSET_NO << MAXM86161_PPG_CFG_OFFSET)
                                    | (MAXM86161_PPG_ADC_RANGE_16 << MAXM86161_PPG_CFG_ADC_RANGE)
                                    | (MAXM86161_PPG_CFG_TINT_123ms << MAXM86161_PPG_CFG_TINT)));
    
    //inizialization ppg configuration 2 
    writeRegister(MAXM86161_REG_PPG_CONFIG2,
                                  ((MAXM86161_PPG_CFG_SMP_RATE_P1_25sps << MAXM86161_PPG_CFG_SMP_RATE)
                                    | (MAXM86161_PPG_CFG_SMP_AVG_1  << MAXM86161_PPG_CFG_SMP_AVG)));

}

//initialization
bool MAXM86161::begin(){

    //ppg configuration
    //pulse width = 123.8 ms
    //adc range = 16 uA
    // sample average =1
    //sample rate = 25 sps
    PPG_config();

    //led sequence configuration

    
}

// write data
void MAXM86161::writeRegister(uint8_t address, uint8_t data){
    Wire.beginTransmission(MAXM86161_I2C_ADDRESS);
    Wire.write(address);
    Wire.write(data);
    Wire.endTrasmission();
}
//read data
uint8_t MAXM86161::readRegister(uint8_t address){
    Wire.beginTransmission(MAXM86161_I2C_ADDRESS);
    Wire.write(address);
    Wire.endTransimission(false);
    Wire.requestFrom(MAXM86161_I2C_ADDRESS, 1);
    return Wire.read();
}



