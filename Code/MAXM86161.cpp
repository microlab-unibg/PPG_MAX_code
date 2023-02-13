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

//LED PA configuration
void MAXM86161::LED_PA_config(){
    //initialitazion led_PA config
    writeRegister( MAXM86161_REG_LED1_PA, MAXM86161_DRV_LED_PA_15mA );
    writeRegister( MAXM86161_REG_LED2_PA, MAXM86161_DRV_LED_PA_15mA );
    writeRegister( MAXM86161_REG_LED3_PA, MAXM86161_DRV_LED_PA_15mA );
}

//LED RANGE configuration
void MAXM86161::LED_RANGE_config(){
    writeRegister(MAXM86161_REG_LED_RANGE1,
                                  ((MAXM86161_LED_RANGE_CURRENT_124_MA << MAXM86161_LED_RANGE_SHIFT_GREEN)
                                   | (MAXM86161_LED_RANGE_CURRENT_124_MA << MAXM86161_LED_RANGE_SHIFT_IR)
                                   | (MAXM86161_LED_RANGE_CURRENT_124_MA << MAXM86161_LED_RANGE_SHIFT_RED)));

}

//LED SEQ configuration
void MAXM86161::LED_SEQ_config(){
    
}

//initialization
bool MAXM86161::begin(){

    //ppg configuration
    //pulse width = 123.8 ms
    //adc range = 16 uA
    // sample average =1
    //sample rate = 25 sps
    PPG_config();

    //LED PA configuration
    //drive current to 15 mA
    LED_PA_config();

    //LED RANGE configuration
    //range current of 124 mA
    LED_RANGE_config();

    //LED SEQ configuration

    
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



