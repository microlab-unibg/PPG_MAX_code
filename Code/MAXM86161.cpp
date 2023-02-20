#include "MAXM86161.h"
#include <Wire.h>
#include <windows.h>

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
    writeRegister(MAXM86161_REG_LED_SEQ1,
                                  (( MAXM86161_LEDSQ_RED<< MAXM86161_LEDSQ_SHIFT) | (MAXM86161_LEDSQ_IR )));

    writeRegister(MAXM86161_REG_LED_SEQ2,
                                  ((MAXM86161_LEDSQ_OFF << MAXM86161_LEDSQ_SHIFT) | (MAXM86161_LEDSQ_DIRECT_AMBIENT )));

    writeRegister(MAXM86161_REG_LED_SEQ3,
                                  ((MAXM86161_LEDSQ_OFF << MAXM86161_LEDSQ_SHIFT) | (MAXM86161_LEDSQ_OFF)));
    
}

//reset of software
void MAXM86161::soft_reset(){
    writeRegister(MAXM86161_REG_SYSTEM_CONTROL, MAXM86161_SYS_CTRL_SW_RESET);

}

//fifo configuration
void MAXM86161::FIFO_config(){

    //FIFO INT triggered condition
    uint8_t value = 15;
    writeRegister(MAXM86161_REG_FIFO_CONFIG1, value);

    //FIFO Roll Over enabled
    writeRegister(MAXM86161_REG_FIFO_CONFIG2,
                                  (MAXM86161_FIFO_CFG_2_FULL_TYPE_RPT
                                  | MAXM86161_FIFO_CFG_2_FIFO_READ_DATA_CLR | MAXM86161_FIFO_CFG_2_FIFO_ROLL_OVER ));

    //FIFO_A_FULL interrupt enabled



}


//initialization
bool MAXM86161::begin(){

    //software reset
    soft_reset();
    
    //1ms delay
    Sleep(1);

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

    //FIFO configuration
    FIFO_config();

    //LED SEQ configuration
    //LEDC1= 0x2 LED2 exposure
    //LEDC2= 0x3 LED3 exposure
    //LEDC3= 0x9 DIRECT AMBIENT exposure
    //LEDC4=0X0 NONE
    //LEDC5=0X0 NONE
    //LEDC6=0X0 NONE
    LED_SEQ_config();


    
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



