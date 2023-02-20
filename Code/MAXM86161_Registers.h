/* Registers MAXM86161*/

//i2c configuration
// slave address = 0b11000100 -> 0xC4
#define MAXM86161_I2C_ADDRESS 0xc4



/***************************************************************************//**
 ********    max86161 Register and Parameter Bit Definitions  ******************
 ******************************************************************************/


// Interrupt status register 1
//<< bitwise left shift operator
// 1 << 0 is 1 shifted to the left by 0 position
#define MAXM86161_REG_INTERRUPT_STATUS        0x00
#define MAXM86161_PWR_RDY                     0
#define MAXM86161_DIE_TEMP_RDY                2
#define MAXM86161_LED_COMPB                   3
#define MAXM86161_PROX_INT                    4
#define MAXM86161_ALC_OVF                     5
#define MAXM86161_DATA_RDY                    6
#define MAXM86161_A_FULL                      7

// Interrupt status register 2
#define MAXM86161_REG_INTERRUPT_STATUS        0x01
#define MAXM86161_SHA_DONE                    (1 << 0)

// Interrupt enable register 1
#define MAXM86161_REG_INTERRUPT_ENABLE           0x02
#define MAXM86161_DIE_TEMP_RDY_EN                2
#define MAXM86161_LED_COMPB_EN                   3
#define MAXM86161_PROX_INT_EN                    4
#define MAXM86161_ALC_OVF_EN                     5
#define MAXM86161_DATA_RDY_EN                    6
#define MAXM86161_A_FULL_EN                      7

// Interrupt enable register 2
#define MAXM86161_REG_INTERRUPT_ENABLE_2         0x03
#define MAXM86161_SHA_DONE_EN                	(1 << 0)

//FIFO
// FIFO control and data registers
#define MAXM86161_REG_FIFO_WRITE_POINTER         0x04
#define MAXM86161_REG_FIFO_READ_POINTER          0x05
#define MAXM86161_REG_FIFO_OVERFLOW_COUNTER      0x06
#define MAXM86161_REG_FIFO_DATA_COUNTER          0x07
#define MAXM86161_REG_FIFO_DATA                  0x08  
#define MAXM86161_REG_FIFO_CONFIG1		         0x09
#define MAXM86161_REG_FIFO_CONFIG2		         0x0A

//register ppg configuration
#define MAXM86161_REG_PPG_CONFIG1               0x11
#define MAXM86161_REG_PPG_CONFIG2               0x12
#define MAXM86161_REG_PPG_CONFIG3               0x13
#define MAXM86161_REG_PROX_INT_THRESHOLD	    0x14
#define MAXM86161_REG_PD_BIAS				    0x15



//left bit shift ppg config1
#define MAXM86161_PPG_CFG_ALC                           7
#define MAXM86161_PPG_CFG_OFFSET                        6
#define MAXM86161_PPG_CFG_ADC_RANGE                     2
#define MAXM86161_PPG_CFG_TINT                          0
//left bit shift ppg config2
#define MAXM86161_PPG_CFG_SMP_RATE                      3
#define MAXM86161_PPG_CFG_SMP_AVG                       0

//LEd sequence control
#define MAXM86161_REG_LED_SEQ1		0x20
#define MAXM86161_REG_LED_SEQ2		0x21
#define MAXM86161_REG_LED_SEQ3		0x22

//LED pulse amplitude
#define MAXM86161_REG_LED1_PA			  0x23
#define MAXM86161_REG_LED2_PA			  0x24
#define MAXM86161_REG_LED3_PA			  0x25
#define MAXM86161_REG_LED_PILOT_PA	      0x29
#define MAXM86161_REG_LED_RANGE1		  0x2A

//left bit shift
#define MAXM86161_LED_RANGE_SHIFT_GREEN                 0
#define MAXM86161_LED_RANGE_SHIFT_IR                    2
#define MAXM86161_LED_RANGE_SHIFT_RED                   4



//Led sequence shift
#define MAXM86161_LEDSQ_SHIFT                           4

//System
#define MAXM86161_REG_SYSTEM_CONTROL		0x0D




// ******************PARAMETERS SETUP*************

// ******************INIT***********************
#define MAXM86161_INT_1_FULL                            0x80
#define MAXM86161_INT_1_NOT_FULL                        0x00
#define MAXM86161_INT_1_DATA_RDY                        0x40
#define MAXM86161_INT_1_DATA_NOT_RDY                    0x00
#define MAXM86161_INT_1_ALC_OVERFLOW                    0x20
#define MAXM86161_INT_1_ALC_NORMAL                      0x00
#define MAXM86161_INT_1_PROXIMITY_INT                   0x10
#define MAXM86161_INT_1_PROXIMITY_NORMAL                0x00
#define MAXM86161_INT_1_LED_COMPLIANT                   0x08
#define MAXM86161_INT_1_LED_NOT_COMPLIANT               0x00
#define MAXM86161_INT_1_DIE_TEMP_RDY                    0x04
#define MAXM86161_INT_1_DIE_TEMP_NOT_RDY                0x00
#define MAXM86161_INT_1_PWR_RDY                         0x01
#define MAXM86161_INT_1_PWR_NOT_RDY                     0x00


// ******************CONFIGURATION PPG*************
//CONFIGURATION 1
//ALC(bit 7)
#define MAXM86161_PPG_CFG_ALC_EN                        0x01
#define MAXM86161_PPG_CFG_ALC_DS                        0x00
//OFFSET(bit 6)
#define MAXM86161_PPG_CFG_OFFSET_ADD                    0x01
#define MAXM86161_PPG_CFG_OFFSET_NO                     0x00
//PPG1_PPG1_ADC_RGE (bit2-3)
#define MAXM86161_PPG_ADC_RANGE_16                      0x02
//PPG_TINT (bit1-0) = pulse width
//I set pulse width to 123,8 ms
#define MAXM86161_PPG_CFG_TINT_123ms                    0x03

//CONFIGURATION 2
//sample rate (I set 25sps)
#define  MAXM86161_PPG_CFG_SMP_RATE_P1_25sps            0x00
//sample avereage (I set to 1)
#define MAXM86161_PPG_CFG_SMP_AVG_1                     0x00

//********************LED CURRENT*********************
// I set drive current to 15,36 mA
#define MAXM86161_DRV_LED_PA_15mA                       0x20
// I set LED Driver range to 124mA
#define MAXM86161_LED_RANGE_CURRENT_124_MA              0x03

//********************LED SEQUENCE*********************
#define MAXM86161_LEDSQ_OFF                             0x0
#define MAXM86161_LEDSQ_GREEN                           0x1
#define MAXM86161_LEDSQ_IR                              0x2
#define MAXM86161_LEDSQ_RED                             0x3
#define MAXM86161_LEDSQ_PILOT_LED1                      0x8
#define MAXM86161_LEDSQ_DIRECT_AMBIENT                  0x9


//********************FIFO*********************
#define MAXM86161_FIFO_CFG_2_FLUSH_FIFO                 0x10
#define MAXM86161_FIFO_CFG_2_FIFO_READ_DATA_CLR         0x08
#define MAXM86161_FIFO_CFG_2_FIFO_STAT_DATA_CLR         0x00
#define MAXM86161_FIFO_CFG_2_FULL_TYPE_RPT              0x00
#define MAXM86161_FIFO_CFG_2_FULL_TYPE_ONCE             0x04
#define MAXM86161_FIFO_CFG_2_FIFO_ROLL_OVER             0x02
#define MAXM86161_FIFO_CFG_2_FIFO_STOP                  0x00

//********************SOFT RESET*********************
#define MAXM86161_SYS_CTRL_SW_RESET                     0x01
#define MAXM86161_SYS_CTRL_SHUT_DOWN                    0x02