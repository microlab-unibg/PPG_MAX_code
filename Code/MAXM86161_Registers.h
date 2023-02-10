/* Registers MAXM86161*/

//i2c configuration
// slave address = 0b11000100 -> 0xC4
#define MAXM86161_I2C_ADDRESS 0xc4

// Interrupt status register 1
//<< bitwise left shift operator
// 1 << 0 is 1 shifted to the left by 0 position
#define MAXM86161_REG_INTERRUPT_STATUS        0x00
#define MAXM86161_PWR_RDY                     (1 << 0)
#define MAXM86161_DIE_TEMP_RDY                (1 << 2)
#define MAXM86161_LED_COMPB                   (1 << 3)
#define MAXM86161_PROX_INT                    (1 << 4)
#define MAXM86161_ALC_OVF                     (1 << 5)
#define MAXM86161_DATA_RDY                    (1 << 6)
#define MAXM86161_A_FULL                      (1 << 7)

// Interrupt status register 2
#define MAXM86161_REG_INTERRUPT_STATUS        0x01
#define MAXM86161_SHA_DONE                    (1 << 0)

// Interrupt enable register 1
#define MAXM86161_REG_INTERRUPT_ENABLE           0x02
#define MAXM86161_DIE_TEMP_RDY_EN                (1 << 2)
#define MAXM86161_LED_COMPB_EN                   (1 << 3)
#define MAXM86161_PROX_INT_EN                    (1 << 4)
#define MAXM86161_ALC_OVF_EN                     (1 << 5)
#define MAXM86161_DATA_RDY_EN                    (1 << 6)
#define MAXM86161_A_FULL_EN                      (1 << 7)

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




