#ifndef MAXM86161_H
#define MAXM86161_H

#include <stdint.h>
#include "MAXM86161_Registers.h"

class MAXM86161{
    public:
        MAXM86161();
        bool begin();
    
    private:
        void writeRegister(uint8_t address, uint8_t data);
        uint8_t readRegister (uint8_t adrress);
};

#endif