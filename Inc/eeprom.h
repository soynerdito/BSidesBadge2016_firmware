
#include "stm32f0xx_hal.h"

//EEPROM information
struct stEpromCommand
{
     unsigned sb:1;
     unsigned opCode:2;
     unsigned address:9;
     unsigned data:8;     
};

void doEEPromWrite( uint16_t data, uint8_t address );