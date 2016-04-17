
#ifndef _EEPROM_
#define _EEPROM_

#include "stm32f0xx_hal.h"
#include "spi_helper.h"

//EEPROM information

/*
 * Function pointers for structure
*/
typedef void (*type_eeprom_write)( uint8_t address, uint16_t data );
typedef uint16_t (*type_eeprom_readData)( uint8_t address );
typedef void (*type_eeprom_disableProgramming)( void  );
typedef void (*type_eeprom_enableWrite)(void);
typedef void (*type_eeprom_erase)(uint8_t address );
typedef void (*type_eeprom_eraseAllChip)( void );
typedef void (*type_eeprom_initialize)(struct _spiControl* SPI_Control, GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin );
typedef void (*type_eeprom_disable)(void);

void eeprom_write( uint8_t address, uint16_t data );
uint16_t eeprom_readData( uint8_t address );
void eeprom_disableProgramming( void  );
void eeprom_enableWrite(void);
void eeprom_erase(uint8_t address );
void eeprom_eraseAllChip( void );
void eeprom_initialize(struct _spiControl* SPI_Control, GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin );
void eeprom_disable(void);

struct _eeprom {
	SPI_HandleTypeDef *SPI_Handle;
	type_eeprom_initialize initialize;
	type_eeprom_write write;
	type_eeprom_readData readData;	
	type_eeprom_disableProgramming disableProg;	
	type_eeprom_enableWrite enableWrite;
	type_eeprom_erase	erase;
	type_eeprom_eraseAllChip	eraseAllChip;
	type_eeprom_disable disableChip;
}  ;

extern struct _eeprom eeprom;

#endif
