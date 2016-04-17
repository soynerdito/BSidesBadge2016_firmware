/*
	Control led matrix for BSides badge
*/
#ifndef _LED_MATRIX
#define _LED_MATRIX

#include "stm32f0xx_hal.h"
#include "spi_helper.h"


/*
 * Function pointers for structure
*/
typedef uint8_t (*type_spi_write_read)( uint8_t byte );
typedef void (*type_clear)( void );
typedef void (*type_write)(uint16_t Row, uint16_t Column );
typedef uint8_t (*type_spi_read)(uint8_t readSize);
typedef void (*type_initialize)( struct _spiControl* SPI_Control, GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin );
//typedef void (*type_clear)( void );


/*
 *	Actual functions that read/write to the SPI
*/
void _latch( void );
void _write(uint16_t Row, uint16_t Column );
void _clear( void);
void _disable( void);
void _initialize(struct _spiControl* SPI_Control, GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin );

/*
 * Helper structure to call SPI in object like fashion
 */
struct _ledMatrix {
	SPI_HandleTypeDef *SPI_Handle;
	type_initialize initialize;
	type_write write;	
	type_clear clear;	
	type_clear disable;	
}  ;

extern struct _ledMatrix ledMatrix;

#endif
