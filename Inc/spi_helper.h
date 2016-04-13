#include "stm32f0xx_hal.h"
#ifndef SPI_HELPER_IMPORT
#define SPI_HELPER_IMPORT


#define CS_SET(GPIO_PORT,PIN) \
    HAL_GPIO_WritePin(GPIO_PORT, PIN, GPIO_PIN_SET )

#define CS_RESET(GPIO_PORT,PIN) \
    HAL_GPIO_WritePin(GPIO_PORT, PIN, GPIO_PIN_RESET )


typedef void (*onSPIError)(void);

void SPIx_Error(SPI_HandleTypeDef *hspi );

/*
 * Function pointers for structure
*/
typedef uint16_t (*type_spi_write_read)( uint8_t byte );
typedef void (*type_spi_write)( uint16_t value);
typedef uint32_t (*type_spi_read)(uint8_t readSize);
typedef void (*type_spi_initialize)( SPI_HandleTypeDef *hspi, onSPIError funcSPIErrorCallback );

/*
 *	Actual functions that read/write to the SPI
*/
uint16_t _spi_write_read( uint8_t byte );
void _spi_write( uint16_t value);
uint32_t _spi_read(uint8_t readSize);
void _spi_initialize( SPI_HandleTypeDef *hspi, onSPIError funcSPIErrorCallback );

/*
 * Helper structure to call SPI in object like fashion
 */
struct _spiControl {
	SPI_HandleTypeDef *SPI_Handle;
	type_spi_initialize initialize;
	type_spi_write_read writeRead;
	type_spi_write write;
	type_spi_read read;
	onSPIError funcSPIErrorCallback;

}  ;

extern struct _spiControl spiControl;


#endif
