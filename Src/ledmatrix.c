
#include "ledmatrix.h"


//Actual structure that handles SPI communication
struct _ledMatrix ledMatrix = { 0, _initialize, _write, _clear, _disable };

/*
	Hardware connection information
*/
GPIO_TypeDef* m_GPIOx_CS;
uint16_t m_CS_Pin;
struct _spiControl* m_SPI_Control;

#define selectCS() \
    CS_SET(m_GPIOx_CS, m_CS_Pin )

#define resetCS() \
    CS_RESET(m_GPIOx_CS, m_CS_Pin )

void _write(uint16_t Row, uint16_t Column )
{
	m_SPI_Control->write( Row );
	m_SPI_Control->write( Column );	
	selectCS();
	resetCS();
	//selectCS();
	//deselectCS();
}

void _disable( void)
{
		resetCS();
}

void _clear( void)
{
	_write( 0x00, 0x00 );	
}

void _initialize(struct _spiControl* SPI_Control, GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin )
{
	m_SPI_Control = SPI_Control;
	m_GPIOx_CS = GPIOx;
	m_CS_Pin = GPIO_Pin;
	resetCS();
}	
