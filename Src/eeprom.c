#include "eeprom.h"


/*
	Hardware connection information
*/
GPIO_TypeDef* m_EEPROM_GPIOx_CS;
uint16_t m_EEPROM_CS_Pin;
struct _spiControl* m_EEPROM_SPI_Control;
	
const uint8_t READ	= 0x6;
const uint8_t EWEN = 0x4;
const uint8_t ERASE = 0x7;
const uint8_t WRITE = 0x5;
const uint8_t ERAL = 0x4;
const uint8_t WRAL = 0x4;
const uint8_t EWDS = 0x4;

const uint8_t ERAL_DATA = 0x80;
const uint8_t EWEN_DATA = 0xFF;

struct _eeprom eeprom = { 0, eeprom_initialize, eeprom_write, eeprom_readData, 
			eeprom_disableProgramming, eeprom_enableWrite, eeprom_erase, eeprom_eraseAllChip, eeprom_disable };

#define EEPROM_CS_SET() \
		CS_RESET(m_EEPROM_GPIOx_CS, m_EEPROM_CS_Pin ); \
    CS_SET(m_EEPROM_GPIOx_CS, m_EEPROM_CS_Pin )

#define EEPROM_CS_SET_RESET() \
		CS_RESET(m_EEPROM_GPIOx_CS, m_EEPROM_CS_Pin ); \
		HAL_Delay(1); \
		CS_SET(m_EEPROM_GPIOx_CS, m_EEPROM_CS_Pin ); \
		CS_RESET(m_EEPROM_GPIOx_CS, m_EEPROM_CS_Pin )

#define EEPROM_CS_RESET_SET() \
		CS_RESET(m_EEPROM_GPIOx_CS, m_EEPROM_CS_Pin ); \
		HAL_Delay(1); \
		CS_SET(m_EEPROM_GPIOx_CS, m_EEPROM_CS_Pin )		
		
#define EEPROM_CS_RESET() \
    CS_RESET(m_EEPROM_GPIOx_CS, m_EEPROM_CS_Pin )

uint8_t local;
void eeprom_write( uint8_t address, uint16_t data )
{
	EEPROM_CS_SET();
	//Build instruction		
	m_EEPROM_SPI_Control->write(0x00);
	m_EEPROM_SPI_Control->write(WRITE);	
	local = ((data >> 8)& 0xFF);
	m_EEPROM_SPI_Control->write( address );
	m_EEPROM_SPI_Control->write( local );
	m_EEPROM_SPI_Control->write( ((data) & 0xFF) );		
	//m_EEPROM_SPI_Control->write( ((data) & 0xFF) );
	//m_EEPROM_SPI_Control->write(0x00);
	EEPROM_CS_RESET_SET();
}

/*
	Horrible code, yes I know.
	Long time trying to make this work, and like this ran.
	Beautify is a job for another day.
	My refactor tries haven't work so...
	Note to self: leave it and keep work on other stuff
	Eventually I will get back to this (God knows when)
*/
uint16_t _fieldValue;
uint8_t _fieldValue1;
uint8_t buffer2;
uint16_t eeprom_readData( uint8_t address )
{
	
	EEPROM_CS_SET();
	_fieldValue = 0x0000;
	m_EEPROM_SPI_Control->write(_fieldValue);
	//Read something just to read something	
	
	m_EEPROM_SPI_Control->write(READ);
	m_EEPROM_SPI_Control->write(address);	
	_fieldValue1 =  spiControl.writeRead(address);
	_fieldValue = _fieldValue1 & 0x00FF;
	_fieldValue = _fieldValue << 8;	
	buffer2 = spiControl.read( 1 );	
	_fieldValue = _fieldValue | buffer2;
	buffer2 = spiControl.read( 1 );
	_fieldValue = _fieldValue << 1;
	_fieldValue = _fieldValue | ( buffer2 >> 7 );
	EEPROM_CS_RESET_SET();
	return _fieldValue;	
}



//EWDS
void eeprom_disableProgramming( void  )
{
	EEPROM_CS_SET();
	m_EEPROM_SPI_Control->write( EWDS );
	m_EEPROM_SPI_Control->write( 0x00 );
	EEPROM_CS_RESET_SET();
	
}

void eeprom_enableWrite(void)
{
	EEPROM_CS_SET();
	m_EEPROM_SPI_Control->write( EWEN );
	m_EEPROM_SPI_Control->write( EWEN_DATA );
	EEPROM_CS_RESET_SET();
}

//Not tested
void eeprom_erase(uint8_t address )
{
	EEPROM_CS_SET();
	m_EEPROM_SPI_Control->write( ERASE );
	m_EEPROM_SPI_Control->write( address );
	EEPROM_CS_RESET_SET();
}

void eeprom_disable()
{
	EEPROM_CS_RESET();
}

//Not tested
void eeprom_eraseAllChip( void )
{
	EEPROM_CS_SET();
	m_EEPROM_SPI_Control->write( ERAL );
	m_EEPROM_SPI_Control->write( ERAL_DATA );
	EEPROM_CS_RESET_SET();
}


void eeprom_initialize(struct _spiControl* SPI_Control, GPIO_TypeDef * GPIOx, uint16_t GPIO_Pin )
{
	m_EEPROM_SPI_Control = SPI_Control;
	m_EEPROM_GPIOx_CS = GPIOx;
	m_EEPROM_CS_Pin = GPIO_Pin;	
	
	
}	
