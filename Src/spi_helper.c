#include "spi_helper.h"


#ifndef SPI_HELPER_C
#define SPI_HELPER_C


#define SPIx_TIMEOUT_MAX                      ((uint32_t)0x1000)
uint32_t SpixTimeout = SPIx_TIMEOUT_MAX; /*<! Value of Timeout when SPI communication fails */

//Actual structure that handles SPI communication
struct _spiControl spiControl = { 0, _spi_initialize, _spi_write_read ,_spi_write, _spi_read, 0 };


 void _spi_initialize( SPI_HandleTypeDef *hspi, onSPIError funcSPIErrorCallback )
 {
	 spiControl.SPI_Handle = hspi;
	 spiControl.funcSPIErrorCallback = funcSPIErrorCallback;
 }

	/**
  * @brief  Sends a Byte through the SPI interface and return the Byte received 
  *         from the SPI bus.
  * @param  Byte : Byte send.
  * @retval The received byte value
  */ 
uint8_t _spi_write_read( uint8_t byte )
{	
 
	uint8_t receivedbyte = 0x00;
  /* Send a Byte through the SPI peripheral */
  /* Read byte from the SPI bus */
  if(HAL_SPI_TransmitReceive( spiControl.SPI_Handle, (uint8_t*) &byte, (uint8_t*) &receivedbyte, 1, SpixTimeout) != HAL_OK)
  {
    SPIx_Error(spiControl.SPI_Handle);
  }
  
  return receivedbyte;
}

/**
  * @brief SPI Write a byte to device
  * @param Value: value to be written
  * @retval None
  */
void _spi_write( uint8_t value){	
  HAL_StatusTypeDef status = HAL_OK;

  status = HAL_SPI_Transmit( spiControl.SPI_Handle, (uint8_t*) &value, 1, SpixTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    SPIx_Error(spiControl.SPI_Handle);
  }
}


/**
  * @brief SPI Read 4 bytes from device
  * @param  ReadSize Number of bytes to read (max 4 bytes)
  * @retval Value read on the SPI
  */
uint8_t _spi_read(uint8_t readSize){

  HAL_StatusTypeDef status = HAL_OK;
  uint32_t readvalue;
  
  status = HAL_SPI_Receive( spiControl.SPI_Handle, (uint8_t*) &readvalue, readSize, SpixTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Re-Initiaize the BUS */
    SPIx_Error(spiControl.SPI_Handle);
  }
  return readvalue;
}

void SPIx_Error(SPI_HandleTypeDef *hspi )
{
  /* De-initialize the SPI comunication BUS */
  HAL_SPI_DeInit(hspi);
  
  /* Re- Initiaize the SPI comunication BUS */
	if( spiControl.funcSPIErrorCallback!= 0 ){
		spiControl.funcSPIErrorCallback();
	}
  //MX_SPI1_Init( );
}








#endif
