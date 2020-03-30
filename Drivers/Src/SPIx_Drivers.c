/*
 * SPIx_Drivers.c
 *
 *  Created on: Mar 29, 2020
 *      Author: dubey
 */
#include"SPIx_Drivers.h"

/*********************************************************************************************************
 * @fn 				   - SPI_PCLK_Control															     *
 * 																									     *
 * @brief			   - this Function enables or disable peripheral clock for the given SPI Peripheral  *
 * 																									     *
 * @param[in]		   - base address of the SPIx peripheral										     *
 * @param[in]		   - ENABLE or DISABLE Macros													     *
 * @param[in]          -																				 *
 * 																										 *
 * @return			   -																				 *
 * 																										 *
 * @Note			   -																				 *
 *********************************************************************************************************/

void SPI_PCLK_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			if(pSPIx == SPI1)
				SPI1_PCLK_EN();
			else if(pSPIx == SPI2)
				SPI2_PCLK_EN();
			else if(pSPIx == SPI3)
				SPI3_PCLK_EN();
			else if(pSPIx == SPI4)
				SPI4_PCLK_EN();
			else if(pSPIx == SPI5)
				SPI5_PCLK_EN();
			else if(pSPIx == SPI6)
				SPI6_PCLK_EN();
		}
		else if(EnorDi == DISABLE)
		{
			if(pSPIx == SPI1)
				SPI1_PCLK_DI();
			else if(pSPIx == SPI2)
				SPI2_PCLK_DI();
			else if(pSPIx == SPI3)
				SPI3_PCLK_DI();
			else if(pSPIx == SPI4)
				SPI4_PCLK_DI();
			else if(pSPIx == SPI5)
				SPI5_PCLK_DI();
			else if(pSPIx == SPI6)
				SPI6_PCLK_DI();
		}
}


/*********************************************************************************************************
 * @fn 				   - SPI_Init        															     *
 * 																									     *
 * @brief			   - this Function enables or disable peripheral clock for the given SPI Peripheral  *
 * 																									     *
 * @param[in]		   - base address of the SPIx peripheral										     *
 * @param[in]		   - ENABLE or DISABLE Macros													     *
 * @param[in]          -																				 *
 * 																										 *
 * @return			   -																				 *
 * 																										 *
 * @Note			   -																				 *
 *********************************************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{

}

/*********************************************************************************************************
 * @fn 				   - SPI_DeInit  				        										     *
 * 																									     *
 * @brief			   - this Function enables or disable peripheral clock for the given SPI Peripheral  *
 * 																									     *
 * @param[in]		   - base address of the SPIx peripheral										     *
 * @param[in]		   - ENABLE or DISABLE Macros													     *
 * @param[in]          -																				 *
 * 																										 *
 * @return			   -																				 *
 * 																										 *
 * @Note			   -																				 *
 *********************************************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{

}

/*********************************************************************************************************
 * @fn 				   - SPI_TransmitData		     												     *
 * 																									     *
 * @brief			   - this Function enables or disable peripheral clock for the given SPI Peripheral  *
 * 																									     *
 * @param[in]		   - base address of the SPIx peripheral										     *
 * @param[in]		   - ENABLE or DISABLE Macros													     *
 * @param[in]          -																				 *
 * 																										 *
 * @return			   -																				 *
 * 																										 *
 * @Note			   -																				 *
 *********************************************************************************************************/

void SPI_TransmitData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{

}

/*********************************************************************************************************
 * @fn 				   - SPI_ReceiveData															     *
 * 																									     *
 * @brief			   - this Function enables or disable peripheral clock for the given SPI Peripheral  *
 * 																									     *
 * @param[in]		   - base address of the SPIx peripheral										     *
 * @param[in]		   - ENABLE or DISABLE Macros													     *
 * @param[in]          -																				 *
 * 																										 *
 * @return			   -																				 *
 * 																										 *
 * @Note			   -																				 *
 *********************************************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}


/*********************************************************************************************************
 * @fn 				   - SPI_IRQInterruptConfig														     *
 * 																									     *
 * @brief			   - this Function enables or disable peripheral clock for the given SPI Peripheral  *
 * 																									     *
 * @param[in]		   - base address of the SPIx peripheral										     *
 * @param[in]		   - ENABLE or DISABLE Macros													     *
 * @param[in]          -																				 *
 * 																										 *
 * @return			   -																				 *
 * 																										 *
 * @Note			   -																				 *
 *********************************************************************************************************/
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}

/*********************************************************************************************************
 * @fn 				   - SPI_IRQPriorityConfig															     *
 * 																									     *
 * @brief			   - this Function enables or disable peripheral clock for the given SPI Peripheral  *
 * 																									     *
 * @param[in]		   - base address of the SPIx peripheral										     *
 * @param[in]		   - ENABLE or DISABLE Macros													     *
 * @param[in]          -																				 *
 * 																										 *
 * @return			   -																				 *
 * 																										 *
 * @Note			   -																				 *
 *********************************************************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}

/*********************************************************************************************************
 * @fn 				   -  SPI_IRQHandling															     *
 * 																									     *
 * @brief			   - this Function enables or disable peripheral clock for the given SPI Peripheral  *
 * 																									     *
 * @param[in]		   - base address of the SPIx peripheral										     *
 * @param[in]		   - ENABLE or DISABLE Macros													     *
 * @param[in]          -																				 *
 * 																										 *
 * @return			   -																				 *
 * 																										 *
 * @Note			   -																				 *
 *********************************************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}
