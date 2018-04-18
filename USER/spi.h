/**
  ******************************************************************************
  * @file    spi.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   This file contains all the functions prototypes for the CRC firmware 
  *          library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @addtogroup CRC
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define MX_SPI_BUFF_SZ 127

struct spi_context
{
    uint8_t state;
    uint8_t reg;
    uint8_t opt;
    uint8_t len;
    uint8_t rx_buff[MX_SPI_BUFF_SZ];
    uint8_t rx_cnt;
    uint8_t tx_buff[MX_SPI_BUFF_SZ];
    uint8_t tx_cnt;
};


#define MAX_REG_IDX 10
#define REG_BUF_LEN 12

enum reg_name{

    SYNC = 1,
    CAN_MESG,
    
    END
};

extern uint8_t reg_map[MAX_REG_IDX][REG_BUF_LEN];



/** @defgroup CRC_Exported_Constants
  * @{
  */
	
extern void SPIx_Init(void);

extern void SPI_IrqHandlerCallback(void);

extern void SPI_GetData(uint8_t *buff, uint8_t *len);

extern uint8_t SPI_Get_recvflag(void);

/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/  



#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

