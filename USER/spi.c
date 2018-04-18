/**
  ******************************************************************************
  * @file    spi.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    
  * @brief   SLAVE SPI
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "string.h"
/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */
#include "stm32f4xx_spi.h"
#include "spi.h"
/** @addtogroup IO_Toggle
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
#define SPI_QUARY_TIMOUT 50000


enum{
    SPI_IDLE= 0x0,
    GET_OPT,    /* 1 */
    GET_REG,    /* 2 */
    GET_LEN,    /* 3 */
    GET_DATA,   /* 4 */
    HANDLE,
    NUL,
};

enum{
    SPI_WRITE = 0,
    SPI_READ,
    SPI_TAIL
};

struct spi_context spi_ctx;

uint8_t reg_map[MAX_REG_IDX][REG_BUF_LEN] = {
	
	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},		/* register index = 0 */
	{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,},		/* register index = 1 */
	{2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,},		/* register index = 2 */
	{3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,},		/* register index = 3 */
};

volatile static uint8_t recvflag = 0;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
void SPIx_Init(void)
{
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */
	SPI_InitTypeDef SPI_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	SPI_Cmd(SPI1, DISABLE);
    SPI_I2S_DeInit(SPI1);
	
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	/**SPI1 GPIO Configuration  
	PA5   ------> SPI1_SCK
	PA6   ------> SPI1_MISO
	PA7   ------> SPI1_MOSI 
	*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	

	/* SPI Init */

	SPI_InitStruct.SPI_Mode = SPI_Mode_Slave;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStruct);

	/* NVIC Init */
	NVIC_InitStruct.NVIC_IRQChannel = SPI1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE | SPI_I2S_IT_RXNE | SPI_I2S_IT_ERR, ENABLE);
	SPI_Cmd(SPI1, ENABLE);

	return ;
}

/**
  * @brief  Slave SPI send data.
  * @param  None
  * @retval None
  */
void spi_sendchar(uint8_t *sendbuff, uint8_t len)
{
    int tim_cnt = 0;

    while(len)
    {
        len--;
        for(tim_cnt=0;tim_cnt<SPI_QUARY_TIMOUT;tim_cnt++)
            if(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE ) != RESET)
                break; 

        spi_ctx.tx_buff[0] = sendbuff[0];

        SPI_I2S_SendData(SPI1, (uint16_t)spi_ctx.tx_buff[0]);
        
        
        for(tim_cnt=0;tim_cnt<SPI_QUARY_TIMOUT;tim_cnt++)
            if(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY ) == RESET)
                break;  


        for(tim_cnt=0;tim_cnt<SPI_QUARY_TIMOUT;tim_cnt++)
            if(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE ))
                break; 
            
        (void)SPI_I2S_ReceiveData(SPI1);
        
    }
}

/**
  * @brief  Set receive flag.
  * @param  None
  * @retval None
  */
static void spi_set_recvflag(void)
{
	recvflag = 1;
}

/**
  * @brief	Clear receive flag.
  * @param	None
  * @retval None
  */
static void spi_clear_recvflag(void)
{
	recvflag = 0;
}

/**
  * @brief	Clear receive flag.
  * @param	None
  * @retval None
  */
uint8_t SPI_Get_recvflag(void)
{
	return recvflag;
}

/**
  * @brief  Get data send by master.
  * @param  None
  * @retval None
  */
void SPI_GetData(uint8_t *buff, uint8_t *len)
{
	memcpy(buff, spi_ctx.rx_buff, spi_ctx.rx_cnt);
	*len = spi_ctx.rx_cnt;
	spi_clear_recvflag();
}

/**
  * @brief  Slave SPI read request.
  * @param  None
  * @retval None
  */
static void spi_upper_read(void)
{
    if(spi_ctx.len > 10)
    {
        spi_ctx.len = 10;
    }
    spi_sendchar(&reg_map[spi_ctx.reg][0], spi_ctx.len);
}

/**
  * @brief  Slave SPI write.
  * @param  None
  * @retval None
  */
static void spi_upper_write(void)
{
	spi_set_recvflag();
}

/**
  * @brief  Slave SPI handle request.
  * @param  None
  * @retval None
  */
static void slave_spi_handle(int handle)
{
	if(handle == SPI_WRITE)
	{
		spi_upper_write();
	}
	else
	{
		spi_upper_read();
	}
}


/**
  * @brief  Handle SPI interrupt request.
  * @param  None
  * @retval None
  */
void SPI_IrqHandlerCallback(void)
{
    uint16_t cr1 = SPI1->CR1;
    uint16_t cr2 = SPI1->CR2;
    uint16_t sr = SPI1->SR;
    uint8_t x = 0xff;
    
    if(sr & SPI_I2S_FLAG_RXNE)
    {
        /* read data && clear rxne flag */
        x = SPI_I2S_ReceiveData(SPI1);
    }
    
    switch(spi_ctx.state)
    {
        case SPI_IDLE:
        {
			spi_ctx.reg = 0xff;
			spi_ctx.len= 0;
            spi_ctx.state = GET_OPT;
            SPI_I2S_ClearFlag(SPI1, SPI_I2S_FLAG_TXE | SPI_I2S_FLAG_RXNE);
        }
        case GET_OPT:
        {
			spi_ctx.opt = x;
            if((spi_ctx.opt == SPI_READ) || (spi_ctx.opt == SPI_WRITE))
            {
                spi_ctx.state = GET_REG;
            }
            else
            {
                spi_ctx.state = SPI_IDLE;
            }

            break;
        }
        case GET_REG:
        {
            spi_ctx.reg = x;
            spi_ctx.state = GET_LEN;
            break;
        }
        case GET_LEN:
        {
            spi_ctx.len = x;
            if(spi_ctx.opt == SPI_READ)
            {
                slave_spi_handle(SPI_READ);
                spi_ctx.state = SPI_IDLE;
            }
            else
            {
                spi_ctx.state = GET_DATA;
            }
           
            break;
        }
        case GET_DATA:
        {
            spi_ctx.rx_buff[spi_ctx.rx_cnt++]= x;
            if(spi_ctx.len == spi_ctx.rx_cnt)
            {
                slave_spi_handle(SPI_WRITE);
                spi_ctx.rx_cnt= 0;
                spi_ctx.state = SPI_IDLE;
                break;
            }
            else
            {
                spi_ctx.state = GET_DATA;
				break;
            }
        }
        default:
        {
            break;
        }
    } 

    x = 0xff;

    return;
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

