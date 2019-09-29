#include<stdint.h>
#include "stm32f4xx.h"
#include "hal_spi_driver.h"
#include "hal_gpio_driver.h"
#include "spi_main.h"
#include "led.h"

/* SPI handle for our SPI device */
spi_handle_t SpiHandle;

int TestReady = 0;


/* slave will reply this data, when master issues read command */
uint8_t slave_reply_data[4]={ 0x55, 0xaa, 0x55, 0xaa};

/* master read/write buffers */
uint8_t master_write_data[]={ 0xa, 0xb, 0xc, 0xd };

uint8_t master_read_buffer[4];

/* configure gpio for spi functionality */
void spi_gpio_init(void)
{
	gpio_pin_conf_t spi_conf;
	
	
	_HAL_RCC_GPIOE_CLK_ENABLE();
	
	/* configure GPIOC_PIN_10 for SPI CLK functionality */ 
	spi_conf.pin = SPI_CLK_PIN;
	spi_conf.mode = GPIO_PIN_ALT_FUN_MODE;
	spi_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	spi_conf.pull = GPIO_PIN_PULL_DOWN;
	spi_conf.speed = GPIO_PIN_SPEED_MEDIUM;
	
	hal_gpio_set_alt_function(GPIOE,SPI_CLK_PIN,GPIO_PIN_AF5_SPI4);
	hal_gpio_init(GPIOE, &spi_conf);
	
	/* configure GPIOC_PIN_11 for SPI MISO functionality */ 
	spi_conf.pin = SPI_MISO_PIN;
	spi_conf.pull = GPIO_PIN_PULL_UP;
	hal_gpio_set_alt_function(GPIOE,SPI_MISO_PIN,GPIO_PIN_AF5_SPI4);
	hal_gpio_init(GPIOE, &spi_conf);
	
	/* configure GPIOC_PIN_12 for SPI MOSI functionality */ 
	spi_conf.pin = SPI_MOSI_PIN;
	spi_conf.pull = GPIO_PIN_PULL_UP;
	hal_gpio_set_alt_function(GPIOE,SPI_MOSI_PIN,GPIO_PIN_AF5_SPI4);
	hal_gpio_init(GPIOE, &spi_conf);
	
}

/* some delay generation */
void delay_gen( )
{
	uint32_t cnt = 800000;
	while(cnt--);
}

//hang on here, if applicaton can not proceed due to error
	
void assert_error(void)
{
	while(1)
	{
	  led_toggle(GPIOG,LED_RED);
		delay_gen();
	}
}

/* function used to compare two buffers */
static uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }
	return 0;
}
	
int main(void)
{
	uint16_t ack_bytes = SPI_ACK_BYTES;
	uint8_t rcv_cmd[2];
	uint8_t ack_buf[2] = {0XD5, 0XE5};
	uint16_t master_cmd;
	
	uint8_t indata;
	uint8_t data = 'b';
	
	spi_gpio_init();
	
	/* To use LED */
	led_init();
	
	/* enable the clock for the SPI2 */
	_HAL_RCC_SPI4_CLK_ENABLE() ;
	
	/*fill up the handle structure */
	SpiHandle.Instance               = SPI_4;
	SpiHandle.Init.Direction         = SPI_ENABLE_2_LINE_UNI_DIR;
	SpiHandle.Init.Mode              = SPI_SLAVE_MODE_SEL;
	SpiHandle.Init.DataSize          = SPI_8BIT_DF_ENABLE;
	SpiHandle.Init.CLKPolarity       = SPI_CPOL_HIGH;
	SpiHandle.Init.CLKPhase          = SPI_SECOND_CLOCK_TRANS;
	SpiHandle.Init.NSS               = SPI_SSM_ENABLE;
	SpiHandle.Init.BaudRatePrescaler = SPI_REG_CR1_BR_PCLK_DIV_32;	
	SpiHandle.Init.FirstBit          = SPI_TX_MSB_FIRST;
	
	
	
	SpiHandle.State = HAL_SPI_STATE_READY;
	
	
	/* Call driver API to initialize the SPI device */
	hal_spi_init(&SpiHandle);
	
		/* Enable the IRQs in the NVIC */
  NVIC_EnableIRQ(SPI4_IRQn);
	 
while(1)
{
	/*Make sure that driver state is ready */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* Receive the master command first */
	hal_spi_slave_rx(&SpiHandle, rcv_cmd,CMD_LENGTH );
	
	/* wait until driver finishes RXing and state becomes ready again */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* This is the command slave got */
	master_cmd = (uint16_t)(rcv_cmd[1] << 8 | rcv_cmd[0]);
	
	/* is it a valid command ? */
	if(master_cmd == CMD_MASTER_WRITE || master_cmd == CMD_MASTER_READ )
	{ 
    /* yes, send out the ACK bytes */		
		hal_spi_slave_tx(&SpiHandle, (uint8_t *)&ack_buf, ACK_LEN);
		while(SpiHandle.State != HAL_SPI_STATE_READY );
		led_turn_on(GPIO_PORT_G, LED_GREEN);
		
  }else
	{
		/*  No, Error !*/
		led_turn_on(GPIO_PORT_G, LED_RED);
	}
	
}

	return 0;
}



/**
  * @brief  This function handles SPI2 interrupt request.
  * @param  None
  * @retval None
  */
void SPI4_IRQHandler(void)
{
	/* call the driver api to process this interrupt */
  hal_spi_irq_handler(&SpiHandle);
}
