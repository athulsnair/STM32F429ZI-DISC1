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
	uint32_t cnt = 100000;
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
	uint32_t i=0;
	uint8_t addrcmd[CMD_LENGTH];
	uint8_t data = 'a';
	uint8_t indata;
	uint8_t ack_buf[2];
	
	spi_gpio_init();
	
	/* To use LED */
	led_init();
	
	/* Configure USER Button interrupt*/
	_HAL_RCC_GPIOA_CLK_ENABLE();
	hal_gpio_configure_interrupt(GPIO_BUTTON_PIN, INT_FALLING_EDGE);
	hal_gpio_enable_interrupt(GPIO_BUTTON_PIN,EXTI0_IRQn);
	
	/* enable the clock for the SPI2 */
	_HAL_RCC_SPI4_CLK_ENABLE() ;
	
	/*fill up the handle structure */
	SpiHandle.Instance               = SPI_4;
	SpiHandle.Init.Direction         = SPI_ENABLE_2_LINE_UNI_DIR;
	SpiHandle.Init.Mode              = SPI_MASTER_MODE_SEL;
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

/* First initilaize the Debug UART */
	//hal_debug_uart_init( DEBUG_USART_BAUD_9600);
	
	//uart_printf("SPI master Application Running ... \n");
	
	
/* Wait for user Button press before starting the communication. Toggles LED_ORANGE until then */
  while(TestReady != SET)
  {
    led_toggle(GPIOG,LED_GREEN);
		//LED3 (orange)
    delay_gen();
  }
	
	 led_turn_off(GPIOG,LED_GREEN);
	
/******************************************************************************/
/*                                                                            */
/*                        Master command sending code                         */
/*                         Write and read commands                            */
/******************************************************************************/

while(1)
{
	//check for state ready 
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* Master write command */
	addrcmd[0] = (uint8_t) CMD_MASTER_WRITE;
	addrcmd[1] = (uint8_t) ( CMD_MASTER_WRITE >> 8 );
	
	/* first send the master write cmd to slave */
	hal_spi_master_tx(&SpiHandle, addrcmd,CMD_LENGTH );
	
	/* application can block here, or can do other task untill above tx finishes */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* this dealy helps for the slave to be ready with the ACK bytes */
	delay_gen();
	
	/* read back the ACK bytes from the slave */
	hal_spi_master_rx(&SpiHandle,ack_buf, ACK_LEN);
	
	/* wait untill ACK reception finishes */
	while(SpiHandle.State != HAL_SPI_STATE_READY );
	
	/* did we rcv the valid ACK from slave ?? */
	if(ack_buf[1] == 0XE5 && ack_buf[0] == 0xD5 )
	{
		//correct ack 
		led_turn_on(GPIO_PORT_G, LED_GREEN);
		memset(ack_buf,0,2);
	}
	else
	{
		//invalide ack 
		assert_error();
		memset(ack_buf,0,2);
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


/**
  * @brief  This function handles EXTI0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	/* In the ISR, first clear out the sticky interrupt pending bit for this interrupt */
  hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	/* Do your task here */
	TestReady = SET;
}


