#include <stdint.h>
#include "gpio_driver.h"
#include "led.h"


/**
	* @brief  Initialize the LEDs 
	* @param  None
	* @retval None
	*/
void led_init(void)
{
	gpio_pin_conf_t led_pin_conf;
	
	/* enable the clock for the GPIOD port */
	_HAL_RCC_GPIOG_CLK_ENABLE();
	
	led_pin_conf.pin = LED_GREEN;
	led_pin_conf.mode = GPIO_PIN_OUTPUT_MODE;
	led_pin_conf.op_type = GPIO_PIN_OP_TYPE_PUSHPULL;
	led_pin_conf.pull = GPIO_PIN_NO_PULL_PUSH;
	led_pin_conf.speed = GPIO_PIN_SPEED_MEDIUM;
	hal_gpio_init(GPIO_PORT_G, &led_pin_conf);
	
	led_pin_conf.pin = LED_RED;
	hal_gpio_init(GPIO_PORT_G, &led_pin_conf);
}

/**
	* @brief  Turns ON the led which is connected on the given pin  
	* @param  *GPIOx : Base address of the GPIO Port
	* @param  Pin : pin number of the LED
	* @retval None
	*/
void led_turn_on(GPIO_TypeDef *GPIOx, uint16_t pin)
{
	hal_gpio_write_to_pin(GPIOx, pin, 1); 
}

/**
	* @brief  Turns OFF the led which is connected on the given pin  
	* @param  *GPIOx : Base address of the GPIO Port
	* @param  Pin : pin number of the LED
	* @retval None
	*/
void led_turn_off(GPIO_TypeDef *GPIOx, uint16_t pin)
{
	hal_gpio_write_to_pin(GPIOx,pin, 0);
}

/**
	* @brief  Toggels the led which is connected on the given pin  
	* @param  *GPIOx : Base address of the GPIO Port
	* @param  Pin : pin number of the LED
	* @retval None
	*/
void led_toggle(GPIO_TypeDef *GPIOx, uint16_t pin)
{
	if(hal_gpio_read_from_pin(GPIOx, pin))
		hal_gpio_write_to_pin(GPIOx, pin, 0);
	else
		hal_gpio_write_to_pin(GPIOx, pin, 1);
}

int main(void)
{
	uint32_t i;
	led_init();
	
	_HAL_RCC_GPIOA_CLK_ENABLE();
	
	hal_gpio_configure_interrupt(GPIO_BUTTON_PIN, INT_FALLING_EDGE);
	hal_gpio_enable_interrupt(GPIO_BUTTON_PIN, EXTI0_IRQn);
	
	while(1)
	{
		led_turn_on(GPIO_PORT_G, LED_GREEN);
		//led_turn_on(GPIO_PORT_G, LED_RED);
		
		for(i=0;i<500000;i++);
		
		led_turn_off(GPIO_PORT_G, LED_GREEN);
		//led_turn_off(GPIO_PORT_G, LED_RED);
		
		for(i=0;i<500000;i++);
		
		/*led_toggle(GPIO_PORT_G, LED_GREEN);
		led_toggle(GPIO_PORT_G, LED_RED);
		for(i=0;i<500000;i++);*/
		
		
		
	}
	
	while(1);
}

void EXTI0_IRQHandler(void)
{
	hal_gpio_clear_interrupt(GPIO_BUTTON_PIN);
	
	/* Do your Task here */
	//led_toggle(GPIO_PORT_G, LED_GREEN);
	led_toggle(GPIO_PORT_G, LED_RED);
}
