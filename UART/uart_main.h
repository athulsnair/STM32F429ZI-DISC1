#ifndef UART_MAIN_H
#define UART_MAIN_H


#define GPIO_AF8_USART5        ((uint8_t)0x08)  /* USART2 Alternate Function mapping     */

#define GPIO_PIN_12_SEL                       12
#define GPIO_PIN_2_SEL                       2             

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_12_SEL
#define USARTx_TX_GPIO_PORT              GPIOC  
#define USARTx_TX_AF                     GPIO_AF8_USART5
#define USARTx_RX_PIN                    GPIO_PIN_2_SEL
#define USARTx_RX_GPIO_PORT              GPIOD 
#define USARTx_RX_AF                     GPIO_AF8_USART5

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      UART5_IRQn
#define USARTx_IRQHandler                UART5_IRQHandler

#define EXTIx_IRQn                 EXTI0_IRQn
#define EXTIx_IRQHandler           EXTI0_IRQHandler

#define GPIO_BUTTON_PIN   0
#define GPIO_BUTTON_PORT  GPIOA

#endif
