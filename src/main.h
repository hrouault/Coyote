#ifndef MAIN_H
#define MAIN_H
/* Small workaround
 * This variables are defined in hw_config.c but are not defined in the
 * respective header hw_config.h, so I redefine them here */
extern uint8_t USART_Rx_Buffer[];
extern uint32_t USART_Rx_ptr_in;
extern uint32_t USART_Rx_ptr_out;

void CatchInputEvents();

#endif // MAIN_H
