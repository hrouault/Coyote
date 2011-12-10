/*
 * =====================================================================================
 *
 *       Filename:  rtc.c
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  08.12.2011 17:35:13
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (),
 *        Company:
 *
 * =====================================================================================
 */
#include <stdio.h>

#include "stm32_eval.h"
#include "stm32f10x.h"
#include "hw_config.h"
#include "pwm.h"
#include "adc.h"
#include "rtc.h"


void RTC_NVIC_configuration()
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    /* Enable the RTC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void RTC_configuration()
{
    /* Select LSE as RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    /* Enable RTC Clock */
    RCC_RTCCLKCmd(ENABLE);
    /* Wait for RTC registers synchronization */
    RTC_WaitForSynchro();
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
    /* Enable the RTC Second */
    RTC_ITConfig(RTC_IT_SEC, ENABLE);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
    /* Set RTC prescaler: set RTC period to 1sec */
    RTC_SetPrescaler(RTC_PRESC);
    /* Wait until last write operation on RTC registers has finished */
    RTC_WaitForLastTask();
}


/**
 * @brief  This function handles RTC global interrupt request.
 * @param  None
 * @retval None
 */


void RTC_IRQHandler(void)
{
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET) {
        /* Clear the RTC Second interrupt */
        RTC_ClearITPendingBit(RTC_IT_SEC);
        /* Wait until last write operation on RTC registers has finished */
        RTC_WaitForLastTask();
        comp_meanadc();
        if (RTC_GetCounter() % 4 == 0 && GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7)) {
            /* output to usb virtual com port */
            uint32_t nbchar;
            nbchar = siprintf((char *)USART_Rx_Buffer, "t %li Kp %li Ki %li kd %li Pi %li  Pp %li  Pd %li Targ %i  T %i  du %i\n\r", RTC_GetCounter() / 4, gain_p, gain_i, gain_d, pid_int, pid_prop, pid_deriv, adc_targettemp / 100, adc_curtemp, (int16_t)pwm_duty);
            USART_Rx_ptr_in = nbchar;
            USART_Rx_ptr_out = 0;
            /* update display */
        }
        pid_control();
    }
}
