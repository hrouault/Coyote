/* Copyright (c) 2010,2011, Hervé Rouault
 * All rights reserved.
 *
 * This file is part of Coyotte
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <organization> nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>


#include "stm32_eval.h"
#include "stm32f10x.h"
#include "main.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "pwm.h"
#include "adc.h"
#include "rtc.h"
#include "display.h"
//#include "touchscreen.h"
//#include "graphicObject.h"
//#include "TscHal.h"

int main(void)
{
    //    Set_System();
    //    Set_USBClock();
    //    USB_Interrupts_Config();
    //    USB_Init();
    /*    // Configure PC.12 as output push-pull (LED)
     *    GPIO_InitTypeDef GPIO_InitStructure;
     *    GPIO_WriteBit(GPIOC,GPIO_Pin_12,Bit_SET);
     *    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
     *    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     *    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     *    GPIO_Init(GPIOC, &GPIO_InitStructure);
     */
    //    adc_init();
    //    pwm_init();
    //    pid_init();
    //    RTC_NVIC_configuration();
    //    RTC_configuration();
    display_init();
    /* Infinite main loop ------------------------------------------------------*/
    //    TSC_Init();
    while (1) {
        /* Catching touch events */
        if (TS_IsCalibrationDone() == SET) {
            ProcessInputData();
        }
        /* Time out calculate for power saving mode */
        TimeOutCalculate();
        CatchInputEvents();
    }
}

void CatchInputEvents()
{
    /* catch touch screen events */
    TSC_Read();
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t * file, uint32_t line)
{
    /* Infinite loop */
    while (1)
    {}
}
#endif
