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
 *
 * =====================================================================================
 *
 *       Filename:  pwm.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  02/06/11 03:28:52
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *        Company:  
 *
 * =====================================================================================
 */

#include "stm32_eval.h"
#include "stm32f10x.h"
#include "pwm.h"
#include "adc.h"

int16_t pwm_duty;


void pwm_init()
{
   pwm_duty=0x000;

   /* TIM2 clock enable */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
   /* GPIOA clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
   /* GPIOB clock enable */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
   

   GPIO_InitTypeDef GPIO_InitStructure;

   TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
   TIM_OCInitTypeDef TIM_OCInitStructure;

   /* GPIOA Configuration:TIM2 Channel1 in Output */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // Alternate function, push-pull
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // 50 MHz : the max
   GPIO_Init(GPIOA, &GPIO_InitStructure);

   // Configure PB.0 as direction output
   GPIO_InitTypeDef GPIO_InitStructuredir;
   GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_SET);
   GPIO_InitStructuredir.GPIO_Pin =  GPIO_Pin_0;
   GPIO_InitStructuredir.GPIO_Mode = GPIO_Mode_Out_PP;
   GPIO_InitStructuredir.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_Init(GPIOB, &GPIO_InitStructuredir);
  

   /* ---------------------------------------------------------------
      TIM2 Configuration: PWM Mode:
      --------------------------------------------------------------- */
   /* Time base configuration */
   TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD; // Should give roughly 100kHz
   TIM_TimeBaseStructure.TIM_Prescaler = 0x00; // Gives maximum precision (better to tune the period)
   TIM_TimeBaseStructure.TIM_ClockDivision = 0;
   TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

   TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

   /* Output Compare Toggle Mode configuration: Channel1 */
   TIM_OCInitStructure.TIM_Pulse = pwm_duty; //This is the period (a fraction of 0x200)
   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
   TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
   TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
   TIM_OC2Init(TIM2, &TIM_OCInitStructure);

   TIM_ARRPreloadConfig(TIM2, ENABLE);

   /* TIM enable counter */
   TIM_Cmd(TIM2, ENABLE);
   
}

int32_t preverror;
int32_t pid_int;
int32_t pid_prop;
int32_t pid_deriv;

int32_t output;

void
pid_init()
{
   preverror=0;
   pid_int=0;
   pid_deriv=0;
   pid_prop=0;
}

#define GAIN_P 20
#define GAIN_I 1
#define GAIN_D 100

void
pid_control()
{
   static uint8_t check=0;
   uint32_t pwm_limit;
   pwm_limit= 101*(uint32_t)PWM_PERIOD;
   pwm_limit/=170; //170 at max
   static int8_t sign=0;

   pid_prop=(int32_t)adc_targettemp-adc_curtemp;

   if (pid_prop>500){
      pwm_duty=pwm_limit;
      GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_SET);
      check=1;
      sign=1;
   } else if (pid_prop<-500) {
      pwm_duty=pwm_limit;
      GPIO_WriteBit(GPIOB,GPIO_Pin_0,Bit_RESET);
      check=1;
      sign=-1;
   } else {

      if (check){
         pid_int=0;
         check=0;
         preverror=pid_prop;
      }
      pid_int+=pid_prop;
      pid_deriv=(int32_t)pid_prop-preverror;
      preverror=pid_prop;

      pwm_duty = sign*(GAIN_P*pid_prop + GAIN_I*pid_int + GAIN_D*pid_deriv);
      pwm_duty/=80;
      if (pwm_duty>pwm_limit){
         pwm_duty=pwm_limit;
      }
   }

   TIM2->CCR2 = pwm_duty;
}
