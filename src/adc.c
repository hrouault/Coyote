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
 *       Filename:  adc.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  02/06/11 02:47:30
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
#include "adc.h"
#include "hw_config.h"


/* 
 * Output R :
 * Coefficients:
 *     (Intercept)              1             2             3             4 
 *       7.487e+01     -1.961e+00     2.898e-02    -2.305e-04     7.255e-07
 *  Here temperature are multiplied by 100 and resistance are real value resistances
 */

uint16_t adc_targettemp;
uint16_t adc_curtemp;


#define COEF0  74870
#define COEF1  19610
#define COEF2  28980
#define COEF3  23050
#define COEF4  72550

//   uint32 R13p5,R23p5; // resistors to generate the 3.5V in entry of the current source
//   uint32 R1t; // resistor to generate the 20 microA current
//   uint16 RTp; // resistor in serie with the thermistor
//   uint32 R2a,R2b; // resistors to generate the offset
//   uint32 Rg, Rf; // resistors to generate the amplification
//#define R13P5 30000
//#define R23P5 70000
//#define R1T 75000
//#define RTP 3000
//#define R2A 85000
//#define R2B 15000
//#define RG 20000
//#define RF 46000

#define RESI1 2970
#define RESI2 46700
#define RESI3 14930
#define RESI4 58000
#define RESI5 46700
#define RESI6 53100

   uint32_t
resisttotemp(uint32_t resi)
{
   uint32_t temp,resip,resip2,resip3,resip4,temp0,temp1,temp2,temp3,temp4;
   resip=resi/2;
   resip2=resip*resip;
   resip2/=0x10000;
   resip3=resip2*resip;
   resip3/=0x10000;
   resip4=resip3*resip;
   resip4/=0x10000;

   temp0=COEF0;
   temp1=COEF1*resip;
   temp1/=5000;
   temp2=COEF2*resip2;
   temp2/=3815;
   temp3=COEF3*resip3;
   temp3/=2910;
   temp4=COEF4*resip4;
   temp4/=22204;

   temp=temp0+temp2+temp4;
   temp-=temp1+temp3;
   return temp;
}

uint32_t ampli;
uint32_t offset;

void
adcconv_init()
{
//   ampli=RF*10000;
//   ampli/=RG;
   ampli+=10000;
//   ampli*=I0;
   ampli/=100000;


//   offset=R2B*10000;
//   offset/=R2A+R2B;
   offset*=5;
//   offset*=RF;
//   offset/=RG;
}

   uint32_t
adctoresist(uint32_t adcval)
{
   uint32_t rT;

   uint32_t f=(uint32_t)RESI4*0x10000;
   f/=RESI3;

   uint32_t r=(uint32_t)RESI6*0x10000;
   r/=(RESI6+RESI5);

   rT=adcval*0x100+(f/8)*r;
   rT/=(0x2000+f/8);

   uint32_t resiT=RESI2*rT;
   resiT/=0x10000-rT;
   resiT-=RESI1;
   return resiT;
}

#define ADC1_DR_ADDRESS                  ((uint32_t)0x4001244C)

uint16_t ADC_ConvertedValue[512];
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;

void
adc_init()
{
   /* Enable DMA1 clock */
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

   /* Enable ADC1 clock */
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

   NVIC_InitTypeDef NVIC_InitStructure;

   /* Enable the DMAChannel1 Interrupt */
   NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
   NVIC_Init(&NVIC_InitStructure);


   adc_targettemp=29000;
   adc_curtemp=25000;

   GPIO_InitTypeDef GPIO_InitStructure;
   /* Configure PC.00 (ADC Channel10) as analog input */
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
   GPIO_Init(GPIOC, &GPIO_InitStructure);


   /* DMA Channel1 Configuration ----------------------------------------------*/
   DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_ADDRESS;
   DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValue;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
   DMA_InitStructure.DMA_BufferSize = 512;
   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
   DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
   DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
   DMA_Init(DMA1_Channel1, &DMA_InitStructure);

   DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

   /* Enable DMA Channel1 */
   DMA_Cmd(DMA1_Channel1, ENABLE);
   
   


   /* ADC1 Configuration (ADC1CLK = 14 MHz) -----------------------------------*/
   ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
   ADC_InitStructure.ADC_ScanConvMode = ENABLE;
   ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
   ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
   ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
   ADC_InitStructure.ADC_NbrOfChannel = 1;
   ADC_Init(ADC1, &ADC_InitStructure);

   /* ADC1 Regular Channel10 Configuration */
   ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_41Cycles5);

   /* Enable ADC1's DMA interface */
   ADC_DMACmd(ADC1, ENABLE);
   
   /* Enable ADC1 */
   ADC_Cmd(ADC1, ENABLE);

   /* Start ADC1 Software Conversion */
   ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}


/**
|     * @brief  This function handles DMA1Channel1 interrupt request.
|     * @param: None
|     * @retval: None
|     */

uint32_t adc_mean;
    
void DMA1_Channel1_IRQHandler(void)
{
   /* DMA1 channel used for ADC continous conversion */

//   siprintf(USART_Rx_Buffer,"DMA Interrupt\n\r");
//
//   USART_Rx_ptr_in = 150;
//   USART_Rx_ptr_out = 0;
//   Handle_USBAsynchXfer();
//

   uint32_t adc_mean_temp=0;

   uint16_t i = 0;

   /*  Simple averaging (for developping purpose -> to be improved) */

   /* Test on DMA Channel1 Transfer Complete interrupt */
   if (DMA_GetITStatus(DMA1_IT_TC1))
   {

      /* Clear DMA Channel1 Half Transfer, Transfer Complete and Global interrupt pending bits */
      DMA_ClearITPendingBit(DMA1_IT_GL1);

      /* Start data normalisation when half ADC buffer is full */

      for(i = 0; i < 512 ; i++)
      {
         adc_mean_temp+=ADC_ConvertedValue[i];
      }

      adc_mean=adc_mean_temp;
      adc_curtemp=resisttotemp(adctoresist(adc_mean));
//      pid_control();
   }
}


