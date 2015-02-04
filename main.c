#include <stm32f0xx_exti.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_misc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_syscfg.h>

/*
 * A test program for the DHT22 humidity and temperature sensor
 * for the STM32F0 Discovery board.
 *
 * Author: Harris Shallcross
 * Year: 3/2/2015
 *
 *
 *Code and example descriptions can be found on my blog at:
 *www.hsel.co.uk
 *
 *The MIT License (MIT)
 *Copyright (c) 2015 Harris Shallcross
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

//Simple typedef enumeration of pin modes
typedef enum{
	INPUT = 0,
	OUTPUT = 1
} GPIO_PinMode;

//Pin definitions! Some of these are hardcoded
//in the setup... tut!
#define AM_PIN		GPIO_Pin_0
#define AM_GPIO		GPIOA
#define AM_TIM		TIM15

EXTI_InitTypeDef E;
GPIO_InitTypeDef G;
NVIC_InitTypeDef N;
TIM_TimeBaseInitTypeDef T;

volatile uint32_t MSec = 0;
void SysTick_Handler(void){
	MSec++;
}

volatile uint8_t Data = 0;

uint16_t DatArray[41] = {0};

//Where the sampling magic happens!
void EXTI0_1_IRQHandler(void){
	static uint8_t DCnt = 0;

	//Check for falling edge
	if(EXTI_GetITStatus(EXTI_Line0) == SET){
		EXTI_ClearITPendingBit(EXTI_Line0);

		//Get current time of last falling edge to
		//current falling edge and store in array
		DatArray[DCnt] = TIM_GetCounter(AM_TIM);

		//Increment the data counter
		DCnt++;

		//If enough bytes have been received, disable
		//the counter
		if(DCnt == 41){
			Data = 1;
			DCnt = 0;
			TIM_Cmd(AM_TIM, DISABLE);
		}

		//Reset the counter for next data
		TIM_SetCounter(AM_TIM, 0);
	}
}

//Function to set the bus pin to either an input or open
//drain output
void AM_Pin(GPIO_PinMode GP){
	if(GP == INPUT){
		GPIO_StructInit(&G);
		G.GPIO_Pin = GPIO_Pin_0;
		G.GPIO_Mode = GPIO_Mode_IN;
		GPIO_Init(GPIOA, &G);
	}
	else{
		GPIO_StructInit(&G);
		G.GPIO_Pin = GPIO_Pin_0;
		G.GPIO_Mode = GPIO_Mode_OUT;
		G.GPIO_OType = GPIO_OType_OD;
		G.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_Init(GPIOA, &G);
	}
}

//Convert an array to an unsigned byte/word/dword
//Type = 0 for u8
//Type = 1 for u16
//Type = 2 for u32
uint32_t AToU(uint8_t *D, uint8_t Type){
	uint32_t V = 0;
	uint8_t Cnt = 0;

	for(Cnt = 0; Cnt<(8<<Type); Cnt++){
		V |= ((D[(8<<Type)-Cnt]&1)<<Cnt);
	}

	return V;
}

int main(void)
{
	//Enable clocks!
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	//Configure systick...
	SysTick_Config(SystemCoreClock/1000);

	//Initialize the data pin as an input
	GPIO_StructInit(&G);
	G.GPIO_Pin = AM_PIN;
	G.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(AM_GPIO, &G);

	//Configure the data pin as an EXTI source, this shouldn't
	//be hardcoded!
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

	//Disable but initialize EXTI line
	E.EXTI_Line = EXTI_Line0;
	E.EXTI_LineCmd = DISABLE;
	E.EXTI_Mode = EXTI_Mode_Interrupt;
	E.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_Init(&E);

	//Enable EXTI interrupts
	N.NVIC_IRQChannel = EXTI0_1_IRQn;
	N.NVIC_IRQChannelCmd = ENABLE;
	N.NVIC_IRQChannelPriority = 0;
	NVIC_Init(&N);

	//Setup period measuring timer
	T.TIM_ClockDivision = TIM_CKD_DIV1;
	T.TIM_CounterMode = TIM_CounterMode_Up;
	T.TIM_Period = 0xFFFF;
	T.TIM_Prescaler = 0xF;
	TIM_TimeBaseInit(TIM15, &T);
	TIM_Cmd(TIM15, DISABLE);
	TIM_SetCounter(TIM15, 0);

	//DConv will store the threshold'd bits
	uint8_t DConv[41] = {0}, Cnt;

	//Clear all current values in both converted
	//and measured period arrays.
	memset(DConv, 0, 41*sizeof(uint8_t));
	memset(DatArray, 0, 41*sizeof(uint16_t));

	MSec = 0;
	//DHT22 "Stability" delay! Needs to be >1s
	while(MSec<1500);

	//Variables to store read and final
	//humidity, temperature, checksum
	//and datasum. DCor defines whether the
	//data was correct (as far as the checksum
	//can tell).
	uint16_t HumR, TempR;
	uint8_t CSum, DCor = 0, DSum;

	//Final temperature and humidity are stored
	//as floats.
	float Temp, Humidity;

	while(1)
	{
		//Start transmission
		Data = 0;
		E.EXTI_LineCmd = DISABLE;
		EXTI_Init(&E);
		AM_Pin(OUTPUT);
		TIM_SetCounter(AM_TIM, 0);
		TIM_Cmd(AM_TIM, ENABLE);

		GPIO_ResetBits(AM_GPIO, AM_PIN);

		//Pull low for 1ms minimum
		while(TIM_GetCounter(AM_TIM) < 3002);
		GPIO_SetBits(AM_GPIO, AM_PIN);
		TIM_Cmd(AM_TIM, DISABLE);
		TIM_SetCounter(AM_TIM, 0);

		AM_Pin(INPUT);
		//Wait for pin to go low
		while(GPIO_ReadInputDataBit(AM_GPIO, AM_PIN));

		//Wait for pin to go high
		while(!GPIO_ReadInputDataBit(AM_GPIO, AM_PIN));

		//Next falling edge will be start of transmission
		E.EXTI_LineCmd = ENABLE;
		EXTI_Init(&E);

		TIM_Cmd(AM_TIM, ENABLE);

		//Wait for data
		while(!Data);

		//Preclear the threshold bits array
		memset(DConv, 0, 41*sizeof(uint8_t));

		//Do the bit thresholding! Theoretically, a 1 should be
		//360 but for a bit of tolerance, I made it 340
		for(Cnt = 0; Cnt<41; Cnt++){
			if(DatArray[Cnt]<340) DConv[Cnt] = 0;
			else DConv[Cnt] = 1;
		}

		//Clear the period array for next time
		memset(DatArray, 0, 41*sizeof(uint16_t));

		//Convert the read humidity value from the threshold array
		//to an unsigned 16bit unsigned integer and convert to real
		//value, stored in a float.
		HumR = AToU(DConv, 1);
		Humidity = (float)HumR*0.1f;

		//Convert the read temperature from the threshold array to
		//a 16bit unsigned integer. Check to see if MSB is set, which
		//would indicate a negative temperature. If MSB is set, clear
		//MSB and assign a negative temperature to the real temperature
		//storing variable (also a float).

		TempR = AToU(&DConv[16], 1);
		if(TempR&(1<<15)){
			TempR &= (1<<16)-1;
			Temp = -(float)TempR*0.1f;
		}
		else{
			Temp = (float)TempR*0.1f;
		}

		//Convert checksum from threshold array to unsigned 8bit integer
		CSum = AToU(&DConv[32], 0);
		DCor = 0;

		//Sum all of the data, 8bit at a time
		DSum = 0;
		DSum += (uint8_t)(TempR>>8);
		DSum += (uint8_t)TempR&255;
		DSum += (uint8_t)(HumR>>8);
		DSum += (uint8_t)HumR&255;

		//Compare checksum and datasum. If they're different, data is
		//incorrect.
		if(CSum == DSum) DCor = 1;
		else DCor = 0;

		//Wait for atleast 2 seconds for another conversion!
		MSec = 0;
		SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
		while(MSec<2000);
		SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	}
}
