/*
 * main.c
 *
 *  Created on: 6 Nis 2020
 *      Author: lenovo
 */

#include "STM32F446RE_main.h"
#include <string.h>

char SecondSet[]={0x00,0x40};
char MuniteSet[]={0x01,0x05};
char HourSet[]={0x02,0x12};

void delay(void)
{
	for(uint32_t i=0; i<25000; i++);
}
int main(void)
{
	GPIOA_PCLK_EN();
	GPIOA->MODER |= (1<<10);
	GPIOA->OSPEEDR |= (1<<10);
	GPIOA->PUPDR &= ~(0x03<<10);

	rtc_init();

	while(1)
	{
	rtc_write((uint8_t *)SecondSet,2);
	rtc_write((uint8_t *)Set_SecondPointer,1);
	uint8_t ret_data=rtc_recieve();
	rtc_write((uint8_t *)MuniteSet,2);
	rtc_write((uint8_t *)Set_MunitePointer,1);
	ret_data=rtc_recieve();
	rtc_write((uint8_t *)HourSet,2);
	rtc_write((uint8_t *)Set_HourPointer,1);
	ret_data=rtc_recieve();
	}

//	rtc_set(20,11,7,4,25,8,9);

//	while(1)
//	{
//		uint8_t ret_data = rtc_read(Month_REG);
//		delay();
//		ret_data = rtc_read(Second_REG);
//		delay();
//		ret_data = rtc_read(Munite_REG);
//		delay();
//		ret_data = rtc_read(Year_REG);
//		delay();
//		ret_data = rtc_read(Day_REG);
//		delay();
//		ret_data = rtc_read(Date_REG);
//		delay();
////		if(ret_data>2)
////			GPIOA->ODR |= (1<<5);
//	}

}
