/*
 * RTC_i2c_driver.c
 *
 *  Created on: 6 Nis 2020
 *      Author: lenovo
 */

#include "RTC_i2c_driver.h"


void static delay_ms(uint16_t time) //generates approximately <time> ms delay
{
	for(uint32_t i=0; i< time*10000; i++);
}


static void ack_control(uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		I2C1->CR1 |= (1 << 10);
	else if(EnorDi == DISABLE)
		I2C1->CR1 &= ~(1 << 10);
}


static void ADDR_bitClear(void)
{
	uint32_t dummy_read;
	dummy_read = I2C1->SR1;
	dummy_read = I2C1->SR2;
	(void)dummy_read;
}

static void generate_start(void)
{
	I2C1->CR1 |= (1 << 8);
}

static void generate_stop(void)
{
	I2C1->CR1 |= (1 << 9);
}


/*****************************************************************
 * this function is to set dedicated GPIO pins for I2C1 interface
 * ***************************************************************
 * PB6-> I2C1_SCL
 * PB7-> I2C1_SDA

 * is configured as pins
 */
static void rtc_gpio_init(void)
{
	GPIOB_PCLK_EN();
	delay_ms(1);

	GPIOB->MODER |= (10 << 12); // alternate function selected for pins 6 and 7
	GPIOB->PUPDR |= (5 << 12);  // pull-up selected for pins 6 and 7
	GPIOB->OTYPER |= (3 << 6);  // open-drain selected for pins 6 and 7
	GPIOB->OSPEEDR |= (10 << 12); // fast speed selected for pins 6 and 7
	GPIOB->AFR[0] |=  (0x44 << 24); //AF4 selected for pins 6 and 7


}

static void rtc_i2c1_init(void)
{
	I2C1_PCLK_EN();
	delay_ms(1);

	I2C1->CR2 |= (16 << 0); //apb1_clk=16 MHz
	I2C1->CCR &= ~(1 << 15); // selected SM mode
	I2C1->CCR |= (0x50 << 0); // setting 100 KHz SCL freq.
	I2C1->TRISE = 0x17;		//rise time is 62,5 ns
	I2C1->CR1 |= (1 << 0);  // enable the i2c1





}

void rtc_init(void)
{
	rtc_gpio_init();
	rtc_i2c1_init();
}

void rtc_write(uint8_t *TxBuffer,uint32_t Lenght)
{

	generate_start();

	while(!(I2C1->SR1 & 0x01));		// wait till SB bit is set

//	while(!(I2C1->SR1 & 0b10000000)); // wait till TXE bit is set

	I2C1->DR = (uint8_t)0xD0; //RTC Address and write command

	delay_ms(1);
	ADDR_bitClear();


	while(Lenght>0)
	{

		while(!(I2C1->SR1 & 0b10000000));		// wait till TXE bit is set

		I2C1->DR = *TxBuffer;
		TxBuffer++;

		Lenght--;


	}

	while(!(I2C1->SR1 & 0b10000000));	// wait till TXE bit is set

	generate_stop();



}

uint8_t rtc_recieve(void)
{
	generate_start();

	while(!(I2C1->SR1 & 0x01));

	I2C1->DR = (uint8_t)0xD1;	//RTC Address and read command

	ack_control(DISABLE);

	delay_ms(1);
	ADDR_bitClear();

	while(!(I2C1->SR1 & 0b1000000));

	generate_stop();

	uint8_t red_data = I2C1->DR;

	delay_ms(1);

	return red_data;


}

//static uint8_t bcd_to_dec(uint8_t data)
//{
//	return ((data/16)*10)+(data%16);
//}

uint8_t rtc_read(uint8_t REG)
{
	if(REG==Second_REG)
		{
			rtc_write((uint8_t*)Set_SecondPointer,1);
		}
	if(REG==Munite_REG)
		{
			rtc_write((uint8_t*)Set_MunitePointer,1);
		}
	if(REG==Hour_REG)
		{
			rtc_write((uint8_t*)Set_HourPointer,1);
		}
	if(REG==Day_REG)
		{
			rtc_write((uint8_t*)Set_DayPointer,1);
		}
	if(REG==Date_REG)
		{
			rtc_write((uint8_t*)Set_DatePointer,1);
		}
	if(REG==Month_REG)
		{
			rtc_write((uint8_t*)Set_MonthPointer,1);
		}
	if(REG==Year_REG)
		{
			rtc_write((uint8_t*)Set_YearPointer,1);
		}

	delay_ms(1);

	generate_start();

	while(!(I2C1->SR1 & 0x01));

	I2C1->DR = (uint8_t)0xD1;	//RTC Address and read command

	ack_control(DISABLE);

	delay_ms(1);
	ADDR_bitClear();

	while(!(I2C1->SR1 & 0b1000000));

	generate_stop();

	uint8_t red_data = I2C1->DR;

	delay_ms(1);

	return bcd_to_dec(red_data);

}

//static uint8_t dec_to_bcd(uint8_t data)
//{
//	return ((data/10)*16)+(data%10);
//}

void rtc_set(uint8_t Second,uint8_t Munite,uint8_t Hour,uint8_t Day,uint8_t Date,uint8_t Month,uint8_t Year)
{
	char RTC_Data[]={0x04,Second,Munite,Hour,Day,Date,Month,Year};

	for(uint8_t i=0; i<8; i++)
		RTC_Data[i]=dec_to_bcd(RTC_Data[i]);

	rtc_write((uint8_t*)RTC_Data, 8);


}




