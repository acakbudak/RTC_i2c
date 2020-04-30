/*
 * RTC_i2c_driver.h
 *
 *  Created on: 6 Nis 2020
 *      Author: acakbudak
 */

#ifndef INC_RTC_I2C_DRIVER_H_
#define INC_RTC_I2C_DRIVER_H_

#include "STM32F446RE_main.h"


#define Set_SecondPointer 0x00
#define Set_MunitePointer 0x01
#define Set_HourPointer   0x02
#define Set_DayPointer    0x03
#define Set_DatePointer   0x04
#define Set_MonthPointer  0x05
#define Set_YearPointer   0x06

#define Second_REG		0
#define Munite_REG		1
#define Hour_REG		2
#define Day_REG			3
#define Date_REG		4
#define Month_REG		5
#define	Year_REG		6


void rtc_init(void);
void rtc_set(uint8_t Second,uint8_t Munite,uint8_t Hour,uint8_t Day,uint8_t Date,uint8_t Month,uint8_t Year);
uint8_t rtc_read(uint8_t REG);
void rtc_write(uint8_t *TxBuffer,uint32_t Lenght);
uint8_t rtc_recieve(void);


#endif /* INC_RTC_I2C_DRIVER_H_ */
