/*
 * stm32f4xx.c
 *
 *  Created on: Aug 27, 2020
 *      Author: Imtiaz Ahmed
 */

#include <stm32f4xx.h>

void delay(uint32_t del)
{
	for (int i = 0; i < (del * 100000); i++);
}
