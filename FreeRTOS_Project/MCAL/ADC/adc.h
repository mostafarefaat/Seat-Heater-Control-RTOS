/*
 * adc.h
 *
 *  Created on: Jun 24, 2024
 *      Author: mosta
 */

#ifndef MCAL_ADC_ADC_H_
#define MCAL_ADC_ADC_H_

#include "tm4c123gh6pm_registers.h"

#define  ADC0SS3_PRIORITY_MASK          0xFFFF1FFF
#define  ADC0SS3__PRIORITY_BITS_POS     13
#define  ADC0SS3__INTERRUPT_PRIORITY    5

void ADCInit(void);



#endif /* MCAL_ADC_ADC_H_ */
