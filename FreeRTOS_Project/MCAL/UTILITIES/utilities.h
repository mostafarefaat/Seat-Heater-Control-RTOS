 /******************************************************************************
 *
 * Module: Common - Platform Types Abstraction
 *
 * File Name: std_types.h
 *
 * Description: types for ARM Cortex M4F
 *
 * Author: Edges Team
 *
 *******************************************************************************/

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include "tm4c123gh6pm_registers.h"
#include "std_types.h"

/************************************
 * GPIO I/O PINS
 ************************************/
#define PIN0 0
#define PIN1 1
#define PIN2 2
#define PIN3 3
#define PIN4 4
#define PIN5 5
#define PIN6 6
#define PIN7 7
#define PIN9 8
#define PIN8 9
#define PIN10 10
#define PIN11 11
#define PIN12 12
#define PIN13 13
#define PIN14 14
#define PIN15 15
#define PIN16 16
#define PIN17 17
#define PIN18 18
#define PIN19 19
#define PIN20 20
#define PIN21 21
#define PIN22 22
#define PIN23 23
#define PIN24 24
#define PIN25 25
#define PIN26 26
#define PIN27 27
#define PIN28 28
#define PIN29 29
#define PIN30 30
#define PIN31 31


/************************************
 * BIT MANIPULATIONS
 ************************************/
#define SET_BIT(REGISTER,PIN)    ( REGISTER |= (1<<PIN))

#define CLEAR_BIT(REGISTER,PIN)  ( REGISTER &= ~(1U<<PIN))

#define READ_BIT(REGISTER,PIN)   ((REGISTER & (1<<PIN))>>PIN)

#define TOGGLE_BIT(REGISTER,PIN) ( REGISTER ^= (1<<PIN))
/************************************
 * Functions Declarations
 ************************************/
uint8 Read_Pin(volatile uint32 *PORT, uint8 PIN);
uint8 Read_Port(volatile uint32 *PORT);
uint8 Write_Pin(volatile uint32 *PORT, uint8 PIN, uint8 LEVEL);
uint8 Write_Port(volatile uint32 *PORT, uint32 VALUE_HEX);
uint8 Toggle_Pin(volatile uint32 *PORT, uint8 PIN);

#endif /* UTILITIES_H_ */
