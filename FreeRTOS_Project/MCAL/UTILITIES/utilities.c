/**********************************************************************************************
 *
 * Module: GPIO
 *
 * File Name: GPIO.c
 *
 * Description: Source file for the TM4C123GH6PM DIO driver for TivaC Built-in Buttons and LEDs
 *
 * Author: Edges for Training Team
 *
 ***********************************************************************************************/
#include "utilities.h"



uint8 Read_Pin(volatile uint32 *PORT, uint8 PIN)
{
    if(PORT != NULL_PTR){
        return (uint8)READ_BIT(*PORT,PIN);
    }
    return ERROR;
}
uint8 Read_Port(volatile uint32 *PORT)
{
    if(PORT != NULL_PTR){
        return (uint8)(*PORT);
    }
    return ERROR;
}
uint8 Write_Pin(volatile uint32 *PORT, uint8 PIN, uint8 LEVEL)
{
    if(PORT != NULL_PTR){
    switch(LEVEL)
    {
        case LOGIC_HIGH:
            SET_BIT(*PORT,PIN);
        break;
        case LOGIC_LOW:
            CLEAR_BIT(*PORT,PIN);
        break;
        default:
        break;
    }
    return OK;
}
    else{
        return ERROR;
    }
}
uint8 Write_Port(volatile uint32 *PORT, uint32 VALUE_HEX)
{
    if(PORT != NULL_PTR){
        *PORT = VALUE_HEX;
        return OK;
    }
    else{
        return ERROR;
    }
}
uint8 Toggle_Pin(volatile uint32 *PORT, uint8 PIN)
{
    if(PORT != NULL_PTR){
    TOGGLE_BIT(*PORT,PIN);
    return OK;
    }
    else{
        return ERROR;
    }
}


