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
#include "adc.h"


void ADCInit(void){

    /* Enable Clock to ADC0 and GPIO PortE pins*/
    SYSCTL_RCGCGPIO_REG |= (1<<4);   /* Enable Clock to GPIOE or PE3/AN0 */
    SYSCTL_RCGCADC_REG |= (1<<0);    /* AD0 clock enable*/

    /* initialize PE3 and PE2 for AIN0 and AIN1 input  */
    GPIO_PORTE_AFSEL_REG |= (1<<3) | (1<<2);         /* enable alternate function */
    GPIO_PORTE_DEN_REG &= ~(1<<3);                  /* disable digital function */
    GPIO_PORTE_DEN_REG &= ~(1<<2);                  /* disable digital function */
    GPIO_PORTE_AMSEL_REG |= (1<<3)| (1<<2);;       /* enable analog function */

    /* initialize sample sequencer3 */
    ADC0_ACTSS_REG &= ~(1<<3);              /* disable SS3 during configuration */
    ADC0_ACTSS_REG &= ~(1<<2);              /* disable SS2 during configuration */
    ADC0_EMUX_REG &= ~0xF000;              /* software trigger conversion for SS3 */
    ADC0_EMUX_REG &= ~0x0F00;              /* software trigger conversion for SS2 */
    ADC0_SSMUX3_REG = 0;                  /* get input from channel 0 */
    ADC0_SSMUX2_REG = 1;                  /* get input from channel 0 */
    ADC0_SSCTL3_REG |= (1<<1)|(1<<2);    /* take one sample at a time, set flag at 1st sample */
    ADC0_SSCTL2_REG |= (1<<1)|(1<<2);    /* take one sample at a time, set flag at 1st sample */

    /* Enable ADC Interrupt */
    ADC0_IM_REG |= (1<<3);                  /* Unmask ADC0 sequence 3 interrupt*/
    ADC0_IM_REG |= (1<<2);                  /* Unmask ADC0 sequence 2 interrupt*/

    NVIC_EN0_REG |= 0x00020000;             /* enable IRQ17 for ADC0SS3*/
    NVIC_EN0_REG |= 0x00010000;             /* enable IRQ16 for ADC0SS2*/

    ADC0_ACTSS_REG |= (1<<3);               /* enable ADC0 sequencer 3 */
    ADC0_ACTSS_REG |= (1<<2);               /* enable ADC0 sequencer 2 */

    ADC0_PSSI_REG |= (1<<3);                /* Enable SS3 conversion or start sampling data from AN0 */
    ADC0_PSSI_REG |= (1<<2);                /* Enable SS2 conversion or start sampling data from AN0 */
}

