 /******************************************************************************
 *
 * Module: GPTM
 *
 * File Name: GPTM.c
 *
 * Description: Source file for the TM4C123GH6PM GPTM driver for TivaC general purpose timer
 *
 * Author: Edges for Training Team
 *
 *******************************************************************************/
#include "GPTM.h"
#include "tm4c123gh6pm_registers.h"

void GPTM_WTimer0Init(void)
{
    /* Configure one shot down 32bit timer with tick time = 0.1msec */
    SYSCTL_RCGCWTIMER_REG |= (1<<0);  /* Enable clock WTimer0 in run mode */
    WTIMER0_CTL_REG = 0;              /* Disable WTimer0 output */
    WTIMER0_CFG_REG = 0x04;           /* Select 32-bit configuration option */
    WTIMER0_TAMR_REG = 0x01;          /* Select one-shot down counter mode of WTimer0A */
    WTIMER0_TAPR_REG = 1600 -1;       /* Set the prescaler for WTimer0A */
    WTIMER0_CTL_REG |= (0x01);        /* Enable WTimer0A module */
}

uint32 GPTM_WTimer0Read(void)
{
    return (uint32) (0xFFFFFFFFUL - WTIMER0_TAR_REG);
}

