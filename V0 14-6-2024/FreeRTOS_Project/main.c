/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* MCAL includes. */
#include "gpio.h"

#define NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND 369

void Delay_MS(unsigned long long n)
{
    volatile unsigned long long count = 0;
    while(count++ < (NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND * n) );
}

/* The HW setup function */
static void prvSetupHardware( void );

/* FreeRTOS tasks */
void vPeriodicGreenLedTask(void *pvParameters);
void vPeriodicRedLedTask(void *pvParameters);

int main(void)
{
    /* Setup the hardware for use with the Tiva C board. */
    prvSetupHardware();

    /* Create Tasks here */
    xTaskCreate(vPeriodicGreenLedTask, "Task 1", 256, NULL, 1, NULL);
    xTaskCreate(vPeriodicRedLedTask, "Task 2", 256, NULL, 1, NULL);

    /* Now all the tasks have been started - start the scheduler.

    NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
    The processor MUST be in supervisor mode when vTaskStartScheduler is
    called.  The demo applications included in the FreeRTOS.org download switch
    to supervisor mode prior to main being called.  If you are not using one of
    these demo application projects then ensure Supervisor mode is used here. */
    vTaskStartScheduler();

    /* Should never reach here!  If you do then there was not enough heap
    available for the idle task to be created. */
    for (;;);

}


static void prvSetupHardware( void )
{
    /* Place here any needed HW initialization such as GPIO, UART, etc.  */
    GPIO_BuiltinButtonsLedsInit();
}

void vPeriodicGreenLedTask(void *pvParameters)
{
    for (;;)
    {
        Delay_MS(1000);
        GPIO_GreenLedToggle();
    }
}

void vPeriodicRedLedTask(void *pvParameters)
{
    for (;;)
    {
        GPIO_RedLedToggle();
        Delay_MS(1000);
    }
}
/*-----------------------------------------------------------*/
