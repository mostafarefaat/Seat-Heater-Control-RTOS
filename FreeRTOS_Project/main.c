/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"

/* MCAL includes. */
#include "gpio.h"
#include "uart0.h"

#include "tm4c123gh6pm_registers.h"

#define NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND 369

#define DRIVER_SEAT        1
#define PASSENGER_SEAT     2


#define HEAT_LEVEL_OFF      0
#define HEAT_LEVEL_LOW      1
#define HEAT_LEVEL_MED      2
#define HEAT_LEVEL_HIGH     3

#define HEAT_LEVEL_OFF_DESIRED_TEMP      0
#define HEAT_LEVEL_LOW_DESIRED_TEMP      25
#define HEAT_LEVEL_MED_DESIRED_TEMP      30
#define HEAT_LEVEL_HIGH_DESIRED_TEMP     35

#define HEAT_OUTPUT_OFF      0
#define HEAT_OUTPUT_LOW      1
#define HEAT_OUTPUT_MED      2
#define HEAT_OUTPUT_HIGH     3

int rand(void);

void Delay_MS(unsigned long long n)
{
    volatile unsigned long long count = 0;
    while(count++ < (NUMBER_OF_ITERATIONS_PER_ONE_MILI_SECOND * n) );
}

/* The HW setup function */
static void prvSetupHardware( void );

/* FreeRTOS tasks */
void vButtonTask(void *pvParameters);


void vHeaterTask(void *pvParameters);
void vHeaterHandlerTask(void *pvParameters);

void vTempTask(void *pvParameters);
void vUARTTask(void *pvParameters);

/*FreeRTOS Event Groups*/
EventGroupHandle_t xButtonsEventGroup;
EventGroupHandle_t xBuffersEventGroup;

/* Definitions for the event bits in the event group. */
#define mainSW2_INTERRUPT_BIT ( 1UL << 0UL )  /* Event bit 0, which is set by a SW2 Interrupt. */
#define mainSW1_INTERRUPT_BIT ( 1UL << 1UL )  /* Event bit 1, which is set by a SW1 Interrupt. */

#define QueueFromButtonToHeater_BIT         ( 1UL << 0UL )  /* Event bit 0, which is set by a xQueueFromButtonToHeater. */
#define QueueFromTemperatureToHeater_BIT    ( 1UL << 1UL )  /* Event bit 1, which is set by a xQueueFromTempToHeater. */


/*FreeRTOS Semaphores & Mutexes*/
SemaphoreHandle_t xUARTMutex;
SemaphoreHandle_t xDriverStructMutex;
SemaphoreHandle_t xPassengerStructMutex;

/*FreeRTOS Queue*/
QueueSetHandle_t xQueueFromButtonToHeater;
QueueSetHandle_t xQueueFromHeaterToHeaterHandler;
QueueSetHandle_t xQueueFromTempToHeater;

/*Seat Info Structure*/
typedef struct{
    uint8_t   seat;
    uint8_t   heat_level;
    uint8_t   desired_temp;
    uint8_t   current_temp;
    uint8_t   heater_output_signal;
}Input_Info;

typedef struct{
    uint8_t   *seat;
    uint8_t   *heat_level;
    uint8_t   desired_temp;
    uint8_t   current_temp;
    uint8_t   *heater_output_signal;
}UART_Info;


UART_Info Driver_UART_data = { "Driver","OFF", 0, 0, "HEAT_OUTPUT_OFF"};
UART_Info Passenger_UART_data = { "DrPassengeriver","OFF", 0, 0, "HEAT_OUTPUT_OFF"};;

int main(void)
{
    /* Setup the hardware for use with the Tiva C board. */
    prvSetupHardware();

    /*Creating Event Groups*/
    xButtonsEventGroup = xEventGroupCreate();
    xBuffersEventGroup = xEventGroupCreate();

    /*Creating Semaphores & Mutex*/
    xUARTMutex = xSemaphoreCreateMutex();
    xDriverStructMutex = xSemaphoreCreateMutex();
    xPassengerStructMutex = xSemaphoreCreateMutex();

    /*Creating Queue*/
    xQueueFromButtonToHeater =  xQueueCreate(5,sizeof(uint32_t));
    xQueueFromHeaterToHeaterHandler = xQueueCreate(5,sizeof(uint32_t));
    xQueueFromTempToHeater = xQueueCreate(5,sizeof(uint32_t));

    /* Create Tasks here */

    /*Responsible for capturing which button is pressed and what is the desired Heating Level (Off - Low - Medium - High)
     * and send this data to vHeaterMonitorTask via Message Queue*/
    xTaskCreate(vButtonTask, "Button Task", configMINIMAL_STACK_SIZE, NULL, 2, NULL);


    /*Responsible for Receiving the data (which button is pressed and what is the desired Heating Level) acquired from vButtonTask via Message Queue
     * and pass these data to vHeaterHandlerTask*/
    xTaskCreate(vHeaterTask, "Heater Monitor Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    /*Responsible for Receiving the data acquired from vHeaterMonitorTask via Message Queue
     * and pass these data to vHeaterHandlerTask*/
    xTaskCreate(vHeaterHandlerTask, "Heater Handler Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    xTaskCreate(vTempTask, "Temperature Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

    xTaskCreate(vUARTTask, "UART Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);


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
    GPIO_SW1EdgeTriggeredInterruptInit();
    GPIO_SW2EdgeTriggeredInterruptInit();
    UART0_Init();
}

void vButtonTask(void *pvParameters)
{
    static uint8_t xDriversButtonCounting = 0;
    static uint8_t xPassengerButtonCounting = 0;

    /*Create Seat_Info Struct*/
    Input_Info Driver_Seat_Info = {0,0,0,0};
    Input_Info Passenger_Seat_Info = {0,0,0,0};

    EventBits_t xEventGroupValue;
    const EventBits_t xBitsToWaitFor = (mainSW1_INTERRUPT_BIT | mainSW2_INTERRUPT_BIT); /*Waiting for SW1(Driver) or SW2(Passenger)*/

    for (;;)
    {
        /* Block to wait for event bits to become set within the event group. */
        xEventGroupValue = xEventGroupWaitBits( xButtonsEventGroup,    /* The event group to read. */
                                   xBitsToWaitFor,              /* Bits to test. */
                                   pdTRUE,                      /* Clear bits on exit if the unblock condition is met. */
                                   pdFALSE,                     /* Don't wait for all bits. */
                                   portMAX_DELAY);              /* Don't time out. */

        if( xSemaphoreTake(xUARTMutex,portMAX_DELAY)==pdTRUE ){
            //UART0_SendString("Button Task is Processing\r\n");

            if(  (xEventGroupValue & mainSW1_INTERRUPT_BIT) != 0  ){

                //UART0_SendString("Drivers Button is Pressed\r\n");
                xDriversButtonCounting = (++xDriversButtonCounting) % 4; /*From Off(0) -> Low(1) -> Med(2) -> High(3)*/

                if(xSemaphoreTake(xDriverStructMutex,portMAX_DELAY)==pdTRUE){

                    Driver_Seat_Info.seat = DRIVER_SEAT;
                    switch(xDriversButtonCounting){
                    case 0 : Driver_Seat_Info.heat_level = HEAT_LEVEL_OFF; break;
                    case 1 : Driver_Seat_Info.heat_level = HEAT_LEVEL_LOW; break;
                    case 2 : Driver_Seat_Info.heat_level = HEAT_LEVEL_MED; break;
                    case 3 : Driver_Seat_Info.heat_level = HEAT_LEVEL_HIGH; break;
                    default: break;
                    }
                    xSemaphoreGive(xDriverStructMutex);
                }
                xQueueSend(xQueueFromButtonToHeater,&Driver_Seat_Info,portMAX_DELAY);
                xEventGroupSetBits(xBuffersEventGroup, QueueFromButtonToHeater_BIT);
                //UART0_SendString("Data pushed\r\n");
                //GPIO_Leds_Off();
                //GPIO_RedLedOn();
            }
            else if( (xEventGroupValue & mainSW2_INTERRUPT_BIT) != 0 ){

                //UART0_SendString("Passenger Button is Pressed\r\n");
                xPassengerButtonCounting = (++xPassengerButtonCounting) % 4;  /*From Off(0) -> Low(1) -> Med(2) -> High(3)*/

                if(xSemaphoreTake(xPassengerStructMutex,portMAX_DELAY)==pdTRUE){

                    Passenger_Seat_Info.seat = PASSENGER_SEAT;
                    switch(xPassengerButtonCounting){
                    case 0 : Passenger_Seat_Info.heat_level = HEAT_LEVEL_OFF;break;
                    case 1 : Passenger_Seat_Info.heat_level = HEAT_LEVEL_LOW;break;
                    case 2 : Passenger_Seat_Info.heat_level = HEAT_LEVEL_MED;break;
                    case 3 : Passenger_Seat_Info.heat_level = HEAT_LEVEL_HIGH;break;
                    default: break;
                    }
                    xSemaphoreGive(xPassengerStructMutex);
                }
                xQueueSend(xQueueFromButtonToHeater,&Passenger_Seat_Info,portMAX_DELAY);
                xEventGroupSetBits(xBuffersEventGroup, QueueFromButtonToHeater_BIT);
                //UART0_SendString("Data pushed\r\n");
                //GPIO_Leds_Off();
                //GPIO_BlueLedOn();

            }

            xSemaphoreGive(xUARTMutex);
        }

    }
}

/*------------------------------------------------------------------*/
void vHeaterTask(void *pvParameters)
{
    Input_Info received_data = {0,0,0,0};
    uint8_t   current_received_temperature = 0;

    EventBits_t xEventGroupValue;
    const EventBits_t xBitsToWaitFor = (QueueFromButtonToHeater_BIT | QueueFromTemperatureToHeater_BIT); /*Waiting for SW1(Driver) or SW2(Passenger)*/

    for (;;)
    {

        /* Block to wait for event bits to become set within the event group. */
        xEventGroupValue = xEventGroupWaitBits( xBuffersEventGroup,    /* The event group to read. */
                                   xBitsToWaitFor,              /* Bits to test. */
                                   pdTRUE,                      /* Clear bits on exit if the unblock condition is met. */
                                   pdFALSE,                     /* Don't wait for all bits. */
                                   portMAX_DELAY);              /* Don't time out. */

        if(  (xEventGroupValue & QueueFromButtonToHeater_BIT) != 0  ){

            xQueueReceive(xQueueFromButtonToHeater, &received_data, portMAX_DELAY);

        }
        else if( (xEventGroupValue & QueueFromTemperatureToHeater_BIT) != 0 ){
            xQueueReceive(xQueueFromTempToHeater, &current_received_temperature, portMAX_DELAY);
        }
            received_data.current_temp = current_received_temperature;

            switch(received_data.heat_level){
            case HEAT_LEVEL_OFF :  received_data.desired_temp = HEAT_LEVEL_OFF_DESIRED_TEMP;   break;
            case HEAT_LEVEL_LOW :  received_data.desired_temp = HEAT_LEVEL_LOW_DESIRED_TEMP;   break;
            case HEAT_LEVEL_MED :  received_data.desired_temp = HEAT_LEVEL_MED_DESIRED_TEMP;   break;
            case HEAT_LEVEL_HIGH : received_data.desired_temp = HEAT_LEVEL_HIGH_DESIRED_TEMP; break;
            default: break;
            }

            xQueueSend(xQueueFromHeaterToHeaterHandler,&received_data,portMAX_DELAY);

    }
}
/*------------------------------------------------------------------*/

void vHeaterHandlerTask(void *pvParameters)
{
    Input_Info received_data = {0,0,0,0};
    uint8_t     desired_received_temp;
    uint8_t     current_received_temp;
    sint8       delta_temp = 0;

    for (;;)
    {
        if ( xQueueReceive(xQueueFromHeaterToHeaterHandler, &received_data, portMAX_DELAY) == pdTRUE ) {

            current_received_temp = received_data.current_temp;
            desired_received_temp = received_data.desired_temp;

            delta_temp = current_received_temp - desired_received_temp ;

            if( delta_temp <= (-10) ){
                received_data.heater_output_signal = HEAT_OUTPUT_HIGH;
                GPIO_Leds_Off();
                GPIO_GreenLedOn();
                GPIO_BlueLedOn();
            }
            else if( (delta_temp <= (-5)) && (delta_temp > (-10))  ){
                received_data.heater_output_signal = HEAT_OUTPUT_MED;
                GPIO_Leds_Off();
                GPIO_BlueLedOn();
            }
            else if( (delta_temp <= (-2)) && (delta_temp > (-5))  ){
                received_data.heater_output_signal = HEAT_OUTPUT_LOW;
                GPIO_Leds_Off();
                GPIO_GreenLedOn();
            }
            else if( delta_temp > (-2) ){
                received_data.heater_output_signal = HEAT_OUTPUT_OFF;
                GPIO_Leds_Off();
            }
            else{

            }



            if(received_data.seat == DRIVER_SEAT){

                Driver_UART_data.seat = "DRIVER_SEAT";
                Driver_UART_data.current_temp = received_data.current_temp;
                Driver_UART_data.desired_temp= received_data.desired_temp;

                (received_data.heat_level == HEAT_LEVEL_OFF)  ? (Driver_UART_data.heat_level="OFF") :
                (received_data.heat_level == HEAT_LEVEL_LOW)  ? (Driver_UART_data.heat_level="LOW"):
                (received_data.heat_level == HEAT_LEVEL_MED)  ? (Driver_UART_data.heat_level="MED"):
                (received_data.heat_level == HEAT_LEVEL_HIGH) ? (Driver_UART_data.heat_level="HIGH"):(Driver_UART_data.heat_level="INVALID");

                (received_data.heater_output_signal == HEAT_OUTPUT_OFF)  ? (Driver_UART_data.heater_output_signal="HEAT_OUTPUT_OFF") :
                (received_data.heater_output_signal == HEAT_OUTPUT_LOW)  ? (Driver_UART_data.heater_output_signal="HEAT_OUTPUT_LOW") :
                (received_data.heater_output_signal == HEAT_OUTPUT_MED)  ? (Driver_UART_data.heater_output_signal="HEAT_OUTPUT_MED") :
                (received_data.heater_output_signal == HEAT_OUTPUT_HIGH)  ? (Driver_UART_data.heater_output_signal="HEAT_OUTPUT_HIGH") :(Driver_UART_data.heater_output_signal="INVALID");
            }
            else if(received_data.seat == PASSENGER_SEAT){

                Passenger_UART_data.seat = "PASSENGER_SEAT";
                Passenger_UART_data.current_temp = received_data.current_temp;
                Passenger_UART_data.desired_temp= received_data.desired_temp;

                (received_data.heat_level == HEAT_LEVEL_OFF)  ? (Passenger_UART_data.heat_level="OFF") :
                (received_data.heat_level == HEAT_LEVEL_LOW)  ? (Passenger_UART_data.heat_level="LOW"):
                (received_data.heat_level == HEAT_LEVEL_MED)  ? (Passenger_UART_data.heat_level="MED"):
                (received_data.heat_level == HEAT_LEVEL_HIGH) ? (Passenger_UART_data.heat_level="HIGH"):(Passenger_UART_data.heat_level="INVALID");

                (received_data.heater_output_signal == HEAT_OUTPUT_OFF)  ? (Passenger_UART_data.heater_output_signal="HEAT_OUTPUT_OFF") :
                (received_data.heater_output_signal == HEAT_OUTPUT_LOW)  ? (Passenger_UART_data.heater_output_signal="HEAT_OUTPUT_LOW") :
                (received_data.heater_output_signal == HEAT_OUTPUT_MED)  ? (Passenger_UART_data.heater_output_signal="HEAT_OUTPUT_MED") :
                (received_data.heater_output_signal == HEAT_OUTPUT_HIGH)  ? (Passenger_UART_data.heater_output_signal="HEAT_OUTPUT_HIGH") :(Passenger_UART_data.heater_output_signal="INVALID");
            }
            else{

            }

        }
    }
}

/*------------------------------------------------------------------*/
void vTempTask(void *pvParameters)

{
    uint8_t current_received_temperature = 0;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(2000));
        current_received_temperature = (rand()%35)+5;                   /*Random Number From 5 to 40*/ /*TODO: input from Potentiometer*/
        xQueueSend(xQueueFromTempToHeater,&current_received_temperature,portMAX_DELAY);
        xEventGroupSetBits(xBuffersEventGroup, QueueFromTemperatureToHeater_BIT);

    }
}
/*------------------------------------------------------------------*/

void vUARTTask(void *pvParameters)
{

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if(xSemaphoreTake(xUARTMutex,portMAX_DELAY)==pdTRUE){

            UART0_SendString("Driver Seat: \r\n");
            UART0_SendString("Current Temperature is: ");
            UART0_SendInteger(Driver_UART_data.current_temp);
            UART0_SendString(" and Desired Temperature is: ");
            UART0_SendInteger(Driver_UART_data.desired_temp);
            UART0_SendString(" and Received Heat Level: ");
            UART0_SendString(Driver_UART_data.heat_level);
            UART0_SendString(" and Output Signal is: ");
            UART0_SendString(Driver_UART_data.heater_output_signal);
            UART0_SendString("\r\n");
            UART0_SendString("Passenger Seat: \r\n");
            UART0_SendString("Current Temperature is: ");
            UART0_SendInteger(Passenger_UART_data.current_temp);
            UART0_SendString(" and Desired Temperature is: ");
            UART0_SendInteger(Passenger_UART_data.desired_temp);
            UART0_SendString(" and Received Heat Level: ");
            UART0_SendString(Passenger_UART_data.heat_level);
            UART0_SendString(" and Output Signal is: ");
            UART0_SendString(Passenger_UART_data.heater_output_signal);
            UART0_SendString("\r\n");

            xSemaphoreGive(xUARTMutex);

        }

    }
}
/*------------------------------------------------------------------*/
void GPIOPortF_Handler(void){
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    //UART0_SendString("ISR is Processing\r\n");

    if(GPIO_PORTF_RIS_REG & (1<<0)){       /*SW2*/
        xEventGroupSetBitsFromISR(xButtonsEventGroup, mainSW2_INTERRUPT_BIT, &pxHigherPriorityTaskWoken);
        GPIO_PORTF_ICR_REG   |= (1<<0);       /* Clear Trigger flag for PF0 (Interrupt Flag) */

    }
    else if(GPIO_PORTF_RIS_REG & (1<<4)){   /*SW1*/
        xEventGroupSetBitsFromISR(xButtonsEventGroup, mainSW1_INTERRUPT_BIT, &pxHigherPriorityTaskWoken);
        GPIO_PORTF_ICR_REG   |= (1<<4);       /* Clear Trigger flag for PF4 (Interrupt Flag) */
    }

}
/*-----------------------------------------------------------*/
/* Idle Hook API */
void vApplicationIdleHook(void)
{
    if(xSemaphoreTake(xUARTMutex,portMAX_DELAY)==pdTRUE){
        UART0_SendString("Idle Task is Processing\r\n");
        UART0_SendString("\r\n");
        xSemaphoreGive(xUARTMutex);
    }
}
/*-----------------------------------------------------------*/
