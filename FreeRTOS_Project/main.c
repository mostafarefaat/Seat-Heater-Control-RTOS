/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"

/* MCAL includes. */
#include "gpio.h"
#include "uart0.h"
#include "GPTM.h"

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

void vDriverHeaterTask(void *pvParameters);
void vPassengerHeaterTask(void *pvParameters);

void vDriverHeaterHandlerTask(void *pvParameters);
void vPassengerHeaterHandlerTask(void *pvParameters);

void vDriverTempTask(void *pvParameters);
void vPassengerTempTask(void *pvParameters);

void vScreenTask(void *pvParameters);

void vRunTimeMeasurementsTask(void *pvParameters);

uint32 ullTasksOutTime[10];
uint32 ullTasksInTime[10];
uint32 ullTasksTotalTime[10];
uint32 ullTasksExecutionTime[10];

/* Used to hold the handle of tasks */
TaskHandle_t xButtonTaskTaskHandle;
TaskHandle_t xDriverHeaterTaskHandle;
TaskHandle_t xDriverHeaterHandlerTaskHandle;
TaskHandle_t xPassengerHeaterTaskHandle;
TaskHandle_t xPassengerHeaterHandlerTaskHandle;
TaskHandle_t xDriverTempTaskHandle;
TaskHandle_t xPassengerTempTaskHandle;
TaskHandle_t xScreenTaskHandle;
TaskHandle_t xRunTimeAnalysisTaskHandle;


/*FreeRTOS Event Groups*/
EventGroupHandle_t xButtonsEventGroup;

/* Definitions for the event bits in the event group. */
#define mainSW2_INTERRUPT_BIT ( 1UL << 0UL )  /* Event bit 0, which is set by a SW2 Interrupt. */
#define mainSW1_INTERRUPT_BIT ( 1UL << 1UL )  /* Event bit 1, which is set by a SW1 Interrupt. */

/*FreeRTOS Queues*/
#define QueueFromTemperatureToHeater_BIT    ( 1UL << 1UL )  /* Event bit 1, which is set by a xQueueFromTempToHeater. */

/*FreeRTOS Semaphores & Mutexes*/
SemaphoreHandle_t xDriverBinarySemaphore;
SemaphoreHandle_t xPassengerBinarySemaphore;
SemaphoreHandle_t xUARTMutex;
SemaphoreHandle_t xDriverStructMutex;
SemaphoreHandle_t xPassengerStructMutex;

/*FreeRTOS Queue*/
QueueSetHandle_t xQueueDriverTemperature;
QueueSetHandle_t xQueuePassengerTemperature;

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


/*Create Seat_Info Struct*/
Input_Info Driver_Seat_Info    = {0,0,0,0};
Input_Info Passenger_Seat_Info = {0,0,0,0};

UART_Info UART_Driver_Seat_Info    = {"NULL","NULL",0,0,"NULL"};
UART_Info UART_Passenger_Seat_Info = {"NULL","NULL",0,0,"NULL"};


int main(void)
{
    /* Setup the hardware for use with the Tiva C board. */
    prvSetupHardware();

    /*Creating Event Groups*/
    xButtonsEventGroup = xEventGroupCreate();


    /*Creating Semaphores & Mutex*/
    xDriverBinarySemaphore = xSemaphoreCreateBinary();
    xPassengerBinarySemaphore = xSemaphoreCreateBinary();
    xUARTMutex = xSemaphoreCreateMutex();
    xDriverStructMutex = xSemaphoreCreateMutex();
    xPassengerStructMutex = xSemaphoreCreateMutex();

    /*Creating Queue*/
    xQueueDriverTemperature    =    xQueueCreate(5,sizeof(uint32_t));
    xQueuePassengerTemperature =    xQueueCreate(5,sizeof(uint32_t));

    /* Create Tasks here */

    /*Responsible for capturing which button is pressed and what is the desired Heating Level (Off - Low - Medium - High)
     * and send this data to vHeaterMonitorTask via Message Queue*/
    xTaskCreate(vButtonTask, "Button Task", configMINIMAL_STACK_SIZE, NULL, 3, &xButtonTaskTaskHandle);


    /*Responsible for Receiving the data (which button is pressed and what is the desired Heating Level) acquired from vButtonTask via Message Queue
     * and pass these data to vHeaterHandlerTask*/
    xTaskCreate(vDriverHeaterTask, "Driver Heater", configMINIMAL_STACK_SIZE, NULL, 2, &xDriverHeaterTaskHandle);
    xTaskCreate(vPassengerHeaterTask, "Passenger Heater", configMINIMAL_STACK_SIZE, NULL, 2, &xPassengerHeaterTaskHandle);

    /*Responsible for Receiving the data acquired from vHeaterMonitorTask via Message Queue
     * and pass these data to vHeaterHandlerTask*/
    xTaskCreate(vDriverHeaterHandlerTask, "Driver Handler", configMINIMAL_STACK_SIZE, NULL, 2, &xDriverHeaterHandlerTaskHandle);
    xTaskCreate(vPassengerHeaterHandlerTask, "Passenger Handler", configMINIMAL_STACK_SIZE, NULL, 2, &xPassengerHeaterHandlerTaskHandle);

    xTaskCreate(vDriverTempTask, "Driver Temp ", configMINIMAL_STACK_SIZE, NULL, 2, &xDriverTempTaskHandle);
    xTaskCreate(vPassengerTempTask, "Passenger Temperature ", configMINIMAL_STACK_SIZE, NULL, 2, &xPassengerTempTaskHandle);

    xTaskCreate(vScreenTask, "Screen Task", configMINIMAL_STACK_SIZE, NULL, 2, &xScreenTaskHandle);

    xTaskCreate(vRunTimeMeasurementsTask, "Run-time Measurements", 256, NULL, 1, &xRunTimeAnalysisTaskHandle);


    vTaskSetApplicationTaskTag( xButtonTaskTaskHandle, ( TaskHookFunction_t ) 1 );
    vTaskSetApplicationTaskTag( xDriverHeaterTaskHandle, ( TaskHookFunction_t ) 2 );
    vTaskSetApplicationTaskTag( xPassengerHeaterTaskHandle, ( TaskHookFunction_t ) 3 );
    vTaskSetApplicationTaskTag( xDriverHeaterHandlerTaskHandle, ( TaskHookFunction_t ) 4 );
    vTaskSetApplicationTaskTag( xPassengerHeaterHandlerTaskHandle, ( TaskHookFunction_t ) 5);
    vTaskSetApplicationTaskTag( xDriverTempTaskHandle, ( TaskHookFunction_t ) 6 );
    vTaskSetApplicationTaskTag( xPassengerTempTaskHandle, ( TaskHookFunction_t ) 7 );
    vTaskSetApplicationTaskTag( xScreenTaskHandle, ( TaskHookFunction_t ) 8 );
    vTaskSetApplicationTaskTag( xRunTimeAnalysisTaskHandle, ( TaskHookFunction_t ) 9 );

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
    GPIO_PORTA_LedsInit();
    GPIO_SW1EdgeTriggeredInterruptInit();
    GPIO_SW2EdgeTriggeredInterruptInit();
    GPTM_WTimer0Init();
    UART0_Init();
}
/*------------------------------------------------------------------*/
void vButtonTask(void *pvParameters)
{
    static uint8_t xDriversButtonCounting = 0;
    static uint8_t xPassengerButtonCounting = 0;

    EventBits_t xEventGroupValue;
    const EventBits_t xBitsToWaitFor = (mainSW1_INTERRUPT_BIT | mainSW2_INTERRUPT_BIT); /*Waiting for SW1(Driver) or SW2(Passenger)*/

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        /* Block to wait for event bits to become set within the event group. */
        xEventGroupValue = xEventGroupWaitBits( xButtonsEventGroup,    /* The event group to read. */
                                   xBitsToWaitFor,              /* Bits to test. */
                                   pdTRUE,                      /* Clear bits on exit if the unblock condition is met. */
                                   pdFALSE,                     /* Don't wait for all bits. */
                                   portMAX_DELAY);              /* Don't time out. */

            if(  (xEventGroupValue & mainSW1_INTERRUPT_BIT) != 0  ){

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

            }
            else if( (xEventGroupValue & mainSW2_INTERRUPT_BIT) != 0 ){

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
            }
    }
}
/*------------------------------------------------------------------*/
void vDriverHeaterTask(void *pvParameters)
{
    uint8_t   current_received_temperature = 0;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if( xQueueReceive(xQueueDriverTemperature, &current_received_temperature, portMAX_DELAY) == pdTRUE ){

            if(xSemaphoreTake(xDriverStructMutex,portMAX_DELAY)==pdTRUE){

                Driver_Seat_Info.current_temp = current_received_temperature;

                switch(Driver_Seat_Info.heat_level){
                case HEAT_LEVEL_OFF :  Driver_Seat_Info.desired_temp = HEAT_LEVEL_OFF_DESIRED_TEMP;   break;
                case HEAT_LEVEL_LOW :  Driver_Seat_Info.desired_temp = HEAT_LEVEL_LOW_DESIRED_TEMP;   break;
                case HEAT_LEVEL_MED :  Driver_Seat_Info.desired_temp = HEAT_LEVEL_MED_DESIRED_TEMP;   break;
                case HEAT_LEVEL_HIGH : Driver_Seat_Info.desired_temp = HEAT_LEVEL_HIGH_DESIRED_TEMP; break;
                default: break;
                }
                xSemaphoreGive(xDriverStructMutex);
            }
            if( (Driver_Seat_Info.current_temp > 40) || (Driver_Seat_Info.current_temp <5) ){
                Driver_Seat_Info.heater_output_signal = HEAT_OUTPUT_OFF;
                vTaskSuspend(xDriverHeaterHandlerTaskHandle);
                if(xSemaphoreTake(xUARTMutex,portMAX_DELAY)==pdTRUE){
                    UART0_SendString("ERROR In Driver Temperature Reading!! \r\n");
                }
                GPIO_PORTF_Leds_Off();
                GPIO_RedLedOn();
                xSemaphoreGive(xUARTMutex);
            }
            else{
                vTaskResume(xDriverHeaterHandlerTaskHandle);
            }
            xSemaphoreGive(xDriverBinarySemaphore);
        }

    }
}
/*------------------------------------------------------------------*/
void vPassengerHeaterTask(void *pvParameters)
{
    uint8_t   current_received_temperature = 0;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if( xQueueReceive(xQueuePassengerTemperature, &current_received_temperature, portMAX_DELAY) == pdTRUE ){

            if(xSemaphoreTake(xPassengerStructMutex,portMAX_DELAY)==pdTRUE){

                Passenger_Seat_Info.current_temp = current_received_temperature;

                switch(Passenger_Seat_Info.heat_level){
                case HEAT_LEVEL_OFF :  Passenger_Seat_Info.desired_temp = HEAT_LEVEL_OFF_DESIRED_TEMP;   break;
                case HEAT_LEVEL_LOW :  Passenger_Seat_Info.desired_temp = HEAT_LEVEL_LOW_DESIRED_TEMP;   break;
                case HEAT_LEVEL_MED :  Passenger_Seat_Info.desired_temp = HEAT_LEVEL_MED_DESIRED_TEMP;   break;
                case HEAT_LEVEL_HIGH : Passenger_Seat_Info.desired_temp = HEAT_LEVEL_HIGH_DESIRED_TEMP; break;
                default: break;
                }
                xSemaphoreGive(xPassengerStructMutex);
            }
            if( (Passenger_Seat_Info.current_temp > 40) || (Passenger_Seat_Info.current_temp <5) ){
                Passenger_Seat_Info.heater_output_signal = HEAT_OUTPUT_OFF;
                vTaskSuspend(xPassengerHeaterHandlerTaskHandle);
                if(xSemaphoreTake(xUARTMutex,portMAX_DELAY)==pdTRUE){
                    UART0_SendString("ERROR In Passenger Temperature Reading!! \r\n");
                }
                GPIO_PORTA_Leds_Off();
                GPIO_PORTA_RedLedOn();
                xSemaphoreGive(xUARTMutex);
            }
            else{
                vTaskResume(xPassengerHeaterHandlerTaskHandle);
            }
            xSemaphoreGive(xPassengerBinarySemaphore);
        }
    }
}
/*------------------------------------------------------------------*/
void vDriverHeaterHandlerTask(void *pvParameters)
{
    uint8_t     desired_received_temp;
    uint8_t     current_received_temp;
    sint8       delta_temp = 0;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (xSemaphoreTake(xDriverBinarySemaphore,portMAX_DELAY)==pdTRUE ){

            if( xSemaphoreTake(xDriverStructMutex,portMAX_DELAY)==pdTRUE ){

                current_received_temp = Driver_Seat_Info.current_temp;
                desired_received_temp = Driver_Seat_Info.desired_temp;
                delta_temp = current_received_temp - desired_received_temp ;

                if( delta_temp <= (-10) ){
                    Driver_Seat_Info.heater_output_signal = HEAT_OUTPUT_HIGH;  /*Cyan*/
                    GPIO_PORTF_Leds_Off();
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOn();
                    Delay_MS(500);

                }
                else if( (delta_temp <= (-5)) && (delta_temp > (-10))  ){
                    Driver_Seat_Info.heater_output_signal = HEAT_OUTPUT_MED; /*Blue*/
                    GPIO_PORTF_Leds_Off();
                    GPIO_BlueLedOn();
                    Delay_MS(500);

                }
                else if( (delta_temp <= (-2)) && (delta_temp > (-5))  ){
                    Driver_Seat_Info.heater_output_signal = HEAT_OUTPUT_LOW; /*Green*/
                    GPIO_PORTF_Leds_Off();
                    GPIO_GreenLedOn();
                    Delay_MS(500);

                }
                else if( delta_temp > (-2) ){
                    Driver_Seat_Info.heater_output_signal = HEAT_OUTPUT_OFF;
                    GPIO_PORTF_Leds_Off();
                    Delay_MS(500);

                }
                else{

                }
                xSemaphoreGive(xDriverStructMutex);
            }
        }
    }
}
/*------------------------------------------------------------------*/
void vPassengerHeaterHandlerTask(void *pvParameters)
{
    uint8_t     desired_received_temp;
    uint8_t     current_received_temp;
    sint8       delta_temp = 0;

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (xSemaphoreTake(xPassengerBinarySemaphore,portMAX_DELAY)==pdTRUE ){

            if( xSemaphoreTake(xPassengerStructMutex,portMAX_DELAY)==pdTRUE ){

                current_received_temp = Passenger_Seat_Info.current_temp;
                desired_received_temp = Passenger_Seat_Info.desired_temp;
                delta_temp = current_received_temp - desired_received_temp ;

                if( delta_temp <= (-10) ){
                    Passenger_Seat_Info.heater_output_signal = HEAT_OUTPUT_HIGH;
                    GPIO_PORTA_Leds_Off();
                    GPIO_PORTA_GreenLedOn();
                    GPIO_PORTA_BlueLedOn();
                    Delay_MS(500);

                }
                else if( (delta_temp <= (-5)) && (delta_temp > (-10))  ){
                    Passenger_Seat_Info.heater_output_signal = HEAT_OUTPUT_MED;
                    GPIO_PORTA_Leds_Off();
                    GPIO_PORTA_BlueLedOn();
                    Delay_MS(500);

                }
                else if( (delta_temp <= (-2)) && (delta_temp > (-5))  ){
                    Passenger_Seat_Info.heater_output_signal = HEAT_OUTPUT_LOW;
                    GPIO_PORTA_Leds_Off();
                    GPIO_PORTA_GreenLedOn();
                    Delay_MS(500);

                }
                else if( delta_temp > (-2) ){
                    Passenger_Seat_Info.heater_output_signal = HEAT_OUTPUT_OFF;
                    GPIO_PORTA_Leds_Off();
                    Delay_MS(500);

                }
                else{

                }
                xSemaphoreGive(xPassengerStructMutex);
            }
        }
    }
}
/*------------------------------------------------------------------*/
void vDriverTempTask(void *pvParameters)

{
    uint8_t current_received_temperature = 25;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        //current_received_temperature = (rand()%35)+5;                   /*Random Number From 5 to 40*/ /*TODO: input from Potentiometer*/
        current_received_temperature = rand()%45;
        xQueueSend(xQueueDriverTemperature,&current_received_temperature,portMAX_DELAY);
    }
}
/*------------------------------------------------------------------*/
void vPassengerTempTask(void *pvParameters)

{
    uint8_t current_received_temperature = 25;
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(5000));
        //current_received_temperature = (rand()%35)+5;                   /*Random Number From 5 to 40*/ /*TODO: input from Potentiometer*/
        current_received_temperature = rand()%45;
        xQueueSend(xQueuePassengerTemperature,&current_received_temperature,portMAX_DELAY);
    }
}
/*------------------------------------------------------------------*/
void vScreenTask(void *pvParameters)
{
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));

        if(xSemaphoreTake(xUARTMutex,portMAX_DELAY)==pdTRUE){
            if(  (xSemaphoreTake(xDriverStructMutex,portMAX_DELAY)==pdTRUE) && xSemaphoreTake(xPassengerStructMutex,portMAX_DELAY)==pdTRUE   ){

                UART_Driver_Seat_Info.seat = "DRIVER_SEAT";

                (Driver_Seat_Info.heat_level == HEAT_LEVEL_OFF)  ? (UART_Driver_Seat_Info.heat_level="OFF") :
                (Driver_Seat_Info.heat_level == HEAT_LEVEL_LOW)  ? (UART_Driver_Seat_Info.heat_level="LOW"):
                (Driver_Seat_Info.heat_level == HEAT_LEVEL_MED)  ? (UART_Driver_Seat_Info.heat_level="MED"):
                (Driver_Seat_Info.heat_level == HEAT_LEVEL_HIGH) ? (UART_Driver_Seat_Info.heat_level="HIGH"):(UART_Driver_Seat_Info.heat_level="INVALID");

                (Driver_Seat_Info.heater_output_signal == HEAT_OUTPUT_OFF)  ? (UART_Driver_Seat_Info.heater_output_signal="HEAT_OUTPUT_OFF") :
                (Driver_Seat_Info.heater_output_signal == HEAT_OUTPUT_LOW)  ? (UART_Driver_Seat_Info.heater_output_signal="HEAT_OUTPUT_LOW") :
                (Driver_Seat_Info.heater_output_signal == HEAT_OUTPUT_MED)  ? (UART_Driver_Seat_Info.heater_output_signal="HEAT_OUTPUT_MED") :
                (Driver_Seat_Info.heater_output_signal == HEAT_OUTPUT_HIGH)  ? (UART_Driver_Seat_Info.heater_output_signal="HEAT_OUTPUT_HIGH") :(UART_Driver_Seat_Info.heater_output_signal="INVALID");

                UART_Driver_Seat_Info.seat = "PASSENGER_SEAT";
                (Passenger_Seat_Info.heat_level == HEAT_LEVEL_OFF)  ? (UART_Passenger_Seat_Info.heat_level="OFF") :
                (Passenger_Seat_Info.heat_level == HEAT_LEVEL_LOW)  ? (UART_Passenger_Seat_Info.heat_level="LOW"):
                (Passenger_Seat_Info.heat_level == HEAT_LEVEL_MED)  ? (UART_Passenger_Seat_Info.heat_level="MED"):
                (Passenger_Seat_Info.heat_level == HEAT_LEVEL_HIGH) ? (UART_Passenger_Seat_Info.heat_level="HIGH"):(UART_Passenger_Seat_Info.heat_level="INVALID");

                (Passenger_Seat_Info.heater_output_signal == HEAT_OUTPUT_OFF)  ? (UART_Passenger_Seat_Info.heater_output_signal="HEAT_OUTPUT_OFF") :
                (Passenger_Seat_Info.heater_output_signal == HEAT_OUTPUT_LOW)  ? (UART_Passenger_Seat_Info.heater_output_signal="HEAT_OUTPUT_LOW") :
                (Passenger_Seat_Info.heater_output_signal == HEAT_OUTPUT_MED)  ? (UART_Passenger_Seat_Info.heater_output_signal="HEAT_OUTPUT_MED") :
                (Passenger_Seat_Info.heater_output_signal == HEAT_OUTPUT_HIGH)  ? (UART_Passenger_Seat_Info.heater_output_signal="HEAT_OUTPUT_HIGH") :(UART_Passenger_Seat_Info.heater_output_signal="INVALID");

                UART0_SendString("Driver Seat: \r\n");
                UART0_SendString("Current Temperature is: ");
                UART0_SendInteger(Driver_Seat_Info.current_temp);
                UART0_SendString(" and Desired Temperature is: ");
                UART0_SendInteger(Driver_Seat_Info.desired_temp);
                UART0_SendString(" and Received Heat Level: ");
                UART0_SendString(UART_Driver_Seat_Info.heat_level);
                UART0_SendString(" and Output Signal is: ");
                UART0_SendString(UART_Driver_Seat_Info.heater_output_signal);
                UART0_SendString("\r\n");
                UART0_SendString("Passenger Seat: \r\n");
                UART0_SendString("Current Temperature is: ");
                UART0_SendInteger(Passenger_Seat_Info.current_temp);
                UART0_SendString(" and Desired Temperature is: ");
                UART0_SendInteger(Passenger_Seat_Info.desired_temp);
                UART0_SendString(" and Received Heat Level: ");
                UART0_SendString(UART_Passenger_Seat_Info.heat_level);
                UART0_SendString(" and Output Signal is: ");
                UART0_SendString(UART_Passenger_Seat_Info.heater_output_signal);
                UART0_SendString("\r\n");
            }

            xSemaphoreGive(xUARTMutex);
            xSemaphoreGive(xDriverStructMutex);
            xSemaphoreGive(xPassengerStructMutex);
        }
    }
}
/*------------------------------------------------------------------*/
void vRunTimeMeasurementsTask(void *pvParameters){


    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;){

        vTaskDelayUntil(&xLastWakeTime, 1000);

        uint8 ucCounter, ucCPU_Load;
        uint32 ullTotalTasksTime = 0;

        if(xSemaphoreTake(xUARTMutex,portMAX_DELAY)==pdTRUE){
        UART0_SendString("----------------Run Time Measurements----------------\r\n");


        for(ucCounter = 1; ucCounter < 10; ucCounter++)
        {
            ullTotalTasksTime += ullTasksTotalTime[ucCounter];
        }
        ucCPU_Load = (ullTotalTasksTime * 100) /  GPTM_WTimer0Read();

        taskENTER_CRITICAL();
        UART0_SendString("Total Time is ");
        UART0_SendInteger(ullTotalTasksTime);
        UART0_SendString("\r\n");
        UART0_SendString("Total CPU Load is ");
        UART0_SendInteger(ucCPU_Load);
        UART0_SendString("% \r\n");
        taskEXIT_CRITICAL();

        UART0_SendString("Button Task execution time is ");
        UART0_SendInteger(ullTasksExecutionTime[1] / 10);
        UART0_SendString(" msec \r\n");

        UART0_SendString("Driver Heater Task execution time is ");
        UART0_SendInteger(ullTasksExecutionTime[2] / 10);
        UART0_SendString(" msec \r\n");

        UART0_SendString("Passenger Heater Task execution time is ");
        UART0_SendInteger(ullTasksExecutionTime[3] / 10);
        UART0_SendString(" msec \r\n");

        UART0_SendString("Driver Handler Task execution time is ");
        UART0_SendInteger(ullTasksExecutionTime[4] / 10);
        UART0_SendString(" msec \r\n");

        UART0_SendString("Passenger Handler Task execution time is ");
        UART0_SendInteger(ullTasksExecutionTime[5] / 10);
        UART0_SendString(" msec \r\n");

        UART0_SendString("Driver Temp Task execution time is ");
        UART0_SendInteger(ullTasksExecutionTime[6] / 10);
        UART0_SendString(" msec \r\n");

        UART0_SendString("Passenger Temp Task execution time is ");
        UART0_SendInteger(ullTasksExecutionTime[7] / 10);
        UART0_SendString(" msec \r\n");

        UART0_SendString("Screen Task execution time is ");
        UART0_SendInteger(ullTasksExecutionTime[8] / 10);
        UART0_SendString(" msec \r\n");

        UART0_SendString("Run Time Measurements Task execution time is ");
        UART0_SendInteger(ullTasksExecutionTime[9] / 10);
        UART0_SendString(" msec \r\n");

        UART0_SendString("------------------------------------------------\r\n");

        xSemaphoreGive(xUARTMutex);
        }

    }

}
/*------------------------------------------------------------------*/
void GPIOPortF_Handler(void){
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

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
        UART0_SendString("-----------------------------------------------------------\r\n");
        UART0_SendString("Idle Task is Processing\r\n");
        UART0_SendString("-----------------------------------------------------------\r\n");
        UART0_SendString("\r\n");
        xSemaphoreGive(xUARTMutex);
    }
}
/*-----------------------------------------------------------*/
