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
#include "adc.h"
#include "eeprom.h"
#include "tm4c123gh6pm_registers.h"

/*---------------------------------------------------------------------------*/
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
/*---------------------------------------------------------------------------*/
uint8_t  DriverTemperature;
uint8_t  PassengerTemperature;

/*---------------------------------------------------------------------------*/
/* The HW setup function */
static void prvSetupHardware( void );
/*---------------------------------------------------------------------------*/
/*-----  FreeRTOS tasks--------------------------------------- */
void vButtonTask(void *pvParameters);

void vDriverHeaterTask(void *pvParameters);
void vPassengerHeaterTask(void *pvParameters);

void vDriverHeaterHandlerTask(void *pvParameters);
void vPassengerHeaterHandlerTask(void *pvParameters);

void vDriverTempTask(void *pvParameters);
void vPassengerTempTask(void *pvParameters);

void vScreenTask(void *pvParameters);

void vRunTimeMeasurementsTask(void *pvParameters);
/*---------------------------------------------------------------------------*/
/*----- FreeRTOS Objects---------------------------------------*/

/* Tasks Handlers */
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
EventGroupHandle_t xUpdateScreenEventGroup;
/* Definitions for the event bits in the event group. */
#define mainSW2_INTERRUPT_BIT        ( 1UL << 0UL )  /* Event bit 0, which is set by a SW2 Interrupt. */
#define mainSW1_INTERRUPT_BIT        ( 1UL << 1UL )  /* Event bit 1, which is set by a SW1 Interrupt. */
#define DriverSeat_Temp_BIT         ( 1UL << 0UL )  /* Event bit 0, which is set by Driver Heat Handler. */
#define PassengerSeat_Temp_BIT      ( 1UL << 1UL )  /* Event bit 0, which is set by Passenger Heat Handler. */

/*FreeRTOS Semaphores & Mutexes*/
SemaphoreHandle_t xDriverBinarySemaphore;
SemaphoreHandle_t xPassengerBinarySemaphore;
SemaphoreHandle_t xUARTMutex;
SemaphoreHandle_t xDriverStructMutex;
SemaphoreHandle_t xPassengerStructMutex;

/*FreeRTOS Queue*/
QueueSetHandle_t xQueueDriverTemperature;
QueueSetHandle_t xQueuePassengerTemperature;

/*---------------------------------------------------------------------------*/
/*Error Info Structure*/
typedef struct{

   uint32  ErrorTime;
   uint8   *Error;
   uint8   *HeatingLevel;
   uint8   SensorReading;
}Error_Info;

/*Seat Info Structure*/
typedef struct{
    uint8_t   seat;
    uint8_t   heat_level;
    uint8_t   desired_temp;
    uint8_t   current_temp;
    uint8_t   heater_output_signal;
}Input_Info;

/*Create Seat_Info Struct*/
Input_Info Driver_Seat_Info    = {0,0,0,0};
Input_Info Passenger_Seat_Info = {0,0,0,0};

/*Create Error_Info Struct*/
Error_Info Driver_Error = {0,"",""};
Error_Info Passenger_Error = {0,"",""};
/*---------------------------------------------------------------------------*/
/*UART data*/
uint8_t    Driver_Seat_CurrentTemp ;
uint8_t    Driver_Seat_DesiredTemp ;
uint8_t   *Driver_Seat_HeatingLevel = "";
uint8_t   *Driver_Seat_HeaterState ;

uint8_t    Passenger_Seat_CurrentTemp ;
uint8_t    Passenger_Seat_DesiredTemp ;
uint8_t   *Passenger_Seat_HeatingLevel= "";
uint8_t   *Passenger_Seat_HeaterState ;
/*---------------------------------------------------------------------------*/
/*-----Run Time Measurements---------------------------------------*/

uint32 ullTasksOutTime[10];
uint32 ullTasksInTime[10];
uint32 ullTasksTotalTime[10];
uint32 ullTasksExecutionTime[10];

uint32 Resource_LockTime[5];
uint32 Resource_UnLockTime[5];
uint32 Resource_TotalLockTime[5];

uint32 DriverERROR_Data[13];
uint32 DriverERROR_Read[13];

uint32 PassengerERROR_Data[13];
uint32 PassengerERROR_Read[13];

uint32 driver_error_time;
uint32 passenger_error_time;

char dr_hl_c1;
char dr_hl_c2;
char dr_hl_c3;
char dr_hl_c4;
char dr_err_c1;
char dr_err_c2;
char dr_err_c3;
char dr_err_c4;

char ps_hl_c1;
char ps_hl_c2;
char ps_hl_c3;
char ps_hl_c4;
char ps_err_c1;
char ps_err_c2;
char ps_err_c3;
char ps_err_c4;
/*---------------------------------------------------------------------------*/

int main(void)
{
    /* Setup the hardware for use with the Tiva C board. */
    prvSetupHardware();

    /*Creating Event Groups*/
    xButtonsEventGroup = xEventGroupCreate();
    xUpdateScreenEventGroup = xEventGroupCreate();

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
    xTaskCreate(vButtonTask, "Button Task", configMINIMAL_STACK_SIZE, NULL, 4, &xButtonTaskTaskHandle);


    /*Responsible for Receiving the data (which button is pressed and what is the desired Heating Level) acquired from vButtonTask via Message Queue
     * and pass these data to vHeaterHandlerTask*/
    xTaskCreate(vDriverHeaterTask, "Driver Heater", configMINIMAL_STACK_SIZE, NULL, 2, &xDriverHeaterTaskHandle);
    xTaskCreate(vPassengerHeaterTask, "Passenger Heater", configMINIMAL_STACK_SIZE, NULL, 2, &xPassengerHeaterTaskHandle);

    /*Responsible for Receiving the data acquired from vHeaterMonitorTask via Message Queue
     * and pass these data to vHeaterHandlerTask*/
    xTaskCreate(vDriverHeaterHandlerTask, "Driver Handler", configMINIMAL_STACK_SIZE, NULL, 3, &xDriverHeaterHandlerTaskHandle);
    xTaskCreate(vPassengerHeaterHandlerTask, "Passenger Handler", configMINIMAL_STACK_SIZE, NULL, 3, &xPassengerHeaterHandlerTaskHandle);

    xTaskCreate(vDriverTempTask, "Driver Temp ", configMINIMAL_STACK_SIZE, NULL, 3, &xDriverTempTaskHandle);
    xTaskCreate(vPassengerTempTask, "Passenger Temperature ", configMINIMAL_STACK_SIZE, NULL, 3, &xPassengerTempTaskHandle);

    xTaskCreate(vScreenTask, "Screen Task", configMINIMAL_STACK_SIZE, NULL, 1, &xScreenTaskHandle);

    xTaskCreate(vRunTimeMeasurementsTask, "Run-time Measurements", 256, NULL, 1, &xRunTimeAnalysisTaskHandle);


    vTaskSetApplicationTaskTag( xButtonTaskTaskHandle,              ( TaskHookFunction_t ) 1 );
    vTaskSetApplicationTaskTag( xDriverHeaterTaskHandle,            ( TaskHookFunction_t ) 2 );
    vTaskSetApplicationTaskTag( xPassengerHeaterTaskHandle,         ( TaskHookFunction_t ) 3 );
    vTaskSetApplicationTaskTag( xDriverHeaterHandlerTaskHandle,     ( TaskHookFunction_t ) 4 );
    vTaskSetApplicationTaskTag( xPassengerHeaterHandlerTaskHandle,  ( TaskHookFunction_t ) 5 );
    vTaskSetApplicationTaskTag( xDriverTempTaskHandle,              ( TaskHookFunction_t ) 6 );
    vTaskSetApplicationTaskTag( xPassengerTempTaskHandle,           ( TaskHookFunction_t ) 7 );
    vTaskSetApplicationTaskTag( xScreenTaskHandle,                  ( TaskHookFunction_t ) 8 );
    vTaskSetApplicationTaskTag( xRunTimeAnalysisTaskHandle,         ( TaskHookFunction_t ) 9 );

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
    ADCInit();
    UART0_Init();
    EEPROMInit();


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
                    case 0 : Driver_Seat_Info.heat_level = HEAT_LEVEL_OFF;  Driver_Seat_HeatingLevel = "OFF";  break;
                    case 1 : Driver_Seat_Info.heat_level = HEAT_LEVEL_LOW;  Driver_Seat_HeatingLevel = "LOW"; break;
                    case 2 : Driver_Seat_Info.heat_level = HEAT_LEVEL_MED;  Driver_Seat_HeatingLevel = "MED"; break;
                    case 3 : Driver_Seat_Info.heat_level = HEAT_LEVEL_HIGH; Driver_Seat_HeatingLevel = "HIGH"; break;
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
                    case 0 : Passenger_Seat_Info.heat_level = HEAT_LEVEL_OFF;  Passenger_Seat_HeatingLevel = "OFF"; break;
                    case 1 : Passenger_Seat_Info.heat_level = HEAT_LEVEL_LOW;  Passenger_Seat_HeatingLevel = "LOW"; break;
                    case 2 : Passenger_Seat_Info.heat_level = HEAT_LEVEL_MED;  Passenger_Seat_HeatingLevel = "MED"; break;
                    case 3 : Passenger_Seat_Info.heat_level = HEAT_LEVEL_HIGH; Passenger_Seat_HeatingLevel = "HIGH"; break;
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
        if( xQueueReceive(xQueueDriverTemperature, &current_received_temperature, portMAX_DELAY) == pdTRUE ){

            if(xSemaphoreTake(xDriverStructMutex,portMAX_DELAY)==pdTRUE){


                Driver_Seat_Info.current_temp = current_received_temperature;

                switch(Driver_Seat_Info.heat_level){
                case HEAT_LEVEL_OFF :  Driver_Seat_Info.desired_temp = HEAT_LEVEL_OFF_DESIRED_TEMP;  break;
                case HEAT_LEVEL_LOW :  Driver_Seat_Info.desired_temp = HEAT_LEVEL_LOW_DESIRED_TEMP;  break;
                case HEAT_LEVEL_MED :  Driver_Seat_Info.desired_temp = HEAT_LEVEL_MED_DESIRED_TEMP;  break;
                case HEAT_LEVEL_HIGH : Driver_Seat_Info.desired_temp = HEAT_LEVEL_HIGH_DESIRED_TEMP; break;
                default: break;
                }

                xSemaphoreGive(xDriverStructMutex);

            }
            if(  (Driver_Seat_Info.current_temp > 40) || (Driver_Seat_Info.current_temp <5) ){

                vTaskSuspend(xDriverHeaterHandlerTaskHandle);
                GPIO_PORTF_Leds_Off();
                GPIO_RedLedOn();

                Driver_Seat_Info.heater_output_signal = HEAT_OUTPUT_OFF;
                Driver_Seat_CurrentTemp = Driver_Seat_Info.current_temp ;
                Driver_Seat_HeaterState="OFF";

                Driver_Error.ErrorTime = GPTM_WTimer0Read()/10;
                Driver_Error.SensorReading = Driver_Seat_CurrentTemp;
                Driver_Error.Error = "Temp";

                (Driver_Seat_Info.heat_level == HEAT_LEVEL_OFF)  ? (Driver_Error.HeatingLevel="OFF") :
                (Driver_Seat_Info.heat_level == HEAT_LEVEL_LOW)  ? (Driver_Error.HeatingLevel="LOW"):
                (Driver_Seat_Info.heat_level == HEAT_LEVEL_MED)  ? (Driver_Error.HeatingLevel="MED"):
                (Driver_Seat_Info.heat_level == HEAT_LEVEL_HIGH) ? (Driver_Error.HeatingLevel="HIGH"):(Driver_Error.HeatingLevel="INVALID");

                 time_split_write(Driver_Error.ErrorTime,DriverERROR_Data);
                 string_split_write(Driver_Error.HeatingLevel, Driver_Error.Error, DriverERROR_Data);
                 DriverERROR_Data[4] = Driver_Seat_CurrentTemp;
                 EEPROMProgram(DriverERROR_Data, 0x0, sizeof(DriverERROR_Data));
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
        if( xQueueReceive(xQueuePassengerTemperature, &current_received_temperature, portMAX_DELAY) == pdTRUE ){

            if(xSemaphoreTake(xPassengerStructMutex,portMAX_DELAY)==pdTRUE){


                Passenger_Seat_Info.current_temp = current_received_temperature;
                Passenger_Seat_CurrentTemp = current_received_temperature;

                switch(Passenger_Seat_Info.heat_level){
                case HEAT_LEVEL_OFF :  Passenger_Seat_Info.desired_temp = HEAT_LEVEL_OFF_DESIRED_TEMP;   break;
                case HEAT_LEVEL_LOW :  Passenger_Seat_Info.desired_temp = HEAT_LEVEL_LOW_DESIRED_TEMP;   break;
                case HEAT_LEVEL_MED :  Passenger_Seat_Info.desired_temp = HEAT_LEVEL_MED_DESIRED_TEMP;   break;
                case HEAT_LEVEL_HIGH : Passenger_Seat_Info.desired_temp = HEAT_LEVEL_HIGH_DESIRED_TEMP;  break;
                default: break;
                }

                xSemaphoreGive(xPassengerStructMutex);

            }
            if(  (Passenger_Seat_Info.current_temp > 40) || (Passenger_Seat_Info.current_temp <5)  ){

                vTaskSuspend(xPassengerHeaterHandlerTaskHandle);
                GPIO_PORTA_Leds_Off();
                GPIO_PORTA_RedLedOn();

                Passenger_Seat_Info.heater_output_signal = HEAT_OUTPUT_OFF;
                Passenger_Seat_CurrentTemp = Passenger_Seat_Info.current_temp ;
                Passenger_Seat_HeaterState="OFF";

                Passenger_Error.ErrorTime = GPTM_WTimer0Read()/10;
                Passenger_Error.Error = "Temp";
                Passenger_Error.SensorReading = Passenger_Seat_CurrentTemp;

                (Passenger_Seat_Info.heat_level == HEAT_LEVEL_OFF)  ? (Passenger_Error.HeatingLevel="OFF") :
                (Passenger_Seat_Info.heat_level == HEAT_LEVEL_LOW)  ? (Passenger_Error.HeatingLevel="LOW"):
                (Passenger_Seat_Info.heat_level == HEAT_LEVEL_MED)  ? (Passenger_Error.HeatingLevel="MED"):
                (Passenger_Seat_Info.heat_level == HEAT_LEVEL_HIGH) ? (Passenger_Error.HeatingLevel="HIGH"):(Passenger_Error.HeatingLevel="INVALID");

                time_split_write(Passenger_Error.ErrorTime,PassengerERROR_Data);
                string_split_write(Passenger_Error.HeatingLevel, Passenger_Error.Error, PassengerERROR_Data);
                PassengerERROR_Data[4] = Passenger_Seat_CurrentTemp;
                EEPROMProgram(PassengerERROR_Data, 0x400, sizeof(PassengerERROR_Data));

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
        if (xSemaphoreTake(xDriverBinarySemaphore,portMAX_DELAY)==pdTRUE ){

            if( xSemaphoreTake(xDriverStructMutex,portMAX_DELAY)==pdTRUE ){

                current_received_temp = Driver_Seat_Info.current_temp;
                desired_received_temp = Driver_Seat_Info.desired_temp;
                Driver_Seat_CurrentTemp = current_received_temp;
                Driver_Seat_DesiredTemp = desired_received_temp;

                delta_temp = current_received_temp - desired_received_temp ;

                if( delta_temp <= (-10) ){
                    Driver_Seat_Info.heater_output_signal = HEAT_OUTPUT_HIGH;  /*Cyan*/
                    Driver_Seat_HeaterState="HIGH";
                    GPIO_PORTF_Leds_Off();
                    GPIO_GreenLedOn();
                    GPIO_BlueLedOn();
                }
                else if( (delta_temp <= (-5)) && (delta_temp > (-10))  ){
                    Driver_Seat_Info.heater_output_signal = HEAT_OUTPUT_MED; /*Blue*/
                    Driver_Seat_HeaterState="MED";
                    GPIO_PORTF_Leds_Off();
                    GPIO_BlueLedOn();
                }
                else if( (delta_temp <= (-2)) && (delta_temp > (-5))  ){
                    Driver_Seat_Info.heater_output_signal = HEAT_OUTPUT_LOW; /*Green*/
                    Driver_Seat_HeaterState="LOW";
                    GPIO_PORTF_Leds_Off();
                    GPIO_GreenLedOn();
                }
                else if( delta_temp > (-2) ){
                    Driver_Seat_Info.heater_output_signal = HEAT_OUTPUT_OFF;
                    Driver_Seat_HeaterState="OFF";
                    GPIO_PORTF_Leds_Off();
                }
                else{

                }
                xSemaphoreGive(xDriverStructMutex);
            }
            xEventGroupSetBits(xUpdateScreenEventGroup, DriverSeat_Temp_BIT);
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
        if (xSemaphoreTake(xPassengerBinarySemaphore,portMAX_DELAY)==pdTRUE ){

            if( xSemaphoreTake(xPassengerStructMutex,portMAX_DELAY)==pdTRUE ){

                current_received_temp = Passenger_Seat_Info.current_temp;
                desired_received_temp = Passenger_Seat_Info.desired_temp;
                Passenger_Seat_CurrentTemp = current_received_temp;
                Passenger_Seat_DesiredTemp = desired_received_temp;
                delta_temp = current_received_temp - desired_received_temp ;

                if( delta_temp <= (-10) ){
                    Passenger_Seat_Info.heater_output_signal = HEAT_OUTPUT_HIGH;
                    Passenger_Seat_HeaterState="HIGH";
                    GPIO_PORTA_Leds_Off();
                    GPIO_PORTA_GreenLedOn();
                    GPIO_PORTA_BlueLedOn();
                }
                else if( (delta_temp <= (-5)) && (delta_temp > (-10))  ){
                    Passenger_Seat_Info.heater_output_signal = HEAT_OUTPUT_MED;
                    Passenger_Seat_HeaterState="MED";
                    GPIO_PORTA_Leds_Off();
                    GPIO_PORTA_BlueLedOn();
                }
                else if( (delta_temp <= (-2)) && (delta_temp > (-5))  ){
                    Passenger_Seat_Info.heater_output_signal = HEAT_OUTPUT_LOW;
                    Passenger_Seat_HeaterState="LOW";
                    GPIO_PORTA_Leds_Off();
                    GPIO_PORTA_GreenLedOn();
                }
                else if( delta_temp > (-2) ){
                    Passenger_Seat_Info.heater_output_signal = HEAT_OUTPUT_OFF;
                    Passenger_Seat_HeaterState="OFF";
                    GPIO_PORTA_Leds_Off();
                }
                else{

                }
                xSemaphoreGive(xPassengerStructMutex);
            }
            xEventGroupSetBits(xUpdateScreenEventGroup, PassengerSeat_Temp_BIT);
        }
    }
}
/*------------------------------------------------------------------*/
void vDriverTempTask(void *pvParameters)
{
    for (;;)
    {

        xQueueSend(xQueueDriverTemperature,&DriverTemperature,portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(2000));
        ADC0_PSSI_REG |= (1<<3);        /* Enable SS3 conversion or start sampling data from AN0 */
    }
}
/*------------------------------------------------------------------*/
void vPassengerTempTask(void *pvParameters)
{
    for (;;)
    {
        xQueueSend(xQueuePassengerTemperature,&PassengerTemperature,portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(2000));
        ADC0_PSSI_REG |= (1<<2);                /* Enable SS2 conversion or start sampling data from AN0 */
    }
}
/*------------------------------------------------------------------*/
void vScreenTask(void *pvParameters)
{

    const EventBits_t xBitsToWaitFor = (DriverSeat_Temp_BIT | PassengerSeat_Temp_BIT); /*Waiting for Driver Handler or Passenger Handler*/
    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        /* Block to wait for event bits to become set within the event group. */
        xEventGroupWaitBits( xUpdateScreenEventGroup,    /* The event group to read. */
                                   xBitsToWaitFor,              /* Bits to test. */
                                   pdTRUE,                      /* Clear bits on exit if the unblock condition is met. */
                                   pdFALSE,                     /* Don't wait for all bits. */
                                   portMAX_DELAY);              /* Don't time out. */

            if(xSemaphoreTake(xUARTMutex,portMAX_DELAY)==pdTRUE){

                    UART0_SendString("Driver Seat: \r\n");
                    UART0_SendString("Current Temperature is: ");
                    UART0_SendInteger(Driver_Seat_CurrentTemp);
                    UART0_SendString("\r\n");
                    UART0_SendString("Desired Temperature is: ");
                    UART0_SendInteger(Driver_Seat_DesiredTemp);
                    UART0_SendString("\r\n");
                    UART0_SendString("Received Heat Level: ");
                    UART0_SendString(Driver_Seat_HeatingLevel);
                    UART0_SendString("\r\n");
                    UART0_SendString("Output Signal is: ");
                    UART0_SendString(Driver_Seat_HeaterState);
                    UART0_SendString("\r\n");
                    UART0_SendString( "-------------------------------------------\r\n" );
                    UART0_SendString("Passenger Seat: \r\n");
                    UART0_SendString("Current Temperature is: ");
                    UART0_SendInteger(Passenger_Seat_CurrentTemp);
                    UART0_SendString("\r\n");
                    UART0_SendString("Desired Temperature is: ");
                    UART0_SendInteger(Passenger_Seat_DesiredTemp);
                    UART0_SendString("\r\n");
                    UART0_SendString("Received Heat Level: ");
                    UART0_SendString(Passenger_Seat_HeatingLevel);
                    UART0_SendString("\r\n");
                    UART0_SendString("Output Signal is: ");
                    UART0_SendString(Passenger_Seat_HeaterState);
                    UART0_SendString("\r\n");
                    UART0_SendString( "-------------------------------------------\r\n" );

                    xSemaphoreGive(xUARTMutex);
            }
    }
}
/*------------------------------------------------------------------*/
void vRunTimeMeasurementsTask(void *pvParameters){

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;){

        uint8 ucCounter, ucCPU_Load;
        uint32 ullTotalTasksTime = 0;
        vTaskDelayUntil(&xLastWakeTime, 10000);
        if(xSemaphoreTake(xUARTMutex,portMAX_DELAY)==pdTRUE){
            taskENTER_CRITICAL();
            UART0_SendString("---------------------Run Time Analysis-------------------------\r\n");

            for(ucCounter = 1; ucCounter < 10; ucCounter++)
            {
                ullTotalTasksTime += ullTasksTotalTime[ucCounter];
            }
            ucCPU_Load = (ullTotalTasksTime * 100) /  GPTM_WTimer0Read();

            UART0_SendString("CPU Load is ");
            UART0_SendInteger(ucCPU_Load);
            UART0_SendString("%");
            UART0_SendString( "-------------------------------------------\r\n" );

            UART0_SendString("Button execution time is: ");
            if( (ullTasksExecutionTime[1] / 10) == 0 )  { ullTasksExecutionTime[1] = 1;}
            UART0_SendInteger( ullTasksExecutionTime[1] );
            UART0_SendString(" msec \r\n");

            UART0_SendString("Driver Heater execution time is: ");
            if( (ullTasksExecutionTime[2] / 10) == 0 )  { ullTasksExecutionTime[2] = 1;}
            UART0_SendInteger(ullTasksExecutionTime[2] );
            UART0_SendString(" msec \r\n");

            UART0_SendString("Passenger Heater execution time is: ");
            if( (ullTasksExecutionTime[3] / 10) == 0 )  { ullTasksExecutionTime[3] = 1;}
            UART0_SendInteger(ullTasksExecutionTime[3] );
            UART0_SendString(" msec \r\n");

            UART0_SendString("Driver Handler execution time is: ");
            if( (ullTasksExecutionTime[4] / 10) == 0 )  { ullTasksExecutionTime[4] = 1;}
            UART0_SendInteger(ullTasksExecutionTime[4] );
            UART0_SendString(" msec \r\n");

            UART0_SendString("Passenger Handler execution time is: ");
            if( (ullTasksExecutionTime[5] / 10) == 0 )  { ullTasksExecutionTime[5] = 1;}
            UART0_SendInteger(ullTasksExecutionTime[5] );
            UART0_SendString(" msec \r\n");

            UART0_SendString("Driver Temp execution time is: ");
            if( (ullTasksExecutionTime[6] / 10) == 0 )  { ullTasksExecutionTime[6] = 1;}
            UART0_SendInteger(ullTasksExecutionTime[6] );
            UART0_SendString(" msec \r\n");

            UART0_SendString("Passenger Temp execution time is: ");
            if( (ullTasksExecutionTime[7] / 10) == 0 )  { ullTasksExecutionTime[7] = 1;}
            UART0_SendInteger(ullTasksExecutionTime[7] );
            UART0_SendString(" msec \r\n");

            UART0_SendString("Screen execution time is: ");
            UART0_SendInteger(ullTasksExecutionTime[8] /10 );
            UART0_SendString(" msec \r\n");

            UART0_SendString("-------------------------Error Report------------------------\r\n");

            EEPROMRead(DriverERROR_Read, 0x0, sizeof(DriverERROR_Read));
            EEPROMRead(PassengerERROR_Read, 0x400, sizeof(PassengerERROR_Read));

            driver_error_time = time_concate_read(DriverERROR_Read);
            passenger_error_time = time_concate_read(PassengerERROR_Read);
            string_concate_read(DriverERROR_Read,PassengerERROR_Read);

            UART0_SendString("Driver Error is: ");
            UART0_SendByte(dr_err_c1); UART0_SendByte(dr_err_c2); UART0_SendByte(dr_err_c3); UART0_SendByte(dr_err_c4);
            UART0_SendString(" - At Time : ");
            UART0_SendInteger(driver_error_time);
            UART0_SendString(" msec");
            UART0_SendString(" - At Level : ");
            UART0_SendByte(dr_hl_c1); UART0_SendByte(dr_hl_c2); UART0_SendByte(dr_hl_c3); UART0_SendByte(dr_hl_c4);
            UART0_SendString(" - Sensor Reading : ");
            UART0_SendInteger(DriverERROR_Read[4]);
            UART0_SendString(" deg\r\n");

            UART0_SendString("Passenger Error is: ");
            UART0_SendByte(ps_err_c1); UART0_SendByte(ps_err_c2); UART0_SendByte(ps_err_c3); UART0_SendByte(ps_err_c4);
            UART0_SendString(" - At Time : ");
            UART0_SendInteger(passenger_error_time);
            UART0_SendString(" msec");
            UART0_SendString(" - At Level : ");
            UART0_SendByte(ps_hl_c1); UART0_SendByte(ps_hl_c2); UART0_SendByte(ps_hl_c3); UART0_SendByte(ps_hl_c4);
            UART0_SendString(" - Sensor Reading : ");
            UART0_SendInteger(PassengerERROR_Read[4] );
            UART0_SendString(" deg\r\n");

            UART0_SendString("-------------------------Resources--------------------------\r\n");
            UART0_SendString("Driver Mutex Lock Time is: ");
            if( (Resource_TotalLockTime[0] / 10) == 0 )  { Resource_TotalLockTime[0] = 1;}
            UART0_SendInteger(Resource_TotalLockTime[0]);
            UART0_SendString(" msec\r\n");
            if( (Resource_TotalLockTime[1] / 10) == 0 )  { Resource_TotalLockTime[1] = 1;}
            UART0_SendString("Passenger Mutex Lock Time is: ");
            UART0_SendInteger(Resource_TotalLockTime[1]);
            UART0_SendString(" msec\r\n");
            UART0_SendString("Driver Semaphore Lock Time is: ");
            UART0_SendInteger(Resource_TotalLockTime[2]/10);
            UART0_SendString(" msec\r\n");
            UART0_SendString("Passenger Semaphore Lock Time is: ");
            UART0_SendInteger(Resource_TotalLockTime[3]/10);
            UART0_SendString(" msec\r\n");
            UART0_SendString("UART Mutex Lock Time is: ");
            UART0_SendInteger(Resource_TotalLockTime[4]/10);
            UART0_SendString(" msec\r\n");
            UART0_SendString( "-------------------------------------------\r\n" );

            taskEXIT_CRITICAL();
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

void ADC0SS2_Handler(void){

    float64 adc_value ;
    adc_value  = ADC0_SSFIFO2_REG;              /* read adc coversion result from SS2 FIFO*/
    adc_value = adc_value * 0.0008;
    PassengerTemperature = 45*(adc_value/3.3);
    ADC0_ISC_REG |= (1<<2);                           /* clear coversion clear flag bit*/

}

void ADC0SS3_Handler(void){

    float64 adc_value ;
    adc_value  = ADC0_SSFIFO3_REG;              /* read adc coversion result from SS3 FIFO*/
    adc_value = adc_value * 0.0008;
    DriverTemperature = 45*(adc_value/3.3);
    ADC0_ISC_REG |= (1<<3);                            /* clear coversion clear flag bit*/

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
