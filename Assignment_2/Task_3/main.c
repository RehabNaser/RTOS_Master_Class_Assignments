/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"


/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */

static void prvSetupHardware( void );
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/
/* xQueue handle used between 4 tasks:
 * 3 tasks -> writing on xQueue.
 * 1 task -> reading from xQueue.
 */
QueueHandle_t  xQueue;
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

#define Task_F100_Frequency 100 // Period (Frequency) of Task_F100.

TaskHandle_t Task_F100Handle = NULL; // Task_F100 handle.

/*	
 * Task_F100 for sending a string every 100ms to the xQueue.
 */
 
void Task_F100( void * Task_F100_Parameters )
{
	uint8_t Task_F100StringBuffer[20]="Hello Rehab"; // An array hold the string which is sent to xQueue every 100ms.
	
	TickType_t xLastWakeTime; // Local variable for saving The last wake time for this task.
	xLastWakeTime = xTaskGetTickCount(); // Initialise the xLastWakeTime variable with the current time.

  for( ;; )
	{
		xQueueSend(xQueue,Task_F100StringBuffer,( TickType_t )portMAX_DELAY);// Sending the string to xQueue.
																																			  // And if there isn't an space on the xQueue, this task will be blocked until xQueue has a free space.
		
		vTaskDelayUntil(&xLastWakeTime,Task_F100_Frequency);
	}
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

#define Button1_Task_Frequency 20 // Period (Frequency) of Button1_Task.

TaskHandle_t Button1_TaskHandle = NULL; // Button1_Task handle.

/*	
 * Button1_Task for sending its state(raising or falling edge) to the xQueue.
 * Button connected on PORT0 on PIN0
 */
 
void Button1_Task( void * Button1_Task_Parameters )
{

	pinState_t ButtonState; // Local variable for saving the button state (pressed or released).
	uint8_t Flag = 0; // Local variable(flag) used to know the start of pressing and start of releasing.
	
	uint8_t RisingEdgeStringBuffer[20]=  "Rising Edge B1"; // An array hold the string which is sent to xQueue on rising edge.
	uint8_t FallingEdgeStringBuffer[20]= "Falling Edge B1"; // An array hold the string which is sent to xQueue on falling edge.
	
	TickType_t xLastWakeTime; // Local variable for saving The last wake time for this task.
	xLastWakeTime = xTaskGetTickCount(); // Initialise the xLastWakeTime variable with the current time.

  for( ;; )
	{
		ButtonState = GPIO_read(PORT_0,PIN0); // Read the button and save its state on ButtonState variable.
		
		if((ButtonState == PIN_IS_HIGH) && (Flag == 0)) // IF the button pressed and it's the starting of pressing(Rising edge).
		{
			xQueueSend(xQueue,RisingEdgeStringBuffer,( TickType_t )portMAX_DELAY); // Sending RisingEdgeStringBuffer to xQueue.
			Flag = 1;
		}
		if((ButtonState == PIN_IS_LOW) && (Flag == 1)) // IF the button released and it's the starting of releasing(Falling edge).
		{
			xQueueSend(xQueue,FallingEdgeStringBuffer,( TickType_t )portMAX_DELAY); // Sending FallingEdgeStringBuffer to xQueue.
			Flag = 0;
		}
		vTaskDelayUntil(&xLastWakeTime,Button1_Task_Frequency);
	}
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

#define Button2_Task_Frequency 20 // Period (Frequency) of Button2_Task.

TaskHandle_t Button2_TaskHandle = NULL; // Button2_Task handle.

/*	
 * Button2_Task for sending its state(raising or falling edge) to the xQueue.
 * Button connected on PORT0 on PIN1
 */
 
void Button2_Task( void * Button2_Task_Parameters )
{
	pinState_t ButtonState; // Local variable for saving the button state (pressed or released).
	uint8_t Flag = 0; // Local variable(flag) used to know the start of pressing and start of releasing.
	
	uint8_t RisingEdgeStringBuffer[20]=  "Rising Edge B2"; // An array hold the string which is sent to xQueue on rising edge.
	uint8_t FallingEdgeStringBuffer[20]= "Falling Edge B2"; // An array hold the string which is sent to xQueue on falling edge.
	
	TickType_t xLastWakeTime; // Local variable for saving The last wake time for this task.
	xLastWakeTime = xTaskGetTickCount(); // Initialise the xLastWakeTime variable with the current time.

  for( ;; )
	{
		ButtonState = GPIO_read(PORT_0,PIN1); // Read the button and save its state on ButtonState variable.
		
		if((ButtonState == PIN_IS_HIGH) && (Flag == 0)) // IF the button pressed and it's the starting of pressing(Rising edge).
		{
			xQueueSend(xQueue,RisingEdgeStringBuffer,( TickType_t )portMAX_DELAY); // Sending RisingEdgeStringBuffer to xQueue.
			Flag = 1;
		}
		if((ButtonState == PIN_IS_LOW) && (Flag == 1)) // IF the button released and it's the starting of releasing(Falling edge).
		{
			xQueueSend(xQueue,FallingEdgeStringBuffer,( TickType_t )portMAX_DELAY); // Sending FallingEdgeStringBuffer to xQueue.
			Flag = 0;
		}
		vTaskDelayUntil(&xLastWakeTime,Button2_Task_Frequency);
	}
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

#define UARTConsumer_Task_Frequency 20 // Period (Frequency) of UARTConsumer_Task.

TaskHandle_t UARTConsumer_TaskHandle = NULL; // UARTConsumer_Task handle.

/*	
 * UARTConsumer_Task for writing the strings which in the xQueue on the UART.
 */
 
void UARTConsumer_Task( void * UARTConsumer_Task_Parameters )
{
	uint8_t UARTStringBuffer[20]={0}; // An array used to save the strings which come from the xQueueReceive function.
																		// And send to vSerialPutString function.
  for( ;; )
	{
		xQueueReceive(xQueue,UARTStringBuffer,( TickType_t )portMAX_DELAY); // Reading a new data(string) from the xQueue.
																																			  // And if there isn't an new data, this task will be blocked until receive a new data.
		vSerialPutString(UARTStringBuffer,20); // Sending the new string to the UART. 
		vTaskDelay(UARTConsumer_Task_Frequency);
		xSerialPutChar('\n'); // Send a new line character to separate between every string.
	}
}
/*-----------------------------------------------------------*/

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */

int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();
	
	xQueue = xQueueCreate(5,20*sizeof(unsigned char)); // Creating a Queue from 5 locations each location has a size of our string (20 characters).
	
	/* Create Tasks here */
	xTaskCreate(	Task_F100, "Task F100",	100, ( void * ) 0, 2, &Task_F100Handle);	//Creating Task_F100
	xTaskCreate(	Button1_Task, "Button1 Task",	100, ( void * ) 0, 3, &Button1_TaskHandle);	//Creating Button1_Task	
	xTaskCreate(	Button2_Task, "Button2 Task",	100, ( void * ) 0, 3, &Button2_TaskHandle);	//Creating Button2_Task
	xTaskCreate(	UARTConsumer_Task, "UARTConsumer Task",	100, ( void * ) 0, 1, &UARTConsumer_TaskHandle);	//Creating UARTConsumer_Task

	/* Now all the tasks have been started - start the scheduler.
	
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */		
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}

/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/


