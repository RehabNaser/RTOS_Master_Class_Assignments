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

/*	
 * LedToggleTime is a Global variable 
 * which is updated by Button_Task depend on the time of bush-button pressing 
 * and used by LED_Task to adjust the toggling time of the LED.
 */
uint16_t LedToggleTime = 0;

/*-----------------------------------------------------------*/

/*	
 * Button_Task for changing the toggling time depend on the push-button pressing time.
 		 * (IF push-button pressing time <= 2) then toggling time will be 0.
		 * (IF push-button pressing time > 2 && <4) then toggling time will be 400 msec.
		 * (IF push-button pressing time >= 4) then toggling time will be 100 msec.
 * Button connected on PORT0 on PIN0.
 */

TaskHandle_t ButtonTaskHandle = NULL; // Button_Task handle.

void Button_Task( void * Button_Task_Parameters )
{
	pinState_t ButtonState; // Local variable for saving the button state (pressed or released).
	TickType_t NumberOfStartTicks = 0; // Local variable to save the number of ticks when the button is pressed.
	TickType_t TicksNumber = 0; // Local variable to save the total number of ticks from pressing till releasing.
	float TotalTime = 0; // Local variable to save the total time from button pressing till button releasing.
		
	static uint8_t Flag = 0; // Static variable(flag) used to know the start of pressing and start of releasing.
	
    for( ;; )
    {
		ButtonState = GPIO_read(PORT_0,PIN0); // Read the button and save its state on ButtonState variable.
		
		if((ButtonState == PIN_IS_HIGH) && (Flag == 0)) // IF button is pressed and it's the starting of pressing.
		{
			// Save the number of ticks when the button starting on pressing on NumberOfStartTicks variable.
			// xTaskGetTickCount() an API to know numbers of ticks which passed from starting the scheduler.
			NumberOfStartTicks = xTaskGetTickCount();
			
			Flag = 1;
		}
		if((ButtonState == PIN_IS_LOW) && (Flag == 1)) // IF button is released and it's the starting of releasing.
		{
			// Calculate total numbers of ticks which passed from the starting of pressing to the end of the pressing.
			TicksNumber = xTaskGetTickCount() - NumberOfStartTicks;
			
			// Calculate total time which passed from the starting of pressing to the end of the pressing
			// by multiplying TicksNumber * time of the one tick.
			// TotalTime variable has the total time in Seconds.
			TotalTime = TicksNumber * (1.0/configTICK_RATE_HZ);
			
			if(TotalTime >= 4) // (IF push-button pressing time >= 4).
			{
				LedToggleTime = 100; // Then toggling time will be 100 msec.
			}
			else if((TotalTime < 4) && (TotalTime > 2)) // (IF push-button pressing time > 2 && <4).
			{
				LedToggleTime = 400; // Then toggling time will be 400 msec.
			}
			else // (IF push-button pressing time <= 2).
			{
				LedToggleTime = 0; // Then toggling time will be 0.
			}
			Flag = 0;
		}
		vTaskDelay(10);
	}
}
/*-----------------------------------------------------------*/

/*	
 * LED_Task for toggling an LED by different frequencies.
		 * (IF LedToggleTime == 0 ) then LED will be off.
		 * (IF LedToggleTime!=0) then LED will toggle by value of LedToggleTime variable.
 * LED connected on PORT0 on PIN1.
 * LED_Task use LedToggleTime variable (global) to know the toggling rate.
 */

TaskHandle_t LEDTaskHandle = NULL; // LED_Task handle.

void LED_Task( void * LED_Task_Parameters )
{
    for( ;; )
    {
		if(LedToggleTime == 0) // IF(LedToggleTime == 0) then LED will be off.
		{
			GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
			vTaskDelay(10);
		}
		else // IF(LedToggleTime != 0) then LED will toggle by value of LedToggleTime variable.
		{
			GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);
			vTaskDelay(LedToggleTime);
			
			GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
			vTaskDelay(LedToggleTime);
		}
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
	
	/* Create Tasks here */
	
	xTaskCreate( LED_Task,  "LED Task",  100, (void *)0, 1, &LEDTaskHandle);
	xTaskCreate( Button_Task,  "Button Task",  100, (void *)0, 1, &ButtonTaskHandle);

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


