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
void timer1Reset(void);
static void configTimer1(void);

/*-----------------------------------------------------------*/

// Tick Hook implementation
void vApplicationTickHook( void )
{
	GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);
	GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
}

/*-----------------------------------------------------------*/

volatile TickType_t TotalSystemTime=0;	/* Global variable to save the total system time which used in CPU load calculations. */
volatile float CPULoad=0; 							/* Global variable for saving the CPU load. */
char RunTimeStatsBuff[130]; 				  	/* Global array to save the system's run time stats which is coming from vTaskGetRunTimeStats(). */

/*-----------------------------------------------------------*/

#define TASK1_TAG			1														/* Task1 Tag. */
#define TASK1_PERIOD 	20  												/* Task1 Period. */

TaskHandle_t Task1Handle = NULL; 									/* Task1 Handler. */

volatile TickType_t Task1LastDeadline=0;          /* variable to save the last deadline of the task1. */
volatile TickType_t Task1EndTime=0;								/* variable to save the end of the execution time for the task1. */
volatile uint32_t Task1DeadlineMissesCount=0;     /* variable to save the numbers of the deadline's misses for the task1. */

/*
 * Task1TimeIn -> variable to save the time in of the task1, used to calculate the total time for task1.
 * Task1TimeOut -> variable to save the time out of the task1, used to calculate the total time for task1. 
 * Task1TimeTotal -> variable to save the total time of the task1, used to calculate the CPU load. 
 */
volatile TickType_t Task1TimeIn=0,Task1TimeOut=0,Task1TimeTotal=0;

void Task1( void * Task1_Parameters )
{
	uint32_t i=0;
	
	TickType_t Task1LastWakeTime;										/* Local variable to save the last wake time of the task1. */
	Task1LastWakeTime =xTaskGetTickCount();					/* Initialise the Task1LastWakeTime variable with the current time. */
	
	/* Initialise the Task1LastDeadline variable with the current deadline. */
	Task1LastDeadline=vTaskGetApplicationTaskItemValue(Task1Handle);

	for( ;; )
	{	
		for(i=0;i<100000;i++) 												/* Loop to increase the execution time of task1 */
		{
			i=i;
		}
		
		Task1EndTime= xTaskGetTickCount();            /* Update the Task1EndTime variable with the current time. */
		
		/* Checking if the task1 missed its deadline or not, 
		 * to know if the system is schedulable or not. 
		 */
		if(Task1EndTime > Task1LastDeadline)
		{
			/* If the task1 missed its deadline -> Increase the Task1DeadlineMissesCount variable by one. */
			Task1DeadlineMissesCount++;
		}
		
		vTaskDelayUntil(&Task1LastWakeTime, TASK1_PERIOD );
		
		/* Update the Task1LastDeadline variable with the current deadline. */
		Task1LastDeadline=vTaskGetApplicationTaskItemValue(Task1Handle);
	}
}

/*-----------------------------------------------------------*/

#define TASK2_TAG			2														/* Task2 Tag. */
#define TASK2_PERIOD 	40  												/* Task2 Period. */

TaskHandle_t Task2Handle = NULL; 									/* Task2 Handler. */

volatile TickType_t Task2LastDeadline=0;					/* variable to save the last deadline of the task2. */
volatile TickType_t Task2EndTime=0;               /* variable to save the end of the execution time for the task2. */
volatile uint32_t Task2DeadlineMissesCount=0;     /* variable to save the numbers of the deadline's misses for the task2. */

/*
 * Task2TimeIn -> variable to save the time in of the task2, used to calculate the total time for task2.
 * Task2TimeOut -> variable to save the time out of the task2, used to calculate the total time for task2. 
 * Task2TimeTotal -> variable to save the total time of the task2, used to calculate the CPU load. 
 */
volatile TickType_t Task2TimeIn=0,Task2TimeOut=0,Task2TimeTotal=0;

void Task2( void * Task2_Parameters )
{
	uint32_t i=0;
	
	TickType_t Task2LastWakeTime;                   /* Local variable to save the last wake time of the task2. */
	Task2LastWakeTime =xTaskGetTickCount();         /* Initialise the Task2LastWakeTime variable with the current time. */
	
	/* Initialise the Task2LastDeadline variable with the current deadline. */
	Task2LastDeadline=vTaskGetApplicationTaskItemValue(Task2Handle);

	for( ;; )
	{	
		for(i=0;i<100000;i++) 												/* Loop to increase the execution time of task2 */
		{
			i=i;
		}
				
		vTaskGetRunTimeStats( RunTimeStatsBuff );     /* Save the run time stats of the system on RunTimeStatsBuff array. */
		xSerialPutChar('\n');                         /* Send a new line character to UART for separating between the new stats. */
		vSerialPutString(RunTimeStatsBuff,130);       /* Send the new stats to the UART. */
		
		Task2EndTime= xTaskGetTickCount();						/* Update the Task2EndTime variable with the current time. */
		
		/* Checking if the task2 missed its deadline or not, 
		 * to know if the system is schedulable or not. 
		 */		
		if(Task2EndTime > Task2LastDeadline)
		{
			/* If the task2 missed its deadline -> Increase the Task2DeadlineMissesCount variable by one. */
			Task2DeadlineMissesCount++;
		}
		
		vTaskDelayUntil(&Task2LastWakeTime, TASK2_PERIOD );
		
		/* Update the Task2LastDeadline variable with the current deadline. */
		Task2LastDeadline=vTaskGetApplicationTaskItemValue(Task2Handle);
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
	
	xTaskPeriodicCreate( Task1,  "Task1",  100, (void *)0, 1, TASK1_PERIOD, &Task1Handle); /* Creating Task1. */
	xTaskPeriodicCreate( Task2,  "Task2",  100, (void *)0, 2, TASK2_PERIOD, &Task2Handle); /* Creating Task2. */
	
	vTaskSetApplicationTaskTag(Task1Handle,(void*) TASK1_TAG); /* Setting Task1 Tag. */
	vTaskSetApplicationTaskTag(Task2Handle,(void*) TASK2_TAG); /* Setting Task2 Tag. */
	
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
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();	

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}

/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/*-----------------------------------------------------------*/

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}