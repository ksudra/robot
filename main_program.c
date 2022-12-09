/*
 * FreeRTOS Kernel V10.1.1
 * Copyright (C) 2018 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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

// Author:      Mohd A. Zainol
// Date:        1 Oct 2018
// Chip:        MSP432P401R LaunchPad Development Kit (MSP-EXP432P401R) for TI-RSLK
// File:        main_program.c
// Function:    The main function of our code in FreeRTOS

/* Standard includes. */
#include <stdio.h>

/* Kernel includes. */
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* TI includes. */
#include "gpio.h"

/* ARM Cortex */
#include <stdint.h>
#include "msp.h"
#include "SysTick.h"
#include "inc/CortexM.h"

# include "../inc/songFile.h"
# include "../inc/dcMotor.h"
# include "../inc/bumpSwitch.h"
# include "../inc/outputLED.h"
# include "../inc/SysTick.h"

#define SW1IN ((*((volatile uint8_t *)(0x42098004)))^1)
#define SW2IN ((*((volatile uint8_t *)(0x42098010)))^1)
uint8_t bumpSwitch_status;
SemaphoreHandle_t xBSemaphore;

static void Switch_Init(void);
static void taskMasterThread( void *pvParameters );
static void taskPlaySong( void *pvParameters );
static void taskReadInputSwitch( void *pvParameters );

static void taskBumpSwitch( void *pvParameters );
static void taskdcMotor( void *pvParameters );
static void taskDisplayOutputLED( void *pvParameters);


static void Port4_Init(void);
static void BumpEdgeTrigger_Init(void);
static void taskdcMotor_interrupt(void *pvParameters);
static void taskInterrupt(void *pvParameters);
static void outputLED_ISR(unsigned char bumpSwitch_status);
/*
 * Called by main() to create the main program application
 */
void main_program( void );

/*
 * The configuration of clocks for frequency.
 */
static void prvConfigureClocks( void );

xTaskHandle taskHandle_BlinkRedLED;
xTaskHandle taskHandle_BumpSwitch;
xTaskHandle taskHandle_PlaySong;
xTaskHandle taskHandle_dcMotor;
xTaskHandle taskHandle_InputSwitch;
xTaskHandle taskHandle_OutputLED;

xTaskHandle taskHandle_dcMotorInterrupt;
xTaskHandle taskHandle_Interrupt;

int mode = 1;
uint8_t stop = 0;

void main_program( void )
{
    // initialise the clock configuration


    prvConfigureClocks();

    Switch_Init();
    SysTick_Init();



    if(mode == 0) {
        xTaskCreate(taskMasterThread, "TaskT", 128, NULL, 2, &taskHandle_BlinkRedLED);
        xTaskCreate(taskPlaySong, "TaskS", 128, NULL,  1, &taskHandle_PlaySong);
        xTaskCreate(taskReadInputSwitch, "TaskR",  128, NULL, 1, &taskHandle_InputSwitch);
        xTaskCreate(taskBumpSwitch, "TaskB", 128, NULL, 1, &taskHandle_BumpSwitch);
        xTaskCreate(taskdcMotor,  "TaskM", 128, NULL, 1, &taskHandle_dcMotor);
        xTaskCreate(taskDisplayOutputLED, "TaskD", 128, NULL, 1, &taskHandle_OutputLED);
    } else if(mode == 1) {
        xBSemaphore = xSemaphoreCreateBinary();
        Port4_Init();
        BumpEdgeTrigger_Init();

        xTaskCreate(taskMasterThread, "TaskT", 128, NULL, 2, &taskHandle_BlinkRedLED);
        xTaskCreate(taskPlaySong, "TaskS", 128, NULL,  1, &taskHandle_PlaySong);
        xTaskCreate(taskReadInputSwitch, "TaskR",  128, NULL, 1, &taskHandle_InputSwitch);
        xTaskCreate(taskdcMotor_interrupt, "TaskM", 128, NULL, 1, &taskHandle_dcMotorInterrupt);
        xTaskCreate(taskInterrupt, "TaskI", 128, NULL, 1, &taskHandle_dcMotorInterrupt);
    }

    vTaskStartScheduler();


    /* INFO: If everything is fine, the scheduler will now be running,
    and the following line will never be reached.  If the following line
    does execute, then there was insufficient FreeRTOS heap memory
    available for the idle and/or timer tasks to be created. See the
    memory management section on the FreeRTOS web site for more details. */
    for( ;; );
}
/*-----------------------------------------------------------------*/
/*------------------- FreeRTOS configuration ----------------------*/
/*-------------   DO NOT MODIFY ANYTHING BELOW HERE   -------------*/
/*-----------------------------------------------------------------*/
// The configuration clock to be used for the board
static void prvConfigureClocks( void )
{
    // Set Flash wait state for high clock frequency
    FlashCtl_setWaitState( FLASH_BANK0, 2 );
    FlashCtl_setWaitState( FLASH_BANK1, 2 );

    // This clock configuration uses maximum frequency.
    // Maximum frequency also needs more voltage.

    // From the datasheet: For AM_LDO_VCORE1 and AM_DCDC_VCORE1 modes,
    // the maximum CPU operating frequency is 48 MHz
    // and maximum input clock frequency for peripherals is 24 MHz.
    PCM_setCoreVoltageLevel( PCM_VCORE1 );
    CS_setDCOCenteredFrequency( CS_DCO_FREQUENCY_48 );
    CS_initClockSignal( CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
}

// The sleep processing for MSP432 board
void vPreSleepProcessing( uint32_t ulExpectedIdleTime ){}

#if( configCREATE_SIMPLE_TICKLESS_DEMO == 1 )
    void vApplicationTickHook( void )
    {
        /* This function will be called by each tick interrupt if
        configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
        added here, but the tick hook is called from an interrupt context, so
        code must not attempt to block, and only the interrupt safe FreeRTOS API
        functions can be used (those that end in FromISR()). */
        /* Only the full demo uses the tick hook so there is no code is
        executed here. */
    }
#endif
/*-----------------------------------------------------------------*/
/*-------------   DO NOT MODIFY ANYTHING ABOVE HERE   -------------*/
/*--------------------------- END ---------------------------------*/
/*-----------------------------------------------------------------*/

static void Switch_Init(void){
    // negative logic built-in Button 1 connected to P1.1
    // negative logic built-in Button 2 connected to P1.4
    P1->SEL0 &= ~0x12;
    P1->SEL1 &= ~0x12;      // configure P1.4 and P1.1 as GPIO
    P1->DIR &= ~0x12;       // make P1.4 and P1.1 in
    P1->REN |= 0x12;        // enable pull resistors on P1.4 and P1.1
    P1->OUT |= 0x12;        // P1.4 and P1.1 are pull-up
}

static void taskMasterThread( void *pvParameters )
{
    int i;

    ColorLED_Init();
    RedLED_Init();

    while(!SW2IN){                  // Wait for SW2 switch
        for (i=0; i<1000000; i++);  // Wait here waiting for command
        REDLED = !REDLED;           // The red LED is blinking
    }

    REDLED = 0;

    vTaskSuspend(taskHandle_BlinkRedLED);
}

static void taskPlaySong( void *pvParameters ){
    init_song_pwm();
    while (1){
        play_song();
    }
}

// polling
static void taskBumpSwitch( void *pvParameters ){
    BumpSwitch_Init();
    for( ;; )
    {
        bumpSwitch_status = Bump_Read_Input();
    }
}

static void taskDisplayOutputLED (void *pvParameters){
    for( ;; )
    {
        outputLED_response(bumpSwitch_status);
    }
}

static void taskdcMotor( void *pvParameters ){
    dcMotor_Init();
    while(1){
        dcMotor_response(bumpSwitch_status);
    }
}

static void taskReadInputSwitch( void *pvParameters ){

    char i_SW1=0;
    int i;

    for( ;; )
    {
        if (SW1IN == 1) {
            i_SW1 ^= 1;                 // toggle the variable i_SW1
            for (i=0; i<1000000; i++);  // this waiting loop is used
                                        // to prevent the switch bounce.
        }

        if (i_SW1 == 1) {
            REDLED = 1;     // turn on the red LED
            vTaskSuspend(taskHandle_PlaySong);
        }
        else {
            REDLED = 0;     // turn off the red LED
            vTaskResume(taskHandle_PlaySong);
        }

    }
}

//interrupt
void BumpEdgeTrigger_Init(void){
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;      // configure as GPIO
    P4->DIR &= ~0xED;       // make in
    P4->REN |= 0xED;        // enable pull resistors
    P4->OUT |= 0xED;        // pull-up
    P4->IES |= 0xED;        // falling edge event
    P4->IFG &= ~0xED;       // clear flag
    P4->IE |= 0xED;         // arm the interrupt
    // priority 2 on port4
    NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF)|0x00D00000; //Numerical Priority 6
    // enable interrupt 38 in NVIC on port4
    NVIC->ISER[1] = 0x00000040;
}

void Port4_Init(void){
    P4->SEL0 &= ~0xED;
    P4->SEL1 &= ~0xED;      // configure as GPIO
    P4->DIR &= ~0xED;       // set as input
    P4->REN |= 0xED;        // enable pull resistors
    P4->OUT |= 0xED;        // set xx are pull-up
    P4->IES |= 0xED;      // falling edge event
}

void PORT4_IRQHandler(void){
    bumpSwitch_status = P4->IV;
    BaseType_t xHigher;
    xHigher = pdFALSE;
    P4->IFG &= ~0xED;
    xSemaphoreGiveFromISR(xBSemaphore, &xHigher);
}

void outputLED_ISR(unsigned char bumpSwitch_status){
    int i;
    switch(bumpSwitch_status){
      case 0x02: // Bump switch 1
          Port2_Output2(SKYBLUE);
        break;
      case 0x06:// Bump switch 2
          Port2_Output2(RED);
        break;
      case 0x08: // Bump switch 3
          Port2_Output2(PINK);
        break;
      case 0x0C: // Bump switch 4
          Port2_Output2(YELLOW);
        break;
      case 0x0E: // Bump switch 5
          Port2_Output2(GREEN);
        break;
      case 0x10: // Bump switch 6
          Port2_Output2(BLUE);
        break;
      case 0xED: // neither switch pressed
          Port2_Output2(COLOUROFF);
        break;
      default:
          Port2_Output2(WHITE);
    }
    for (i=0; i<100000; i++);
    Port2_Output2(COLOUROFF);
}

static void taskInterrupt(void *pvParamters){
    EnableInterrupts();
    for( ;; )
    {
        if(xBSemaphore != NULL) {
            if(xSemaphoreTake(xBSemaphore, portMAX_DELAY) == pdTRUE) {
                vTaskSuspend(taskHandle_dcMotorInterrupt);
                dcMotor_response_interrupt(bumpSwitch_status);
                vTaskDelay(1);
                outputLED_ISR(bumpSwitch_status);
                vTaskResume(taskHandle_dcMotorInterrupt);
                vTaskDelay(1);
            }
        }

    }
}

static void taskdcMotor_interrupt(void *pvParameters){
    dcMotor_Init();
    for( ;; ) {
        dcMotor_Forward(300, 10);
    }
}
