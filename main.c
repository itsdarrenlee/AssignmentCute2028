/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 ******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"

#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"
#include "rgb.h"
#include "temp.h"
#include "light.h"

#include "lpc17xx_uart.h"
#include "uart2.h"

volatile uint32_t msTicks;

const uint8_t ledArray[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void SysTick_Handler(void)
{
    msTicks++;
}

uint32_t getTicks(void)
{
	return msTicks;
}

/***** Creates a sysTick Delay *****/
__INLINE static void systick_delay (uint32_t delayTicks) {
  uint32_t currentTicks;

  currentTicks = msTicks;
  while ((msTicks - currentTicks) < delayTicks);
}

/****************
 *	UART INIT
 ****************/

void pinsel_uart3(void){
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
}

void init_uart(void){
    UART_CFG_Type uartCfg;
    uartCfg.Baud_rate = 115200;
    uartCfg.Databits = UART_DATABIT_8;
    uartCfg.Parity = UART_PARITY_NONE;
    uartCfg.Stopbits = UART_STOPBIT_1;
    //pin select for uart3;
    pinsel_uart3();
    //supply power & setup working parameters for uart3
    UART_Init(LPC_UART3, &uartCfg);
    //enable transmit for uart3
    UART_TxCmd(LPC_UART3, ENABLE);
}

/****************
 *	SSP INIT
 ****************/

static void init_ssp(void)
{
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */

	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);
}

/****************
 *	I2C INIT
 ****************/
static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

/****************
 *	GPIO INIT
 ****************/
static void init_GPIO(void)
{
	// Initialize button
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;						//sw4
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1<<31, 0);


	PinCfg.Funcnum = 0;						//sw3 Config
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1<<10, 0);
}

void caretaker(void)
{
	led7seg_setChar('{', FALSE); // clear 7 segment display
	oled_clearScreen(OLED_COLOR_BLACK); // clear OLED display
	GPIO_ClearValue( 0, (1<<26) ); // clear blue LED
	GPIO_ClearValue( 2, (1<<0) ); // clear red LED
	light_shutdown(); // shutdown light sensor
}



void EINT3_IRQHandler(void)
{

	if ((LPC_GPIOINT -> IO2IntStatF>>10) & 0x01)
	{
		printf("Sw3 is on interrupt \n");
		LPC_GPIOINT -> IO2IntClr |=  1 << 10;
	}
	else if (light_getIrqStatus())
	{
		printf("Light is on interrupt \n");
		light_clearIrqStatus();
		LPC_GPIOINT -> IO2IntClr |=  1 << 5;
	}
	else if ((LPC_GPIOINT -> IO0IntStatF>>24) & 0x01)
	{
		printf("Rotary is on interrupt \n");
		LPC_GPIOINT -> IO0IntClr |=  1 << 24;
	}
}

void init(void)
{
	init_i2c();
	init_ssp();
	init_GPIO();
	acc_init();
	oled_init();
	led7seg_init ();
	rgb_init ();
	light_init();
	init_uart();
}

void uartSendMessage(uint8_t msg)
{
	UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
}

static char* msg = NULL;

int main (void) {

	init();
    //msg = "Entering MONITOR mode\r\n";
	//UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
    //void uartSendMessage(uint8_t msg);


    LPC_GPIOINT -> IO2IntEnF |= 1<<10;
    LPC_GPIOINT -> IO2IntEnF |= 1<<5;
    LPC_GPIOINT -> IO0IntEnF |= 1<<24;

    light_setHiThreshold(700);
    light_setLoThreshold(50);
    light_clearIrqStatus();
    NVIC_ClearPendingIRQ(EINT3_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);

    SysTick_Config(SystemCoreClock/1000);

    volatile static int i = 0 ;
    while (1)
    {
    	systick_delay(1000);
    	led7seg_setChar((ledArray[i%16]), FALSE);
    	i++;



    	/*
    	switch (mode) {
				case CARETAKER:
					printf("Caretaker\n");
					caretaker();
					break;

				case MONITOR:
					printf("Monitor\n")
					break;

				default:
					printf("Not defined yet\n");
					break;
			}
			*/

    }
}









    	/*acc_read(&x,&y,&z);
    	lightvalue = light_read();
        my_temp_value = temp_read();
        printf("%2.2f degrees \n", my_temp_value/10.0);
        printf("%d lux \n", lightvalue);
        printf("x=%d, y=%d, z=%d \n", x,y,z);
        if (lightvalue < 200)
        {
        	pca9532_setLeds(0x0000, 0xFFFF);
        }
        else if ((lightvalue > 200) && (lightvalue < 300))
        {
        	pca9532_setLeds(0xFF00, 0xFFFF);
        }
        else if ((lightvalue > 300) && (lightvalue < 400))
        {
        	pca9532_setLeds(0xFFF0, 0xFFFF);
        }
        else
        {
        	pca9532_setLeds(0xFFFF, 0xFFFF);
        }


        sprintf(display_acc, "MONITOR");
        oled_putString(5,5,display_acc,OLED_COLOR_WHITE, OLED_COLOR_BLACK);*/









void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
