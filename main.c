/*****************************************************************************
 *
 *
 *   Final Project EE2028 - Lee Weihan Darren/Darren Lee Ting Jue
 *
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

/****************
 *	SysTick Delay
 ****************/

__INLINE static void systick_delay (uint32_t delayTicks) {
  uint32_t currentTicks;

  currentTicks = msTicks;
  while ((msTicks - currentTicks) < delayTicks);
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
	// Initialize Switch 4
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 0;						//sw4
	PinCfg.Portnum = 1;
	PinCfg.Pinnum = 31;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(1, 1<<31, 0);

	// Initialize Switch 3
	PinCfg.Funcnum = 0;						//sw3
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, 1<<10, 0);
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
 *	INTERUPT INIT
 ****************/
void extInteruptInit(void)
{
	NVIC_SetPriority(SysTick_IRQn,1);
	
	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_SetPriority(EINT3_IRQn,2);
	NVIC_EnableIRQ(EINT3_IRQn);

	LPC_SC->EXTINT = 1;  /* Clear Interrupt Flag */
	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_SetPriority(EINT1_IRQn,3);
	NVIC_EnableIRQ(EINT0_IRQn);

	//https://www.exploreembedded.com/wiki/LPC1768:_External_Interrupts
}

void EINT3_IRQHandler(void)
{
	if ((LPC_GPIOINT -> IO2IntStatF>>10) & 0x01)
	{
		printf("Sw3 is on interrupt \n");
	}
	LPC_GPIOINT -> IO2IntClr |=  1 << 10;
}

void EINT0_IRQHandler(void)
{
	LPC_SC->EXTINT = (1<<0);  /* Clear Interrupt Flag */
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

	light_setHiThreshold(700);
	light_setLoThreshold(50);

	NVIC_SetPriorityGrouping(5); // IRR width for lpc1769 is 5 bits
}

void caretakerMode(void)
{
	uint8_t line[] = "Entering CARETAKER mode\r\n";
	UART_SendString(LPC_UART3, line);
	led7seg_setChar('{', FALSE); // clear 7 segment display
	oled_clearScreen(OLED_COLOR_BLACK); // clear OLED display
	GPIO_ClearValue( 0, (1<<26) ); // clear blue LED
	GPIO_ClearValue( 2, (1<<0) ); // clear red LED
	light_shutdown(); // shutdown light sensor
}

void monitorMode(void)
{
	sevenSegmentOut();
}

void sevenSegmentOut(void)
{
	volatile static int i = 0 ;
	led7seg_setChar((ledArray[i%16]), FALSE);
	systick_delay(1000);
	i++;
}


int main (void)
{
	init();
	SysTick_Config(SystemCoreClock/1000);
	caretakerMode();

	bool monitorFlag = 1;
	bool mode;

    while (1)
    {
    	int SW_MONITOR = (GPIO_ReadValue(1) >> 31) & 0x01; // polling sw4

    	if (SW_MONITOR == 0)
    	monitorFlag = 0;

    	if (monitorFlag == 0)
    	mode = 0;
    	else
    	mode = 1;


    	switch (mode) {
			case 1:
				caretakerMode(); // start caretaker mode
				break;

			case 0:
				monitorMode(); // start monitor mode
				break;
		}
    }
}


void check_failed(uint8_t *file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while(1);
}
