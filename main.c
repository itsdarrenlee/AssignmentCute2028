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

#define TEMP_HIGH_WARNING 35
#define LIGHT_LOW_WARNING 50
#define ACCEL_LIMIT 10

volatile uint32_t msTicks = 0;

int switchCounter = 0;
uint32_t endTime, startTime, initialTime;

volatile uint8_t blink_blue = 0, blink_red = 0;
volatile bool monitorStatus = false; // default mode is caretaker mode (false)

const uint8_t ledArray[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

int8_t x, y, z;
int32_t xoff, yoff, zoff;

typedef struct envVariables
{
	float currentTemp;
	int currentLight;
	int8_t accelX, accelY, accelZ;
} env;

void SysTick_Handler(void)
{
    msTicks++;
}

uint32_t getTicks(void)
{
    return msTicks;
}

/****************
 *  Time Comparator Function
 ****************/
uint32_t timeCompare (uint32_t ticks, uint32_t delaydiff)
{
    delaydiff = delaydiff * 1000;
    return (getTicks() - ticks) >= delaydiff;
}

/****************
 *  UART INIT
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
 *  SSP INIT
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
 *  I2C INIT
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
 *  GPIO INIT
 ****************/
static void init_GPIO(void)
{
    // Initialize Switch 4 using default config
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 0;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 31;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(1, 1<<31, 0);

    // Initialize Switch 3 using EINT0 for port 2 pin 10
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 10;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(2, 1 << 10, 0);

    // Settings for EINT0 Interrupt
	LPC_SC->EXTINT = 1;
	LPC_SC->EXTMODE |= 1<<0;
	LPC_SC->EXTPOLAR &= ~(1 << 0);
}

/****************
 *  INTERUPT INIT
 ****************/

void extInteruptInit(void)
{
    NVIC_SetPriority(SysTick_IRQn,1);
    NVIC_SetPriority(EINT0_IRQn,2);
    NVIC_SetPriority(EINT3_IRQn,3);

    NVIC_ClearPendingIRQ(EINT0_IRQn); // configure sw3 to use eint0 interrupt
    NVIC_ClearPendingIRQ(EINT3_IRQn);

    NVIC_EnableIRQ(EINT0_IRQn);
    NVIC_EnableIRQ(EINT3_IRQn);
}

volatile int8_t currentTemp;

void EINT3_IRQHandler(void)
{
	// using EINT3 for interrupt

	/*
	// temperature rising/falling edge
	if (((LPC_GPIOINT ->IO0IntStatF >> 2) & 0x1) || ((LPC_GPIOINT ->IO0IntStatR >> 2) & 0x1))
	{
		currentTemp = ((GPIO_ReadValue(0) & (1 << 2)) != 0);
		printf("in eint3, currentTemp is %d\n", currentTemp);

		LPC_GPIOINT -> IO0IntClr =  1 << 2;
	}*/
}


void EINT0_IRQHandler(void)
{
    // using EINT0 for interrupt
	if(monitorStatus == true)       //monitor mode
	{
		if(switchCounter == 0)
		{
			startTime = getTicks();
			switchCounter = 1;
			printf("1st Press, time = %d \n", startTime);
		}
		else
		{
			endTime = getTicks();
			switchCounter = 2;
			printf("2nd Press, time = %d \n", endTime);
		}

		if (endTime-startTime < 1000 && switchCounter == 2)
		{
			printf("leaving monitor mode \n");
			switchCounter = 0;
			monitorStatus = false;

			unsigned char caretakermodeMsg[] = "Leaving Monitor mode\r\n";
			UART_SendString(LPC_UART3, caretakermodeMsg);
		}
	}
    LPC_SC->EXTINT = (1<<0);
}


void init(void)
{
    SysTick_Config(SystemCoreClock/1000);
    init_i2c();
    init_ssp();
    init_GPIO();

    acc_init();
    acc_read(&x, &y, &z);
    xoff = 0-x;
    yoff = 0-y;
    zoff = 0-z;

    //printf("x is %d, y is %d, z is %d\n",x,y,z);
    //printf("xoff is %d, yoff is %d, zoff is %d\n",xoff,yoff,zoff);

    oled_init();
    led7seg_init ();
    rgb_init ();
    light_init();
    light_enable();
    temp_init(getTicks);
    init_uart();
    acc_init();

    light_setHiThreshold(700);
    light_setLoThreshold(50);
    light_clearIrqStatus();
}

void caretakerMode(int caretakerFlag)
{
	if (caretakerFlag == true) // default true
    {
        unsigned char caretakerMsg[] = "Entering CARETAKER mode\r\n";
        UART_SendString(LPC_UART3, caretakerMsg);
    }
    led7seg_setChar('}', FALSE); // clear 7 segment display
    oled_clearScreen(OLED_COLOR_BLACK); // clear OLED display
    blink_red = 0;  // Clear blink red led display
    blink_blue = 0; // Clear blink blue led display
    GPIO_ClearValue( 0, (1<<26) ); // clear blue LED
    GPIO_ClearValue( 2, (1<<0) ); // clear red LED
}

void sevenSegmentOut(int charCounter)
{
    led7seg_setChar((ledArray[charCounter%16]), FALSE);
}

/** initialization for the monitor functions **/
int monitorFlag = true;
unsigned char monitorOled[] = "MONITOR";

char floatArray[50] = {};
char integerArray[50] = {};

unsigned char nameMsg[64] = "";
int msgCount = 0;

/*************************************************/

void sampleEnv(env *ptr)
{
    ptr->currentTemp = temp_read()/10.0;
    ptr->currentLight = light_read();

    acc_read(&x, &y, &z);
    x = x+xoff;
	y = y+yoff;
	z = z+zoff;

    ptr->accelX = x;
    ptr->accelY = y;
    ptr->accelZ = z;

}

void oledDisplay(env *ptr)
{
	//printf("accel x is %d, accel y is %d, accel z is %d\n",ptr->accelX,ptr->accelY,ptr->accelZ);
	//printf("x is %d, y is %d, z is %d\n",x,y,z);

	snprintf (floatArray, sizeof(floatArray), "Temp: %2.2fdegC", ptr->currentTemp); // print temperature
    oled_putString (1, 20, floatArray, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    snprintf (integerArray, sizeof(integerArray), "light: %d lux", ptr->currentLight); // print current light in lux
    oled_putString (1, 30, integerArray, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    snprintf (integerArray, sizeof(integerArray), "X:%d, Y:%d", ptr->accelX, ptr->accelY); // print x and y accelerometer values
    oled_putString (1, 40, integerArray, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    snprintf (integerArray, sizeof(integerArray), "Z:%d", ptr->accelZ); // print z accelerometer value on a new line
    oled_putString (1, 50, integerArray, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    return;
}

void SendEnvVariables(env *ptr)
{
    snprintf (nameMsg, sizeof(nameMsg), "%03d_-_T%05.1f_L%05d_AX%05d_AY%05d_AZ%05d\r\n", msgCount, ptr->currentTemp, ptr->currentLight, ptr->accelX, ptr->accelY, ptr->accelZ);
    UART_SendString(LPC_UART3, nameMsg);
    msgCount += 1;
}

void monitorMode(int monitorFlag, env *ptr)
{
	if (monitorFlag == true)
    {
        unsigned char monitorMsg[] = "Entering MONITOR mode\r\n";
        UART_SendString(LPC_UART3, monitorMsg);
    }

	oled_putString (1, 10, monitorOled, OLED_COLOR_WHITE, OLED_COLOR_BLACK); // print MONITOR on oled
}

void displayOnOled(env *ptr, bool oledFlag, int charCounter)
{
	if ((charCounter%16 == 6
	|| charCounter%16 == 11
	|| charCounter%16 == 0)
	&& charCounter != 0
	&& oledFlag == false)
	{
		oledDisplay(ptr);
		oledFlag = true;
	}
}

void fireOrDarkness(env *ptr)
{
	uint8_t x = ptr->accelX;
	uint8_t y = ptr->accelY;
	uint8_t z = ptr->accelZ;

	printf("current temp is %f\n", ptr->currentTemp);
	if ((ptr->currentTemp) >= TEMP_HIGH_WARNING)
	{
		blink_red = 1;
		unsigned char FireMsg[] = "Fire Detected\r\n";
		UART_SendString(LPC_UART3, FireMsg);
	}

	if (((ptr->currentLight) < LIGHT_LOW_WARNING) && (sqrt(x*x+y*y+z*z) >= ACCEL_LIMIT))
	{
		blink_blue = 1;         //raise blink blue flag when light intensity is low and movement is detected
		unsigned char DarkMovementMsg[] = "Movement in Darkness Detected\r\n";
		UART_SendString(LPC_UART3, DarkMovementMsg);
	}
}

bool mode, oledFlag, sendFlag;
bool caretakerFlag = true;

int main (void)
{
    init();
    extInteruptInit();

    //LPC_GPIOINT -> IO2IntEnF |= 1<<10;



    volatile static int charCounter = 0;

    int sevenSegTicks = getTicks();
    int rgbBothTicks = getTicks();

    env envValues;
    env *ptr = &envValues;

    /*
    GPIO_SetDir(0, (1 << 2), 0);
    LPC_GPIOINT -> IO0IntEnF |= 1<<2; // if temp falls
    LPC_GPIOINT -> IO0IntEnR |= 1<<2; // if temp rises
    LPC_GPIOINT -> IO0IntClr =  1<<2; // clear the interrupt
    currentTemp = ((GPIO_ReadValue(0) & (1 << 2)) != 0); // update current temp
    printf("in main, current temp is %d\n", currentTemp);
	*/

    oled_clearScreen(OLED_COLOR_BLACK);

    while (1)
    {
		int SW_MONITOR = (GPIO_ReadValue(1) >> 31) & 0x01; // polling sw4

        if (SW_MONITOR != true)
        monitorStatus = true; // switch is default high, so a low would indicate a press

        if (monitorStatus == true)
        mode = 0; // change to monitor mode if detect press
        else
        mode = 1; // else remain in caretaker

        initialTime = getTicks();
		if (switchCounter == 1 && (initialTime - startTime > 1000))
			switchCounter = 0;
		else if (switchCounter == 2 && (initialTime - endTime > 1000))
			switchCounter = 0;

        switch (mode) {
            case 1:

                caretakerMode(caretakerFlag); // start caretaker mode
                caretakerFlag = false; // after sending first 'entering caretaker mode', stop sending

                monitorFlag = true;
                charCounter = 0;

                break;

            case 0:
                monitorMode(monitorFlag, ptr); // start monitor mode
                fireOrDarkness(ptr);
                sampleEnv(ptr);
                displayOnOled(ptr, oledFlag, charCounter);


                if (charCounter%16 == 15 && sendFlag == false)
				{
					SendEnvVariables(ptr);
					sendFlag = true;
				}

                if (timeCompare(sevenSegTicks, 1))
				{
					sevenSegTicks = getTicks();
					sevenSegmentOut(charCounter++);

					oledFlag = false; sendFlag = false;
				}


                if (blink_red == 1 && blink_blue == 1)
				{
					if (timeCompare(rgbBothTicks, 0.5) )
					{
						GPIO_ClearValue( 0, (1<<26)); // Clear blue
						GPIO_SetValue( 2, (1<<0)); // Red first
					}
					if (timeCompare(rgbBothTicks, 1) )
					{

						GPIO_ClearValue( 2,(1<<0)); // Clear red
						GPIO_SetValue( 0, (1<<26)); // Set blue
						rgbBothTicks = getTicks();
					}
				}

				else if (blink_red == 1 && blink_blue == 0)
				{
					GPIO_ClearValue( 0, (1<<26)); // Clear blue
					GPIO_SetValue( 2, (1<<0)); // set red

					if (timeCompare(rgbBothTicks, 1))
					{
						GPIO_ClearValue( 2,(1<<0)); //Clear red
						rgbBothTicks = getTicks();
					}
				}

				else if (blink_blue == 1 && blink_red == 0)
				{
					GPIO_ClearValue( 2, (1<<0)); // clear red
					GPIO_SetValue( 0, (1<<26)); // set blue

					if (timeCompare(rgbBothTicks, 1))
					{
					   GPIO_ClearValue( 0,(1<<26)); //clear blue
					   rgbBothTicks = getTicks();
					}
				}

				caretakerFlag = true;
                monitorFlag = false; // after sending first 'entering monitor mode', stop sending
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
