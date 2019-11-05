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

#define LIGHT_UPPER_THRESHOLD 700
#define LIGHT_LOWER_THRESHOLD 50

#define ACCEL_LIMIT 10

volatile uint32_t msTicks = 0;
volatile uint8_t blink_blue = 0, blink_red = 0;
volatile bool monitorStatus = false;

const uint8_t ledArray[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

int8_t x, y, z;
int32_t xoff, yoff, zoff;
uint32_t endTime, startTime, initialTime;

bool lightFlag = false;
bool mode, oledFlag, sendFlag;
bool caretakerFlag = true;

int switchCounter = 0;

/** initialization for the monitor mode functions **/
int monitorFlag = true;
unsigned char monitorOled[] = "MONITOR";
char floatArray[50] = {};
char integerArray[50] = {};
char nameMsg[64] = "";
int msgCount = 0;
/*************************************************/

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

/*********************************************************
 * 			Time Comparator Function
 *
 * 	This function takes an 2 inputs, ticks, and a user
 * 	defined value, delaydiff. It will compare difference
 * 	in the current tick value with the value in ticks.
 *
 * 	If the delay difference is reached, it will return
 * 	back to main.
 *
 * 	Inputs: ticks (unsigned 32bit integer)
 * 			delaydiff (unsigned 32bit value, in seconds)
 ********************************************************/
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
    PINSEL_CFG_Type PinCfg; // Configure Pin corresponding to specified parameters passed in the PinCfg.
    PinCfg.Funcnum = 0;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 31;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(1, 1<<31, 0);

	// settings for light interrupt
	PinCfg.Funcnum = 0;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 5;
	PINSEL_ConfigPin(&PinCfg);
	GPIO_SetDir(2, (1 << 5), 0);
}

static void eint0_init(void)
{
    // Initialize Switch 3 using EINT0 for port 2 pin 10
	PINSEL_CFG_Type PinCfg; // Configure Pin corresponding to specified parameters passed in the PinCfg.
    PinCfg.Funcnum = 1;
    PinCfg.OpenDrain = 0;
    PinCfg.Pinmode = 0;
    PinCfg.Portnum = 2;
    PinCfg.Pinnum = 10;
    PINSEL_ConfigPin(&PinCfg);
    GPIO_SetDir(2, 1 << 10, 0);

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


/*********************************************************
 * 			EINT3 Handler Function
 *
 * 	In main, the interupt is initialized via GPIO, port
 * 	2 pin 5: <LPC_GPIOINT->IO2IntEnF |= 1 << 5;>
 *
 * 	This interupt will trigger if the current light lux falls
 * 	below the user defined light threshold value, currently
 * 	set at 50 lux. This is defined as a falling edge.
 *
 * 	Upon triggered, the interupt will clear the interupt
 * 	of the handler, clear the interupt status of the light,
 * 	and set a global flag, lightFlag to be true.
 *
 * 	Inputs: None
 ********************************************************/
void EINT3_IRQHandler(void)
{
	// EINT3 for light falling below threshold
	if ((LPC_GPIOINT ->IO2IntStatF >> 5) & 0x1)
	{
		LPC_GPIOINT->IO2IntClr |= (1 << 5);
		light_clearIrqStatus();
		lightFlag = true; // interrupt based update of light sensor
	}
}

/*********************************************************
 * 			EINT0 Handler Function
 *
 * 	The interupt is initialized in eint0_init();
 *
 * 	The interupt will trigger if the switch 3 is pressed.
 *
 * 	Switchcounter will start from 0. Upon triggered
 * 	by a switch press, the interupt will change
 * 	switchcounter to 1, and obtain the current time via
 * 	getTicks(), storing it in variable startTime.
 *
 * 	Similarly, if a second press is done, the interupt
 * 	will trigger, but switchcounter being at 1 will activate
 * 	the 'else' condition, obtaining currenttime via
 * 	getTicks(), and changing switchcounter value to 2
 * 	yet again.
 *
 * 	For both points, if endTime-startTime < 1000 &&
 * 	switchCounter == 2, monitorstatus will become
 * 	false and it will switch to caretaker mode. A message
 * 	will also be sent via UART to NAME.
 *
 * 	In main, switchcounter is continously being reset
 * 	to value of 0, if the presses are too long in
 * 	between.
 *
 * 	Inputs: None
 ********************************************************/
void EINT0_IRQHandler(void)
{
    // using EINT0 for interrupt
	if(monitorStatus == true)       //monitor mode
	{
		if(switchCounter == 0)
		{
			startTime = getTicks();
			switchCounter = 1;
		}
		else
		{
			endTime = getTicks();
			switchCounter = 2;
		}

		if (endTime-startTime < 1000 && switchCounter == 2)
		{
			switchCounter = 0;
			monitorStatus = false;

			unsigned char caretakermodeMsg[] = "Leaving Monitor mode\r\n";
			UART_SendString(LPC_UART3, caretakermodeMsg);
		}
	}
    LPC_SC->EXTINT = (1<<0);
}
/*********************************************************
 * 					init function
 *
 * 	This function initializes everything that is required
 * 	from systick_config to all the baseboard/on chip
 * 	peripherals in their various libraries.
 *
 * 	It sets the light low threshold to 50 and high to 700
 * 	respectively. The current interupt status of the light
 * 	if any, will be reset.
 *
 * 	Calculation of the current accelerometer values, will
 * 	be stored in global variables x, y, z and the respective
 * 	offsets will be calculated from there.
 *
 * 	Inputs: None
 ********************************************************/
void init(void)
{
    SysTick_Config(SystemCoreClock/1000);
    init_i2c();
    init_ssp();
    init_GPIO();
    eint0_init();

    oled_init();
    led7seg_init ();
    rgb_init ();
    light_init();
    light_enable();
    temp_init(getTicks);
    init_uart();
    acc_init();

    light_setHiThreshold(LIGHT_UPPER_THRESHOLD);
    light_setLoThreshold(LIGHT_LOWER_THRESHOLD);
    light_clearIrqStatus();

    acc_init();
	acc_read(&x, &y, &z); // obtain initial x,y,z values
	xoff = 0-x; // calculate x initial offsets
	yoff = 0-y; // calculate y initial offsets
	zoff = 0-z; // calculate z initial offsets
}

/*********************************************************
 * 					Caretaker function
 *
 *	This function takes in a single input, caretaker flag
 *	to decide if it is the first run of the program. If
 *	flag is true, ie. the program is running for the first
 *	time/the program switched over from monitor mode,
 *	a message is sent to NAME to indicate so.
 *
 *	The 7 segment display, OLED, led blink statuses,
 *	and gpio led outputs are cleared.
 *
 * 	Inputs: bool caretakerFlag
 ********************************************************/

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

/*********************************************************
 * 				sevenSegmentOut function
 *
 *	This function will display on the 7 segment display,
 *	the current charcounter value as defined by the array
 *	ledArray. There is no logic here, all logic is contained
 *	in the main program.
 *
 * 	Inputs: char charCounter
 ********************************************************/

void sevenSegmentOut(int charCounter)
{
    led7seg_setChar((ledArray[charCounter%16]), FALSE);
}

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

/*********************************************************
 * 				oled display function
 *
 *	This program will obtain values from the pointer to
 *	struct, ptr, and output values onto the OLED display
 *	at 5, A and F intervals.
 *
 * 	Inputs: env * ptr
 ********************************************************/

void oledDisplay(env *ptr)
{
	snprintf (floatArray, sizeof(floatArray), "Temp: %2.2fdegC", ptr->currentTemp); // print temperature
    oled_putString (1, 20, floatArray, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    snprintf (integerArray, sizeof(integerArray), "light: %d lux", ptr->currentLight); // print current light in lux
    oled_putString (1, 30, integerArray, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    snprintf (integerArray, sizeof(integerArray), "X:%d, Y:%d", ptr->accelX, ptr->accelY); // print x and y accelerometer values
    oled_putString (1, 40, integerArray, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

    snprintf (integerArray, sizeof(integerArray), "Z:%d", ptr->accelZ); // print z accelerometer value on a new line
    oled_putString (1, 50, integerArray, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

/*********************************************************
 * 				sendEnvVariables function
 *
 *	This program will obtain values from the pointer to
 *	struct, ptr, and send values to NAME via UART. A
 *	counter is used to increment the msgCount
 *
 * 	Inputs: env * ptr
 ********************************************************/
void SendEnvVariables(env *ptr)
{
    snprintf (nameMsg, sizeof(nameMsg), "%03d_-_T%05.1f_L%05d_AX%05d_AY%05d_AZ%05d\r\n", msgCount, ptr->currentTemp, ptr->currentLight, ptr->accelX, ptr->accelY, ptr->accelZ);
    UART_SendString(LPC_UART3, nameMsg);
    msgCount += 1;
}

/*********************************************************
 * 				monitormode function
 *
 *	This function takes in a single input, monitor flag
 *	to decide if it is the first run of the program. If
 *	flag is true, ie. the program is running for the first
 *	time/the program switched over from caretaker mode,
 *	a message is sent to NAME to indicate so.
 *
 *	The oled display is set to continuously display
 *	'MONITOR' at the top if monitor mode is active
 *
 * 	Inputs: monitorflag
 ********************************************************/

void monitorMode(int monitorFlag)
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
	int8_t x = ptr->accelX;
	int8_t y = ptr->accelY;
	int8_t z = ptr->accelZ;
	int8_t accelerationCal = sqrt((x*x)+(y*y)+(z*z));

	if ((ptr->currentTemp) >= TEMP_HIGH_WARNING)
	{
		blink_red = 1;
		unsigned char FireMsg[] = "Fire Detected\r\n";
		UART_SendString(LPC_UART3, FireMsg);
	}

	if (lightFlag == true && (accelerationCal >= ACCEL_LIMIT))
	{
		blink_blue = 1;         //raise blink blue flag when light intensity is low and movement is detected
		unsigned char DarkMovementMsg[] = "Movement in Darkness Detected\r\n";
		UART_SendString(LPC_UART3, DarkMovementMsg);
	}
}


int main (void)
{
    init();
    extInteruptInit();

    env envValues;
    env *ptr = &envValues;

    volatile static int charCounter = 0;
    int sevenSegTicks = getTicks();
    int rgbBothTicks = getTicks();

    LPC_GPIOINT->IO2IntEnF |= 1 << 5; // light interupt triggered on falling edge

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
                lightFlag = false;
                charCounter = 0;

                break;

            case 0:
                monitorMode(monitorFlag); // start monitor mode
                sampleEnv(ptr);
                fireOrDarkness(ptr);
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
