//@author Andrew Capatina / Ryan Bornhorst
/**		pidRTOS.c
 * Main software file for project 2
 *
 * Description:
 * 	This file contains the tasks/functions needed
 * 	to run the PID loop control algorithm for a
 * 	DC motor. Queues and semaphores are used
 * 	to control execution.
 * 	This file intializes the necessary peripherals created
 * 	in the block design file for Vivado. Note that the watchdog timer is
 * 	implemented as well. Interrupts handle the reset and the switch
 * 	is used to stop the application.
 *
 *
 *	5/17/2019
 *
 *
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* BSP includes. */
#include "xtmrctr.h"
#include "xgpio.h"
#include "sleep.h"
#include "xstatus.h"
#include "xuartlite.h"
#include "xwdttb.h"

#include "nexys4IO.h"
#include "xparameters.h"
#include "PmodOLEDrgb.h"
#include "PmodENC.h"

#include "pwm_tmrctr.h"

#define mainQUEUE_LENGTH					( 1 )

/* A block time of 0 simply means, "don't block". */
#define mainDONT_BLOCK						( portTickType ) 0

// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_HIGHADDR

// Definitions for peripheral AXI Timer
#define AXI_TIMER_0_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_0_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_0_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR


#define AXI_TIMER_1_DEVICE_ID		XPAR_AXI_TIMER_1_DEVICE_ID
#define AXI_TIMER_1_BASEADDR		XPAR_AXI_TIMER_1_BASEADDR
#define AXI_TIMER_1_HIGHADDR		XPAR_AXI_TIMER_1_HIGHADDR

#define TmrCtrNumber_0				0	// Timers in each instance
#define TmrCtrNumber_1				1

// Definition for UARTlite
#define UART_DEVICE_ID			XPAR_AXI_UARTLITE_0_DEVICE_ID
#define UART_BASEADDR			XPAR_AXI_UARTLITE_0_BASEADDR
#define UART_BAUDRATE			XPAR_AXI_UARTLITE_0_BAUDRATE
#define UART_DATABITS			XPAR_AXI_UARTLITE_0_DATA_BITS
#define UART_PARITYODD			XPAR_AXI_UARTLITE_0_ODD_PARITY
#define UART_USEPARITY			XPAR_AXI_UARTLITE_0_USE_PARITY

// Definition of Watchdog Timer.
#define WDT_DEVICE_ID			XPAR_AXI_TIMEBASE_WDT_0_DEVICE_ID
#define WDT_BASEADDR			XPAR_AXI_TIMEBASE_WDT_0_BASEADDR
#define WDT_HIGHADDR			XPAR_AXI_TIMEBASE_WDT_0_HIGHADDR
#define WDT_EN_WNDOW			XPAR_WDTTB_0_ENABLE_WINDOW_WDT
#define WDT_MAX_WDTH			XPAR_WDTTB_0_MAX_COUNT_WIDTH
#define WDT_SST_WDTH			XPAR_WDTTB_0_SST_COUNT_WIDTH
#define WDT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_WDT_INTERRUPT_INTR

// Switch defines
#define SW0		0x00000001
#define SW1		0x00000002
#define SW2		0x00000004
#define SW3		0x00000008
#define SW4		0x00000010
#define SW5		0x00000020
#define SW16	0x00008000

// Structure containing pid correction paramaters.
typedef struct	/* Took structure from PID without a phd pdf document. */
{
	int derState;
	int integratState;	// Integrator state.
	int integratMax,	// Maximum and minimum integrator state.
		integratMin;
	int integratGain,	// Integral gain.
		propGain,		// Proportional gain.
		derGain;		// Derivative gain.

} pid_struct;

// Microblaze peripheral instances
PmodOLEDrgb	pmodOLEDrgb_inst;
PmodENC 	pmodENC_inst;
XTmrCtr		AXITimerInst_0, AXITimerInst_1;

//Create Instances
static XGpio xOutputGPIOInstance, xInputGPIOInstance;
static XGpio xOutputGPIOInstance_1, xInputGPIOInstance_1;
static XGpio xOutputGPIOInstance_2;
static XWdtTb		WDTInst;								// Watchdog timer instance.
static XUartLite   	UartInst;								// Uart lite instance.

// Create configuration structures
static XUartLite_Config 	UARTconfig;	// uart
static XWdtTb_Config		WDTconfig;	// watchdog timer

//Function Declarations
static void prvSetupHardware( void );
int AXI_Timer_0_initialize(void);
int AXI_Timer_1_initialize(void);
void timer_load(u32 duty_cycle);

int update_pid(pid_struct * pid, int error, int rpm);

//Declare a Sempahore
xSemaphoreHandle binary_sem;
xSemaphoreHandle fail_sem;		// Created when failure occurs.

/* The queue used by the queue send and queue receive tasks. */
static xQueueHandle xQueue = NULL;

static int is_running;		// Global flag indicating if the application is running.

// used for checking if PID signals are being used
bool p_on = false;
bool i_on = false;
bool d_on = false;


/**
 *  Description:
 * 		Task responsible for prompting user when
 *   	timing failure occurs. Happens when
 *     	watch dog reports timer overflow.
 *
 */

void is_app_running(void * is_running)
{
	while(1)
	{
		if(xSemaphoreTake(fail_sem,500)){	// Checking if timing error occurred.
			xil_printf("WATCHDOG: TIMING FAILURE - RESTART REQUIRED\n\r");	// Prompt for restart.
			is_running = 0;
		}
	}
}

/**
 * 	Description:
 * 		The watchdog timer interrupt handler.
 * 		This function checks the switches to see if
 * 		force reset asserted. Also, determines
 * 		where the reset signal came from.
 * 		Sets program state to not running when reset
 * 		occurs due to timer overflow.
 *
 */

static void wdt_intr(void *pvUnused) {
		u16 switches = NX4IO_getSwitches();

		if((switches & SW16) != SW16)
		{
			XWdtTb_RestartWdt(&WDTInst);		// Restart the watch dog timer.
		}
		else
		{
			is_running = 0;
			XWdtTb_RestartWdt(&WDTInst);		// Restart the watch dog timer.
			xSemaphoreGiveFromISR(fail_sem, NULL);	// Give semaphore on failure.
		}
}

//ISR, to handle interrupt og GPIO dip switch
//Can also use Push buttons.
//Give a Semaphore
static void gpio_intr (void *pvUnused)
{
	xSemaphoreGiveFromISR(binary_sem,NULL);

	XGpio_InterruptClear( &xInputGPIOInstance, 1 );

}

//A task which takes the Interrupt Semaphore and sends a queue to task 2.
void sem_taken_que_tx (void *p)
{
	unsigned long ulValueToSend = 0UL;

	while(1)
	{
		if(xSemaphoreTake(binary_sem,500)){
			if(p_on && i_on && d_on)
				ulValueToSend = (SW0|SW1|SW2);		// Set value for LEDs
			else if(p_on && i_on)
				ulValueToSend = (SW2|SW1);
			else if(p_on && d_on)
				ulValueToSend = (SW2|SW0);
			else if(i_on && d_on)
				ulValueToSend = (SW1|SW0);
			else if(p_on)
				ulValueToSend = SW2;
			else if(i_on)
				ulValueToSend = SW1;
			else if(d_on)
				ulValueToSend = SW0;
			else
				ulValueToSend = 0;
			xil_printf("Queue Sent: %d\n\r",ulValueToSend);
			xQueueSend( xQueue, &ulValueToSend, mainDONT_BLOCK );
		}
	}
}

/**
 * Description:
 * 	Task that writes to LEDs.
 *
 */
void que_rx (void *p)
{
	unsigned long ulReceivedValue;

	while(1){
		xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );
		ulReceivedValue = ulReceivedValue | (NX4IO_getSwitches() & 0xFF00);
		//Write to LED.
		XGpio_DiscreteWrite( &xOutputGPIOInstance, 2, ulReceivedValue );
		xil_printf("Queue Received: %d\n",ulReceivedValue);
	}
}

/**
 * 	Function for converting an integer to a string.
 * 	radix paramaters allows different number formats.
 *
 */
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}

  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;

	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;

  	return;
}

void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];

  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);

  return;
}

/**
 * 	Primary task of the application.
 *
 * 	Description:
 * 		Sets the PWM duty cycle for the motor as well as
 * 		setting correction information using PID algorithm.
 * 		Manages UI for showing user their options.
 *
 *
 *
 */

void run_pwm(void *pwm)
{
	pid_struct pid_info;				// Allocate memory for structure with pid error information.
	pid_struct * pid_ptr = &pid_info;	// Assign address to pointer.

	// encoder states
	u32 state, laststate;

	// button states
	u32 btnU = 0;
	u32 btnU_last = 0;
	u32 btnD = 0;
	u32 btnD_last = 0;

	// encoder ticks for p,i,d
	int p_ticks = 0;				// Changed these initializations from 0 to 1.
	int p_lastticks = 1;
	int i_ticks = 0;
	int i_lastticks = 1;
	int d_ticks = 0;
	int d_lastticks = 1;

	// encoder ticks for rpm
	int rpm_ticks = 0;
	int rpm_lastticks = 1;

	// use to get switch values
	u16 sw_state;

	u32 tmr_duty;		// Holds user selection using encoder.
	int rpm_data = 0;	// Holds rpm
	int rpm_lastdata;	// Last known rpm
	int rpm_data1 = 0;	// Last known rpm 1
	int rpm_data2 = 0;	// last known rpm 2
	int rpm_calc = 0;	// RPM conversion result.

	// booleans used to determine states
	bool chg_const_5;		// p,i,d values will inc/dec by 5
	bool chg_const_10;		// p,i,d values will inc/dec by 10
	bool chg_rpm_5;			// rpm will inc/dec by 5
	bool chg_rpm_10;		// rpm will inc/dec by 10

	// turn off all seven seg digits
	NX410_SSEG_setAllDigits(SSEGLO, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_BLANK, CC_BLANK, CC_BLANK, DP_NONE);

	// setup OLEDrgb display
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst, OLEDrgb_BuildRGB(200,12,44));

	// display values for (Kp, Ki, Kd)
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst, "P:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 2);
	OLEDrgb_PutString(&pmodOLEDrgb_inst, "I:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
	OLEDrgb_PutString(&pmodOLEDrgb_inst, "D:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
	OLEDrgb_PutString(&pmodOLEDrgb_inst, "RPM:");

	// initialize previous state of Encoder
	laststate = ENC_getState(&pmodENC_inst);

	while(1)
	{

		if(is_running == 0)
		{
			continue;			// Checking global run flag. Skip loop if set to zero.
		}

		// grab the Encoders state
		state = ENC_getState(&pmodENC_inst);
		// get switch state
		sw_state = NX4IO_getSwitches();

		// determine how quickly our values are changing
		// by updating these booleans that reflect the switch values
		chg_const_5 = (sw_state & (SW4|SW5)) == SW4;
		chg_const_10 = (sw_state & SW5) == SW5;
		chg_rpm_5 = (sw_state & (SW0|SW1)) == SW0;
		chg_rpm_10 = (sw_state & SW1) == SW1;


		if(rpm_calc == 0)	// Checking if the motor is stopped.
		{
			// change direction of the motor using Encoder switch
			if(ENC_switchOn(state) && !ENC_switchOn(laststate))
			{
				// rotate positive direction
				XGpio_DiscreteWrite( &xOutputGPIOInstance_2, 1, 0x1 );
			} else
			{
				// rotate negative direction
				XGpio_DiscreteWrite( &xOutputGPIOInstance_2, 1, 0x0 );
			}
		}

		// use btnU, btnD to change control constants Kp,Ki,Kd
		btnU = NX4IO_isPressed(BTNU);
		if(btnU && !btnU_last)
		{
			// modify the Kp state ---> SW2
			if((sw_state & (SW2|SW3)) == SW2)
			{
				if(chg_const_5)
					p_ticks += 5;
				else if(chg_const_10)
					p_ticks += 10;
				else
					p_ticks += 1;

				if(p_ticks > 255)
					p_ticks = 255;

			// modify the Ki state ---> SW3
			} else if((sw_state & (SW2|SW3)) == SW3)
			{
				if(chg_const_5)
					i_ticks += 5;
				else if(chg_const_10)
					i_ticks += 10;
				else
					i_ticks += 1;

				if(i_ticks > 255)
					i_ticks = 255;

			// modify the Kd state ---> SW2 & SW3
			} else if((sw_state & (SW2|SW3)) == (SW2|SW3))
			{
				if(chg_const_5)
					d_ticks += 5;
				else if(chg_const_10)
					d_ticks += 10;
				else
					d_ticks += 1;

				if(d_ticks > 255)
					d_ticks = 255;
			}
		}

		// use btnU, btnD to change control constants Kp,Ki,Kd
		btnD = NX4IO_isPressed(BTND);
		if(btnD && !btnD_last)
		{
			// modify the Kp state ---> SW2
			if((sw_state & (SW2|SW3)) == SW2)
			{
				if(chg_const_5)
					p_ticks -= 5;
				else if(chg_const_10)
					p_ticks -= 10;
				else
					p_ticks -= 1;

				if(p_ticks < 0)
					p_ticks = 0;

			// modify the Ki state ---> SW3
			} else if((sw_state & (SW2|SW3)) == SW3)
			{
				if(chg_const_5)
					i_ticks -= 5;
				else if(chg_const_10)
					i_ticks -= 10;
				else
					i_ticks -= 1;

				if(i_ticks < 0)
					i_ticks = 0;

			// modify the Kd state ---> SW2 & SW3
			} else if((sw_state & (SW2|SW3)) == (SW2|SW3))
			{
				if(chg_const_5)
					d_ticks -= 5;
				else if(chg_const_10)
					d_ticks -= 10;
				else
					d_ticks -= 1;

				if(d_ticks < 0)
					d_ticks = 0;
			}
		}
		// clear values on center button press
		// turn off the pwm signal
		if(NX4IO_isPressed(BTNC))
		{
			p_ticks = 0;
			i_ticks = 0;
			d_ticks = 0;
			rpm_ticks = 0;
		} else
		{
			// increment Encoder tick counts for rpm
			if(rpm_ticks < 256)
			{
				// rotation is going up
				if(ENC_getRotation(state, laststate) == 1)
				{
					if(chg_rpm_5)
						rpm_ticks += 5;
					else if(chg_rpm_10)
						rpm_ticks += 10;
					else
						rpm_ticks += 1;

				// rpm is going down
				} else if(ENC_getRotation(state, laststate) == -1)
				{
					if(chg_rpm_5)
						rpm_ticks -= 5;
					else if(chg_rpm_10)
						rpm_ticks -= 10;
					else
						rpm_ticks -= 1;
				}
			}

			// keep the rpm between 0 and 255
			if(rpm_ticks > 255)
				rpm_ticks = 255;
			else if(rpm_ticks < 0)
				rpm_ticks = 0;
		}

		// change display to match Encoder value for Kp
		if(p_ticks != p_lastticks)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 1);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 1);
			PMDIO_putnum(&pmodOLEDrgb_inst, p_ticks, 10);
		}

		// change display to match Encoder value for Ki
		if(i_ticks != i_lastticks)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 2);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 2);
			PMDIO_putnum(&pmodOLEDrgb_inst, i_ticks, 10);
		}

		// change display to match Encoder value for Kd
		if(d_ticks != d_lastticks)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 3);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 3);
			PMDIO_putnum(&pmodOLEDrgb_inst, d_ticks, 10);
		}

		// change display to match Encoder value for rpm
		if(rpm_ticks != rpm_lastticks)
		{
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 5);
			OLEDrgb_PutString(&pmodOLEDrgb_inst,"    ");
			OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 5, 5);
			PMDIO_putnum(&pmodOLEDrgb_inst, rpm_ticks, 10);

			// change the speed of the motor based on rpm ticks
			tmr_duty = rpm_ticks;

		}


		if(rpm_ticks < 10)
			NX4IO_SSEG_putU32Dec(0, true);	// Set display to zero when rpm too low.

		rpm_lastdata = rpm_data;			// Save last rpm known.

		rpm_data2 = rpm_data1;				// Saves second to last known rpm.
		rpm_data1 = rpm_data;				// Saves last known rpm.
		rpm_data = XGpio_DiscreteRead( &xInputGPIOInstance_1, 1 );	// Read RPM from hardware.

		u32 rpm_avg = (rpm_data + rpm_data1 + rpm_data2)/3;			// Average the encoder ticks.
		rpm_calc = ((rpm_avg)/12)*60;								// Convert encoder ticks to RPM.

		/* Section of code for correcting the motor rpm. 	*/
		if(rpm_lastdata != rpm_data || rpm_ticks != rpm_lastticks)
		{
			pid_ptr->integratMin = 0;			// Setting the minimum and max rpm.
			pid_ptr->integratMax = 5000;

			pid_ptr->integratGain = i_ticks;	// Setting gain paramaters for loop control.
			pid_ptr->propGain = p_ticks;
			pid_ptr->derGain = d_ticks;

			u32 tmr_duty_param = tmr_duty*19;		// Multiply by 19 because 5000/255 = 19.6.
													// Converting to rpm essentially.
			int drive = update_pid(pid_ptr, tmr_duty_param - rpm_avg, rpm_avg);	// Run pid loop.
			// display on seven segment
			drive = (drive / 30)/2;				// Scaling the drive result.
			if(drive < 0)		// Limit the output.
			{
				drive = 0;
			}
			else if(drive > 254)
			{
				drive = 254;
			}
			if(pid_ptr->derGain == 0 && pid_ptr->integratGain == 0 && pid_ptr ->propGain == 0)
			{
				drive = tmr_duty;		// Set the drive to user input if their gain parameters are ALL 0.
			}
			NX4IO_SSEG_putU32Dec(tmr_duty*19, true);	// Write to seven segment.
			timer_load(drive);							// Drive PWM output.
			// Print: motor speed, desired motor speed, KP, and error. CSV FORMAT
			xil_printf("%i, %i, %i, %i,\n\r", rpm_calc, tmr_duty*19, pid_ptr->propGain, tmr_duty_param - rpm_avg);

		}

		// setup the interrupt task to display appropriate leds
		// p const is being used
		if((sw_state & (SW2)) == SW2)
			p_on = true;
		else
			p_on = false;

		// i const is beging used
		if((sw_state & (SW3)) == SW3)
			i_on = true;
		else
			i_on = false;

		// d const is being used
		if((sw_state & (SW3|SW2)) == (SW3|SW2))
			d_on = true;
		else
			d_on = false;

		// keep track of previous states
		laststate = state;
		p_lastticks = p_ticks;
		i_lastticks = i_ticks;
		d_lastticks = d_ticks;
		rpm_lastticks = rpm_ticks;
		rpm_lastdata = rpm_data;
		btnU_last = btnU;
		btnD_last = btnD;
	}
}

/**
 * 	Description:
 * 		Function to update pid_struct members.
 * 		This function was taken from PID without a PHD
 * 		pdf document given.
 *
 * 		:param: pid_struct :type: pid_struct* - error info for current state of program
 * 		:param: error :type: int	- error compared to last rpm output
 * 		:param: position :type: int	-
 */
int update_pid(pid_struct * pid, int error, int rpm)
{
	int p_term=0,d_term=0, i_term = 0;

	p_term = pid->propGain * error; // Calculate proportional gain.

	// Calculate integral state with limiting.
	pid->integratState += error;

	// Limit integrat state
	if(pid->integratState > pid->integratMax)
	{
		pid->integratState = pid->integratMax;
	}
	else if(pid->integratState < pid->integratMin)
	{
		pid->integratState = pid->integratMin;
	}

	// Calculate the integral term.
	i_term = pid->integratGain * pid -> integratState;
	// Calculate derivative.
	d_term = pid->derGain * (pid->derState - rpm);
	pid->derState = rpm;
	return p_term + d_term + i_term;


}

int main(void)
{
	is_running = 1;

	//Initialize the HW
	prvSetupHardware();

	//Creatye Semaphore
	vSemaphoreCreateBinary(binary_sem);
	// semaphore used on failure.
	vSemaphoreCreateBinary(fail_sem);

	/* Create the queue */
	xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( unsigned long ) );

	/* Sanity check that the queue was created. */
	configASSERT( xQueue );

	//Create Task1
	xTaskCreate( sem_taken_que_tx,
				 ( const char * ) "TX",
				 2048,
				 NULL,
				 1,
				 NULL );

	//Create Task2
	xTaskCreate( que_rx,
				"RX",
				2048,
				NULL,
				2,
				NULL );

	//Create Task
	xTaskCreate( run_pwm,
				 "PWM",
				 2048,
				 NULL,
				 1,
				 NULL );

	xTaskCreate( is_app_running,
				 "IS_RUNNING",
				 2048,
				 NULL,
				 5,
				 NULL );

	//Start the Secheduler
	vTaskStartScheduler();

	return -1;
}


static void prvSetupHardware( void )
{
portBASE_TYPE xStatus;
const unsigned char ucSetToOutput = 0U;

	/* Initialize Nexys4 driver */
	xStatus = (portBASE_TYPE) NX4IO_initialize(NX4IO_BASEADDR);
	if( xStatus == XST_SUCCESS )
	{
		/* initialize the pmodOLED */
		//OLEDrgb_EnablePmod(&pmodOLEDrgb_inst,1);
		OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);
		OLEDrgb_end(&pmodOLEDrgb_inst);
		OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

		/* initialize the pmodENC */
		ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);

		UARTconfig.BaudRate = UART_BAUDRATE;		// Setting configuration paramaters for UART.
		UARTconfig.DataBits = UART_DATABITS;
		UARTconfig.DeviceId = UART_DEVICE_ID;
		UARTconfig.ParityOdd = UART_PARITYODD;
		UARTconfig.RegBaseAddr = UART_BASEADDR;
		UARTconfig.UseParity = UART_USEPARITY;

		xStatus = XUartLite_CfgInitialize(&UartInst,&UARTconfig, UARTconfig.RegBaseAddr);

		WDTconfig.BaseAddr = WDT_BASEADDR;		// Setting watchdog config paramaters.
		WDTconfig.DeviceId = WDT_DEVICE_ID;
		WDTconfig.EnableWinWdt = WDT_EN_WNDOW;
		WDTconfig.MaxCountWidth = WDT_MAX_WDTH;
		WDTconfig.SstCountWidth = WDT_SST_WDTH;

		XWdtTb_Initialize(&WDTInst, WDTconfig.DeviceId);
	}

	/* initialize AXI Timer 0,1 */
	AXI_Timer_0_initialize();
	AXI_Timer_1_initialize();

	/* Initialize the GPIO for the LEDs. */
	xStatus = XGpio_Initialize( &xOutputGPIOInstance, XPAR_AXI_GPIO_1_DEVICE_ID );

	if( xStatus == XST_SUCCESS )
	{
		/* All bits on this channel are going to be outputs (LEDs). */
		XGpio_SetDataDirection( &xOutputGPIOInstance, 2, ucSetToOutput );

		XGpio_DiscreteWrite( &xOutputGPIOInstance, 2, 0 );
	}

	/* Initialise the GPIO for the button inputs. */
	if( xStatus == XST_SUCCESS )
	{
		xStatus = XGpio_Initialize( &xInputGPIOInstance, XPAR_AXI_GPIO_1_DEVICE_ID );
	}

	/* Initialize the GPIO_1 output */
	xStatus = XGpio_Initialize( &xOutputGPIOInstance_1, XPAR_AXI_GPIO_0_DEVICE_ID );
	if( xStatus == XST_SUCCESS )
	{
		/* All bits on this channel are going to be outputs (LEDs). */
		XGpio_SetDataDirection( &xOutputGPIOInstance_1, 2, ucSetToOutput );

		XGpio_DiscreteWrite( &xOutputGPIOInstance_1, 2, 0 );
	}

	/* Initialize the GPIO_2 output */
	xStatus = XGpio_Initialize( &xOutputGPIOInstance_2, XPAR_AXI_GPIO_2_DEVICE_ID );
	if( xStatus == XST_SUCCESS )
	{
		/* All bits on this channel are going to be outputs (LEDs). */
		XGpio_SetDataDirection( &xOutputGPIOInstance_2, 1, ucSetToOutput );

		XGpio_DiscreteWrite( &xOutputGPIOInstance_2, 1, 0 );
	}

	/* Initialize the GPIO_1 input */
	if( xStatus == XST_SUCCESS )
	{
		xStatus = XGpio_Initialize( &xInputGPIOInstance_1, XPAR_AXI_GPIO_0_DEVICE_ID );
	}

	if( xStatus == XST_SUCCESS )
	{
		XGpio_SetDataDirection( &xInputGPIOInstance_1, 1, 0 );
	}

	if( xStatus == XST_SUCCESS )
	{
		/* Install the handler defined in this task for the button input.
		*NOTE* The FreeRTOS defined xPortInstallInterruptHandler() API function
		must be used for this purpose. */
		xStatus = xPortInstallInterruptHandler( XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR, gpio_intr, NULL );

		/* Install watchdog interrupt handler. */
		xStatus = xPortInstallInterruptHandler(XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_TIMEBASE_INTERRUPT_INTR, wdt_intr, NULL);

		if( xStatus == pdPASS )
		{

			/* Set buttons to input. */
			XGpio_SetDataDirection( &xInputGPIOInstance, 1, 0 );

			/* Enable the button input interrupts in the interrupt controller.
			*NOTE* The vPortEnableInterrupt() API function must be used for this
			purpose. */

			vPortEnableInterrupt( XPAR_MICROBLAZE_0_AXI_INTC_AXI_GPIO_1_IP2INTC_IRPT_INTR );
			/* Enable watchdog interrupt */
			vPortEnableInterrupt( XPAR_MICROBLAZE_0_AXI_INTC_AXI_TIMEBASE_WDT_0_TIMEBASE_INTERRUPT_INTR );

			/* Enable GPIO channel interrupts. */
			XGpio_InterruptEnable( &xInputGPIOInstance, 1 );
			XGpio_InterruptGlobalEnable( &xInputGPIOInstance );
		}
	}

	configASSERT( ( xStatus == pdPASS ) );
}

/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_0_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst_0,AXI_TIMER_0_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst_0, TmrCtrNumber_0);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber_0,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber_0, 100000000);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber_0);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber_0);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber_0, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber_0);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_0_BASEADDR, TmrCtrNumber_0, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_0_BASEADDR, TmrCtrNumber_0);
	return XST_SUCCESS;

}

/*
 * Initializes AXI timer 1 with PWM output; follows the datasheet.
 */
int AXI_Timer_1_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst_1,AXI_TIMER_1_DEVICE_ID);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	status = XTmrCtr_SelfTest(&AXITimerInst_1, TmrCtrNumber_0);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_LOAD_MASK | XTC_CSR_DOWN_COUNT_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_0,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_0, 255);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_0);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_0);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_0, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_0);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_0, ctlsts);

	status = XTmrCtr_SelfTest(&AXITimerInst_1, 1);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;

}

/**
 * 	Function that sets the second timer for AXI timer 1.
 * 	Second timer sets high time of PWM output.
 *
 */
void timer_load(u32 duty_cycle)
{
	u32		ctlsts;		// control/status register or mask

	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_ENABLE_PWM_MASK | XTC_CSR_LOAD_MASK | XTC_CSR_DOWN_COUNT_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1, duty_cycle);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1);
	ctlsts |= XTC_CSR_ENABLE_ALL_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_1_BASEADDR, TmrCtrNumber_1, ctlsts);
}



