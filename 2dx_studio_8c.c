#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define MAXRETRIES              5           // number of receive attempts before giving up
#define DEBOUNCE_TIME 10         // 10ms debounce period

volatile double angle_from_home = 0.0; //increases when moving clockwise, decreases when moving counter clockwise
volatile uint8_t spin_toggle = 0; //toggle to check if the motor should be spinning
volatile uint8_t iteration_counter = 0;
volatile int max_tries = 0; //max tries to get a reading from the ToF sensor
volatile uint32_t gTick = 0;       // global tick counter, updated every 1ms in SysTick_Handler
uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
int status = 0;

// SysTick interrupt handler (assumes SysTick is configured for 1ms period)
void SysTick_Handler(void) 
{
    gTick++;
}

void PortJ_Init(void) 
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;                    // Enable clock for Port J (bit 13)
    while ((SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R8) == 0) {}    // Wait for Port J to be ready

    GPIO_PORTJ_DIR_R &= ~0x3;                       // Set PJ0 and PJ1 as input
    GPIO_PORTJ_DEN_R |= 0x3;                        // Enable digital function for PJ0 and PJ1
    GPIO_PORTJ_PCTL_R &= ~0x000000F0;               // Configure PJ1 as GPIO
    GPIO_PORTJ_AMSEL_R &= ~0x3;                     // Disable analog function for PJ0 and PJ1
    GPIO_PORTJ_PUR_R |= 0x3;                        // Enable pull-up resistor for PJ0 and PJ1
    GPIO_PORTJ_DATA_R &= ~0x03;
    return;
}

void PortH_Init(void)
{
    // Stepper motor outputs on PH0-PH3
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;            // Activate clock for Port H
    while((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R7) == 0){}; // Wait for clock to stabilize

    GPIO_PORTH_DIR_R |= 0x0F;       // Set PH0-PH3 as outputs (0x0F = 0b00001111)
    GPIO_PORTH_AFSEL_R &= ~0x0F;    // Disable alternate functions on PH0-PH3
    GPIO_PORTH_DEN_R |= 0x0F;       // Enable digital I/O on PH0-PH3
    GPIO_PORTH_AMSEL_R &= ~0x0F;    // Disable analog functionality on PH0-PH3
    return;
}

void PortF_Init(void)
{
    // Onboard LEDs on PF1 and PF4
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;            // Activate clock for Port F
    while((SYSCTL_PRGPIO_R & SYSCTL_RCGCGPIO_R5) == 0){}; // Wait for clock to stabilize

    GPIO_PORTF_DIR_R |= 0x11;       // Set PF1 (0x02) and PF4 (0x10) as outputs (0x12 = 0b00010010)
    GPIO_PORTF_AFSEL_R &= ~0x11;    // Disable alternate functions on PF1 and PF4
    GPIO_PORTF_DEN_R |= 0x11;       // Enable digital I/O on PF1 and PF4
    GPIO_PORTF_AMSEL_R &= ~0x11;    // Disable analog functionality on PF1 and PF4
    return;
}

void PortN_Init(void) 
{
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                    // Enable clock for Port N (bit 12)
    while ((SYSCTL_RCGCGPIO_R & SYSCTL_RCGCGPIO_R12) == 0) {}    // Wait for Port N to be ready

    GPIO_PORTN_DIR_R |= 0X3;                        // Set PN0 and PN1 as output
    GPIO_PORTN_AFSEL_R &= ~0X3;                     // Disable alternate function for PN0 and PN1
    GPIO_PORTN_DEN_R |= 0X3;                        // Enable digital function for PN0 and PN1

    GPIO_PORTN_AMSEL_R &= ~0X3;                     // Disable analog function for PN0 and PN1    
    return;
}

void D1ON()
{
	GPIO_PORTN_DATA_R |= 0b00000010;
}

void D1OFF()
{
	GPIO_PORTN_DATA_R &= 0b11111101;
}

void D2ON()
{
	GPIO_PORTN_DATA_R |= 0b0000001;
}

void D2OFF()
{
	GPIO_PORTN_DATA_R &= 0b11111110;
}

void D3ON()
{
	GPIO_PORTF_DATA_R |= 0b00010000;
}

void D3OFF()
{
	GPIO_PORTF_DATA_R &= 0b11101111;
}

void D4ON()
{
	GPIO_PORTF_DATA_R |= 0b00000001;
}

void D4OFF()
{
	GPIO_PORTF_DATA_R &= 0b11111110;
}

void spin(int steps, int direction)
{																			// Complete function spin to implement the Full-step Stepping Method
	if(!steps)
	{
		return;
	}
	else if(steps < 0)
	{
		spin(-1*steps, direction);
	}
	uint32_t delay = 6;	// Does your motor spin clockwise or counter-clockwise?
	
	
	if(direction)
	{
		//clockwise
		for(int i=0; i < steps; i++)
		{												// What should the upper-bound of i be for one complete rotation of the motor shaft?
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait1ms(delay);										// What if we want to reduce the delay between steps to be less than 10 ms?
			GPIO_PORTH_DATA_R = 0b0110;													// Complete the missing code.
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b1100;													// Complete the missing code.
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b1001;													// Complete the missing code.
			SysTick_Wait1ms(delay);
			angle_from_home += 0.72;
			iteration_counter++;
		}
	}
	else
	{
		//counter clockwise
		for(int i=0; i < steps; i++)
		{												// What should the upper-bound of i be for one complete rotation of the motor shaft?
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait1ms(delay);											// What if we want to reduce the delay between steps to be less than 10 ms?
			GPIO_PORTH_DATA_R = 0b1100;													// Complete the missing code.
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b0110;													// Complete the missing code.
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b0011;													// Complete the missing code.
			SysTick_Wait1ms(delay);
			angle_from_home -= 0.72;
			iteration_counter--;
		}
	}
}

void button0()
{
	spin_toggle ^= 1;
}

void I2C_Init(void)
{
	SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
	while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3

    // 6) configure PB2,3 as I2C
  	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
	I2C0_MTPR_R = 0b0000000000000101000000000111011;       // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void)
{
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    // allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                                        // make PG0 in (HiZ)
	GPIO_PORTG_AFSEL_R &= ~0x01;                                     // disable alt funct on PG0
	GPIO_PORTG_DEN_R |= 0x01;                                        // enable digital I/O on PG0
	// configure PG0 as GPIO
	//GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
  	GPIO_PORTG_AMSEL_R &= ~0x01;                                     // disable analog functionality on PN0
    return;
}

//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void)
{
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********					MAIN Function				*****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************

int main(void) 
{
	//initialize Variables and GPIO ports
  	uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  	uint16_t wordData;
  	uint16_t Distance;
  	uint16_t SignalRate;
  	uint16_t AmbientRate;
  	uint16_t SpadNum; 
  	uint8_t RangeStatus;
  	uint8_t dataReady;

	PLL_Init();	
	SysTick_Init();
	PortH_Init();
	PortN_Init();
	PortF_Init();
	PortJ_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	
	
	//Intro stuff to signal beginning of program
	UART_printf("Program Begins\r\n");
	int mynumber = 400501776;
	sprintf(printf_buffer,"2DX3 Deliverable 2 Deev Patel %d\r\n",mynumber);
	UART_printf(printf_buffer);


	/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);
	sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
	UART_printf(printf_buffer);

	// 1 Wait for device booted
	while(sensorState == 0)
	{
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  	}
	FlashAllLEDs();
	UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  	/* 2 Initialize the sensor with the default setting  */
  	status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

    // 4 What is missing -- refer to API flow chart
    status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

	//Start command so the MATLAB code can know when to start
	UART_printf("Start\r\n");
	
	//Main program loop
	while(1)
	{
		//spin toggle gets set when PJ0 is pressed which begins the scan of each layer
		//Done like this for modularity and the number of data points per layer can be changed easily
		if(spin_toggle)
		{
			//rotate one Full-Wave cycle 512 times for one full rotation
			for(int i = 0; i < 512; i++)
			{
				spin(1, 1); //spin 0.72 degrees (4 steps) clockwise

				//Configured to scan every 16 steps, every ~2.88 or 128 measurements per layer
				if(i % 4 == 0)
				{
					//wait until the ToF sensor's data is ready
					while (dataReady == 0)
					{
						status = VL53L1X_CheckForDataReady(dev, &dataReady);
						FlashLED3(1);
						VL53L1_WaitMs(dev, 5);
					}
			
					dataReady = 0;

					D1ON(); //turn on LED1 to indicate measurement taken
					//7 read the data values from ToF sensor
					status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
					status = VL53L1X_GetDistance(dev, &Distance);                    //The Measured Distance value
					
					//If the range status is not valid, try to get a valid reading 
					while(VL53L1X_GetRangeStatus(dev, &RangeStatus) != 0)
					{
						//turn on LED3 to indicate a faulty reading was taken
						D3ON();
						status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
						status = VL53L1X_GetDistance(dev, &Distance);
						SysTick_Wait10ms(1);
						//Implement a timeout to prevent infinite loop
						max_tries++;
						if(max_tries == 10)
						{
							break;
						}
					}
					//turn off LED3 to indicate a valid reading was obtained
					max_tries = 0;
					D3OFF(); 
				
					status = VL53L1X_ClearInterrupt(dev); /* 8 clear interrupt has to be called to enable next interrupt*/

					D1OFF(); //turn off LED1 to indicate measurement taken
				
					//turn on LED2 to indicate measurement taken
					D2ON();
					// print the resulted readings to UART
					sprintf(printf_buffer,"[%u]\r\n", Distance);
					UART_printf(printf_buffer);
				
					SysTick_Wait10ms(10);
					D2OFF(); //turn off LED2 to indicate measurement taken
				}
			}
			//spin 360 degrees counter clockwise to reset the angle_from_home
			spin(512, 0); 
			UART_printf("End\r\n");

			spin_toggle = 0; //reset spin toggle to 0
		}

    	// Check Port J button J0 
    	// For active low: button is pressed when the corresponding bit is 0.
    	if ((GPIO_PORTJ_DATA_R & 0x01) == 0)   // PJ0 is pressed
    	{
        	SysTick_Wait10ms(2);             // debounce delay (~20ms)
        	if ((GPIO_PORTJ_DATA_R & 0x01) == 0)
        	{
            	button0();
            	while ((GPIO_PORTJ_DATA_R & 0x01) == 0) { } // wait for release
        	}
    	}
	}
	//End of program
	UART_printf("End\r\n");
	VL53L1X_StopRanging(dev);
  	return 0;
}

