/*  

Spatial Mapper

*/

#include <stdint.h>
#include <stdio.h>
#include <math.h> // needed for cos and sin
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


/// ************** GLOBAL VARIABLES *************** //

// copied from stepper motor lab 6
uint8_t currentState = 0; // zero is IDLE, 1 is RUNNING
int lastState [1] = {1}; // for button debounce, up to n buttons, for this deliverable: 1 button. keeps track of what's pressed.

uint16_t    dev = 0x29;             //address of the ToF sensor as an I2C slave peripheral
int status=0;

// debugging & excel
double x_coords[8]; // holds the x coords, im assuming x = distance * cos(-angle)
double y_coords[8]; // holds the y coords, im assuming y = distance * cos(-angle)
int dis_data[8]; // holds the distance measured, for debugging.


void getDistanceMeasurement(double angle, int j); // function prototype, forward declaration, j is the iteration variable.

/// ****************** Initializations and Function definitions *************

void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;                                                                                      // activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;                                                                              // activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};                                                                                                       // ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;                                                                                                                 // 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;                                                                                                                   // 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;                                                                                                                   // 5) enable digital I/O on PB2,3
//    GPIO_PORTB_AMSEL_R &= ~0x0C;                                                                                                        // 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                                                                                                 // 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                          // 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//    I2C0_MTPR_R = 0x3B;                                                                       // 8) configure for 100 kbps clock
       
}

//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
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

//input: PM0
void PortM_Init(void){
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R11;                 // Activate the clock for Port M
      while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R11) == 0){};      // Allow time for clock to stabilize
				
      GPIO_PORTM_DIR_R |= 0x01;                                 // Enable PM0
			GPIO_PORTM_DEN_R |= 0x01;                                                                                                              // Enable PM0 as digital pin
      return;
}

void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x02;    										// Make PJ1 input 
  GPIO_PORTJ_DEN_R |= 0x02;     										// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								//  Configure PJ1 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x02;											//  Disable analog functionality on PJ1		
	GPIO_PORTJ_PUR_R |= 0x02;													//	Enable weak pull up resistor on PJ1
}

//output LED D2 (which is PN0) since Student # = (400450701).
void PortN_Init(void){
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12;                 // Activate the clock for Port N
      while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R12) == 0){};      // Allow time for clock to stabilize
            
      GPIO_PORTN_DIR_R |= 0x03;                                 // Enable PN0,PN1 (LED D2) output
			GPIO_PORTN_DEN_R |= 0x03;                                     // Enable PN0,PN1 as digital pins
      return;
}

//stepper motor
void PortH_Init(void){
      SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                    // activate clock for Port H
      while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};     // allow time for clock to stabilize
      GPIO_PORTH_DIR_R |= 0x0F;                                                     // configure Port H pins (PH0-PH3) as output
  GPIO_PORTH_AFSEL_R &= ~0x0F;                                                // disable alt funct on Port H pins (PH0-PH3)
  GPIO_PORTH_DEN_R |= 0x0F;                                                   // enable digital I/O on Port H pins (PH0-PH3)
                                                                                                                                                      // configure Port H as GPIO
  GPIO_PORTH_AMSEL_R &= ~0x0F;                                                // disable analog functionality on Port H pins (PH0-PH3)    
      return;
}


//XSHUT     This pin is an active-low shutdown input;
//                            the board pulls it up to VDD to enable the sensor by default.
//                            Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
   
}

void blink_LED() // blink the measurement LED (aka PN1)
{
      GPIO_PORTN_DATA_R ^= 0x01;
}

// Button debounce routine:
// change SysTick for faster button processing, default is 100ms.
// small sysTick value may result in failed button debounce, better to keep it at =100ms.

int checkButtonPress(uint32_t port, uint32_t pin){ // is not just limited to 1 button, can be used for any amount
      
      int mask = 1 << pin; // Create a mask for the current pin (aka bit, ex: PM0 would be pin=0)
      int curr = GPIO_PORTJ_DATA_R & mask; // Read the current state of button
           
  if((curr != lastState[pin]) && (curr == 0)) // If button state changed (happens when curr=0 due to active LO push btn)
  {
            SysTick_Wait10ms(10); // Debounce delay to check if it wasnt by accident
            curr = GPIO_PORTJ_DATA_R & mask; // Re-read the button state after delay
               
            if((curr != lastState[pin]) && (curr == 0)) // Confirm button press after delay
    {
                  lastState[pin] = curr; // Update the last state for button
                  return 1; //return 1 indicating button was pressed
    }
  }
      else if(curr != 0) // If button i is not pressed, update its state
  {
            lastState[pin] = curr; // Update to reflect the button is not pressed
  }
      return 0;
}

void buttonHandler()
{
            if(checkButtonPress(GPIO_PORTJ_DATA_R, 1))
            {
                  currentState ^= 1; //toggle the state, this way motor status is complemented on every button press.
            }     
}
      
void spin()
{
      uint32_t delay = 5; //fastest speed = 2ms
      double angle = 0; // start at positive x - axis
      int j = 0; // iteration variable for plugging in values into the global debug arrays
      
      for(int i=1; i <= 512; i++) // 512 * 4 = 2048, overall 1 full 360 degree rotation.
      {
            GPIO_PORTH_DATA_R = 0b00000011;
            SysTick_Wait1ms(delay);                                                             
            GPIO_PORTH_DATA_R = 0b00000110;
            SysTick_Wait1ms(delay);
            GPIO_PORTH_DATA_R = 0b00001100;
            SysTick_Wait1ms(delay);
            GPIO_PORTH_DATA_R = 0b00001001;
            SysTick_Wait1ms(delay);
            
            if(i%16==0){ // this will make it so that the motor stops 32 times (or every 11.25 degrees)
            SysTick_Wait10ms(10);
            getDistanceMeasurement(angle, j); // gather data
            angle = angle + 11.25; // angle increases each time
            j++;
            }
            
            if(checkButtonPress(GPIO_PORTJ_DATA_R, 1) || i == 511)
            {
                  currentState^=1;
                  break;
            }                 
      }
}

void getDistanceMeasurement(double angle, int j)
{
        uint16_t Distance;
            uint8_t RangeStatus;
            uint8_t dataReady = 0;
      
            status = VL53L1X_StartRanging(dev);   // 4 This function has to be called to enable the ranging
            GPIO_PORTN_DATA_R = 0b00000011;
            for (int i = 0; i < 5; i++)
            /*
            for some reason theres a delay in gathering distance data, so to fix it I just made it so that the sensor
            gathers distance data 5 times and takes the distance measurement of the 5th iteration so that we know its accurate
            and not some accidental delayed distance that was acquired.
            */
            {
                  //5 wait until the ToF sensor's data is ready
                  while (dataReady == 0){
                  status = VL53L1X_CheckForDataReady(dev, &dataReady);
                                    //FlashLED3(1);
                                    VL53L1_WaitMs(dev, 5);
                  }
                  dataReady = 0;
        
                  //7 read the data values from ToF sensor
                  status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
                  status = VL53L1X_GetDistance(dev, &Distance);                           //7 The Measured Distance value
									if(Distance > 3500 || RangeStatus!=0) Distance = 4000;
   
                  //FlashLED4(1);

                  status = VL53L1X_ClearInterrupt(dev); /* 8 clear interrupt has to be called to enable next interrupt*/
                  SysTick_Wait1ms(5);
									
									//if(RangeStatus != 0) i=0;
            }
						GPIO_PORTN_DATA_R = 0b00000010;
            // print the resulted readings to UART
            sprintf(printf_buffer,"%u, %u\r\n", Distance, RangeStatus);
            UART_printf(printf_buffer);

                        
            VL53L1X_StopRanging(dev);
            
}

void unspin()
{
      uint32_t delay = 2; //fastest speed = 2ms
      
      for(int i=1; i <= 512; i++) // 512 * 4 = 2048, overall 1 full 360 degree rotation.
      {
            GPIO_PORTH_DATA_R = 0b00001001;
            SysTick_Wait1ms(delay); 
            GPIO_PORTH_DATA_R = 0b00001100;           
            SysTick_Wait1ms(delay);
            GPIO_PORTH_DATA_R = 0b00000110;
            SysTick_Wait1ms(delay);
            GPIO_PORTH_DATA_R = 0b00000011;
            SysTick_Wait1ms(delay);             
      }
}


//*********************************************************************************************************
//*********************************************************************************************************
//***********                             MAIN Function                       *****************************************************************
//*********************************************************************************************************
//*********************************************************************************************************

int main(void) {
  uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;

      //initialize
      PLL_Init(); 
      SysTick_Init();
      onboardLEDs_Init();
      I2C_Init();
      UART_Init();
      PortM_Init();                                                                                                           // Initialize Port M for button input
      PortH_Init();                                                                                                           // Initialize Port H for stepper motor
      PortN_Init(); 
			PortJ_Init();
      
      // hello world!
      UART_printf("Program Begins\r\n");
      int mynumber = 1;
      sprintf(printf_buffer,"2DX ToF Program Studio Code %d\r\n",mynumber);
      UART_printf(printf_buffer);


/* Those basic I2C read functions can be used to check your own I2C functions */
      status = VL53L1X_GetSensorId(dev, &wordData);

      sprintf(printf_buffer,"(Model_ID, Module_Type)=0x%x\r\n",wordData);
      UART_printf(printf_buffer);

      // 1 Wait for device ToF booted
      while(sensorState==0){
            status = VL53L1X_BootState(dev, &sensorState);
            SysTick_Wait10ms(10);
  }
      FlashAllLEDs();
      UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
      
      status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
      
  /* 2 Initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
      Status_Check("SensorInit", status);

      
  /* 3 Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
//  status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */
      
      while(1)
            {
                  buttonHandler();
                  switch(currentState)
                  {
                        case 0: //idle
                              GPIO_PORTN_DATA_R &= 0xE;
                              break;
                        
                        case 1: //running
															GPIO_PORTN_DATA_R = 0b00000010; // debug status
                              spin();                                               // get measurements
															GPIO_PORTN_DATA_R = 0b00000000;
                              SysTick_Wait10ms(10);   // wait 100 ms
                              unspin();                                       // untangle wires
                              break;
                        
                        default:
                              break;
                  }
								
            }
}
