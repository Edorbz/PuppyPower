/*******************************************************************************
 *
 *  Puppy_Power_Main.c
 *     - Puppy Power Firmware designed for the MSP430.  This Program is designed to
 *     give puppy power the following features
 *     # Touch sensor controls
 *     # Voltage Power and Current sensing from an INA219
 *     # Led output display through an SAA1064
 *     # Voltage control through a I2C enabled DAC chip
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Jeffrey Putney
 *  Edorbz LLC
 *  Ver 1.0 Jan 2014
 ******************************************************************************/


#include "CapacitiveTouchLibrary/CTS_Layer.h"
#include "I2C_Com.h"
//#include "uart.h"

#define LED_SLV_ADDR               0x70
#define DAC_SLV_ADDR               0x36  //0x34 on 2.1 and lower boards
#define DAC_SLV_READ_ADDR               0x37  //0x35 on 2.1 and lower boards
#define INA_SLV_ADDR               0x80

#define MASKLED0                   0x3F
#define MASKLED1                   0x06
#define MASKLED2                   0x5B
#define MASKLED3                   0x4F
#define MASKLED4                   0x66
#define MASKLED5                   0x6D
#define MASKLED6                   0x7D
#define MASKLED7                   0x07
#define MASKLED8                   0x7F
#define MASKLED9                   0x67
#define MASKLEDOFF                   0x00

void Master_Transmit(void);
void Master_Recieve(void);

void Setup_USI_Master_TX(void);
void Setup_USI_Master_RX(void);
void Contruct_Send_Message(int valueV, int valueA);
void Data_TX (void);
void Data_RX (void);

void MSB_Off(void);
void MSB_On(void);

#define WAKE_UP_UART_CODE       0xBE
#define WAKE_UP_UART_CODE2      0xEF
#define SLEEP_MODE_UART_CODE    0xDE
#define SLEEP_MODE_UART_CODE2   0xAD
#define MIDDLE_BUTTON_CODE      0x80
#define INVALID_GESTURE         0xFD
#define GESTURE_START           0xFC
#define GESTURE_STOP            0xFB
#define COUNTER_CLOCKWISE       1
#define CLOCKWISE               2
#define GESTURE_POSITION_OFFSET 0x20
#define WHEEL_POSITION_OFFSET   0x30

#define WHEEL_TOUCH_DELAY		    12			    //Delay between re-sendings of touches
#define MAX_IDLE_TIME           200
#define PROXIMITY_THRESHOLD     60

#define MaxDACSetting           330
#define MinDACSetting           0;

unsigned int wheel_position=ILLEGAL_SLIDER_WHEEL_POSITION, last_wheel_position=ILLEGAL_SLIDER_WHEEL_POSITION;
unsigned int deltaCnts[4];
unsigned int prox_raw_Cnts;

//int I2C_State, Bytecount, Transmit = 0;     // State variable
int dataLedMsg[6] = {0x20,0x00,0x00,0x00,0x00,0x00};
int dataLedSegments[2] = {0x24, 0x00};
int dataDACMsg[3] = {0x00,0x00,0x00};
int dataINAValue[4] = {0x00,0x00,0x00,0x00};
int firstTouchLocation = 0;
int lastPosition = 0;
int voltage = 1023;
int firstTouch = 1;
int useAsSlider = 0;
int powerOn = 0;
int ledCycle = 0;
int mode = 0;
int blinkUpdate = 0;
int milidelay = 0;
int busVoltage = 0;
int shuntVoltage = 0;
int decimalPlace = 0;
int miliDecimal = 0;
int msbOn = 0;

#define MODE_VOLTS                   0x00
#define MODE_AMPS                    0x01
#define MODE_WATTS                   0x02

#define MODE_OFF                   0x00
#define MODE_ON                    0x01
#define MODE_FIXED		           0x02

#define MASK7                   BIT4
#define MASK6                   BIT5
#define MASK5                   BIT6
#define MASK4                   BIT7

#define MASK3                   (BIT3+BIT4+BIT5+BIT6)
#define MASK2                   (BIT3+BIT4+BIT5+BIT7)
#define MASK1                   (BIT3+BIT4+BIT6+BIT7)
#define MASK0                   (BIT3+BIT5+BIT6+BIT7)

const unsigned char LedWheelPosition[16] = 
                                {
                                  MASK0, MASK0, MASK0 & MASK1, MASK1,
                                  MASK1 & MASK2, MASK2, MASK2 & MASK3, MASK3,
                                  MASK4, MASK4, MASK4 | MASK5, MASK5, 
                                  MASK5 | MASK6,  MASK6, MASK6 | MASK7, MASK7 
                                };
const unsigned char startSequence[8] = 
                                {
								    MASK0,
								    MASK1,
								    MASK2,
								    MASK3,
								    MASK4,
								    MASK5,
								    MASK6,
								    MASK7
								   };
/*----------------- LED definition------------------------------*/
 
/* ----------------InitPuppy--------------------------------------
 * Setup initial clock and port settings
 *
 * ------------------------------------------------------------------------*/
void InitPuppy(void)

{
  BCSCTL1 |= DIVA_0;                    // ACLK/(0:1,1:2,2:4,3:8)
  BCSCTL3 |= LFXT1S_2;                  // LFXT1 = VLO  
  
  // Port init
  P1OUT |= (BIT6+BIT7);
  P1OUT &= ~BIT5;
  P1DIR |= BIT5;  // P1.5 as outputs
  P1REN |= 0xC0;   // P1.6 & P1.7 Pullups, others to 0
}

/* ----------------InitPuppy--------------------------------------
 * Setup initial clock and port settings
 *
 * ------------------------------------------------------------------------*/
void InitLedDriver(void)

{
	dataINAValue[0] = 0x04;		//Configuration Register
	dataINAValue[1] = 0x01;		//Normal operation mode (defaults to 0x00 which is shutdown mode)
	I2C_Master_Transmit(LED_SLV_ADDR,dataINAValue,2);

	dataINAValue[0] = 0x01; 	//Decode Mode Register
	dataINAValue[1] = 0x0F;		//Decode Digits 1,2,3,4 (defaults to 0x00, for no decode)
	I2C_Master_Transmit(LED_SLV_ADDR,dataINAValue,2);

	dataINAValue[0] = 0x02;		//Intensity Regiter
	dataINAValue[1] = 0x2A;		//Set to 42/64(15.1mA) of Max (22.6mA)
	I2C_Master_Transmit(LED_SLV_ADDR,dataINAValue,2);

	dataINAValue[0] = 0x24;		//Segments Register
	dataINAValue[1] = 0x00;		//All segments off
	I2C_Master_Transmit(LED_SLV_ADDR,dataINAValue,2);
}

void InitDAC(void)

{
	 P1OUT &= ~BIT5;  //Turns off the Output by shutting down brownout detector
	 P1OUT |= BIT5;
	 dataDACMsg[0] = 0x35;  //End
	 dataDACMsg[1] = 0x00;
	 dataDACMsg[2] = 0x00;
	 I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);

	dataDACMsg[0] = 0x2D;  //Ref on/output/2.5v
	dataDACMsg[1] = 0x00;
	dataDACMsg[2] = 0x00;
	I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);

	dataDACMsg[0] = 0x00;  //Ref on/output/2.5v
		 	dataDACMsg[1] = 0x00;
		 	dataDACMsg[2] = 0x00;
		 I2C_Master_Recieve(DAC_SLV_ADDR,dataDACMsg,3);

	dataDACMsg[0] = 0x40;  //Normal Power operation
		dataDACMsg[1] = 0x00;
		dataDACMsg[2] = 0x00;
		I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);

		dataDACMsg[0] = 0x60;  //Set output to full on reset
				dataDACMsg[1] = 0x00;
				dataDACMsg[2] = 0x60;
				I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);

	dataDACMsg[0] = 0x50;  //Config Aux off
	dataDACMsg[1] = 0x00;
	dataDACMsg[2] = 0x38;
	I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);

	dataDACMsg[0] = 0xA0;  //CODE and LOAD Command
	dataDACMsg[1] = 0xFF; //top 10 bytes are the only ones used, so this is full on )which equalls puppy power output at minimum
	dataDACMsg[2] = 0xC0;
	I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);

	voltage = 1023;
	          dataDACMsg[0] = 0xA0;  //CODE and LOAD Command
	          dataDACMsg[1] = (voltage >> 2) & 0x00FF; //top 10 bytes are the only ones used, so this is full on )which equalls puppy power output at minimum
	          dataDACMsg[2] = (voltage << 6) & 0x00FF;
	          I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);

	          dataDACMsg[0] = 0xA0;  //CODE and LOAD Command
	          	dataDACMsg[1] = 0x55; //top 10 bytes are the only ones used, so this is full on )which equalls puppy power output at minimum
	          	dataDACMsg[2] = 0xC0;
	          	I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);

	          	voltage = 500;
	          	          dataDACMsg[0] = 0xA0;  //CODE and LOAD Command
	          	          dataDACMsg[1] = (voltage >> 2) & 0x00FF; //top 10 bytes are the only ones used, so this is full on )which equalls puppy power output at minimum
	          	          dataDACMsg[2] = (voltage << 6) & 0x00FF;
	          	          I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);
	          	          voltage = 1;
	          	        dataDACMsg[0] = 0x30;  //End
	          	        		dataDACMsg[1] = 0x00;
	          	        		dataDACMsg[2] = 0x00;
	          	        		I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);

}

/* ----------------UpdateLeds--------------------------------------
 * Update the UI LED's based on the system state
 *
 * ------------------------------------------------------------------------*/
void UpdateLeds()
{
	if(powerOn == MODE_ON)
			{
		dataLedSegments[1] |= BIT3;
		P1OUT |= BIT5;
			}
	else if(powerOn == MODE_FIXED)
			{
		P1OUT |= BIT5;
				if (blinkUpdate < 3)
				{
					dataLedSegments[1] |= BIT3;
				}
				else
				{
					dataLedSegments[1] &= ~BIT3;
				}
				blinkUpdate++;
				if(blinkUpdate >4)
					blinkUpdate = 0;
	}
	else
	{
		P1OUT &= ~BIT5;
		dataLedSegments[1] &= ~BIT3;
	}
	if(mode == MODE_WATTS)
		dataLedSegments[1] |= BIT2;
	else
		dataLedSegments[1] &= ~BIT2;
	if(mode == MODE_AMPS)
		dataLedSegments[1] |= BIT1;
	else
		dataLedSegments[1] &= ~BIT1;
	if(mode == MODE_VOLTS)
		dataLedSegments[1] |= BIT0;
	else
		dataLedSegments[1] &= ~BIT0;

	I2C_Master_Transmit(LED_SLV_ADDR,dataLedSegments,2);
}

 
/* ----------------MeasureCapBaseLine--------------------------------------
 * Re-measure the baseline capacitance of the wheel elements & the center  
 * button. To be called after each wake up event.                          
 * ------------------------------------------------------------------------*/
void MeasureCapBaseLine(void)
{
  //P1OUT = BIT0;
  /* Set DCO to 8MHz */
  /* SMCLK = 8MHz/8 = 1MHz */
  BCSCTL1 = CALBC1_8MHZ;     
  DCOCTL = CALDCO_8MHZ;
  BCSCTL2 |= DIVS_3;
  
  TI_CAPT_Init_Baseline(&voltage_slider);
  TI_CAPT_Update_Baseline(&voltage_slider,2);
  TI_CAPT_Init_Baseline(&on_off_button);
  TI_CAPT_Update_Baseline(&on_off_button,2);
  TI_CAPT_Init_Baseline(&measurement_button);
   TI_CAPT_Update_Baseline(&measurement_button,2);
}




void main(void)
{   
  WDTCTL = WDTPW + WDTHOLD;             // Stop watchdog timer
  
  InitPuppy();
  InitLedDriver();
  InitDAC();

  //I2C_State = 0;
  
  /* Set DCO to 1MHz */
  /* Set SMCLK to 1MHz / 8 = 125kHz */
  BCSCTL1 = CALBC1_1MHZ;                
  DCOCTL = CALDCO_1MHZ;
  BCSCTL2 |= DIVS_3;

  /* Set the DAC to max voltage to set the output voltage to a minimum value */
  voltage = 1023;
  //dataDACMsg[0] = (voltage >> 8);
  //dataDACMsg[1] = voltage & 0x00FF;
  //I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,2);




        P1OUT |= BIT5;
        //P1OUT &= ~BIT5;



  /* Setup the INA 219 power monitor */
  dataINAValue[0] = 0x05;
  dataINAValue[1] = 0x11;
  dataINAValue[2] = 0xC7;
  I2C_Master_Transmit(INA_SLV_ADDR,dataINAValue,3);

/* Establish baseline for all the sensor */
  MeasureCapBaseLine();

  unsigned char measureButtonTouched = 0;
  unsigned char powerButtonTouched = 0;
  int updateDelay = 0;

  /* Set DCO to 8MHz */
  /* SMCLK = 8MHz/8 = 1MHz */
  BCSCTL1 = CALBC1_8MHZ;
  DCOCTL = CALDCO_8MHZ;
  BCSCTL2 |= DIVS_3;
  TACCTL0 &= ~CCIE;

  voltage = 500;
  while (1)
  {
/*
	  if(voltage == 1023)
		  up = 0;
	  if(voltage == 0)
		  up = 1;

	  if(up)
		  voltage++;
	  else
		  voltage--;
	  //voltage = 1023;

		  	          dataDACMsg[0] = 0xA0;  //CODE and LOAD Command
		  	          dataDACMsg[1] = (voltage >> 2) & 0x00FF; //top 10 bytes are the only ones used, so this is full on )which equalls puppy power output at minimum
		  	          dataDACMsg[2] = (voltage << 6) & 0x00FF;
		  	          I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);

*/



	  wheel_position = ILLEGAL_SLIDER_WHEEL_POSITION;
      wheel_position = TI_CAPT_Slider(&voltage_slider);
      if (updateDelay > 28) //was 28
      {
          updateDelay = 0;

          //Read Shunt Voltage
          dataINAValue[0] = 0x01;
          I2C_Master_Transmit(INA_SLV_ADDR,dataINAValue,1);
          dataINAValue[0] = 0xFFFF;
          dataINAValue[1] = 0xFFFF;
          I2C_Master_Recieve(INA_SLV_ADDR,dataINAValue,2);
          dataINAValue[0] = dataINAValue[0] << 8;
          dataINAValue[0] = dataINAValue[0] + dataINAValue[1];
          shuntVoltage = dataINAValue[0];

          //Read Bus Voltage
          dataINAValue[0] = 0x02;
          I2C_Master_Transmit(INA_SLV_ADDR,dataINAValue,1);
          dataINAValue[0] = 0xFFFF;
          dataINAValue[1] = 0xFFFF;
          I2C_Master_Recieve(INA_SLV_ADDR,dataINAValue,2);
          dataINAValue[0] = dataINAValue[0] << 8;
          dataINAValue[0] = dataINAValue[0] + dataINAValue[1];
          dataINAValue[0] = dataINAValue[0] >> 3;
          busVoltage = dataINAValue[0];
      }
      if ((updateDelay % 30) == 0) //was 28
            {
    	  UpdateLeds();
      Contruct_Send_Message(busVoltage, shuntVoltage);
            }
      updateDelay++;

      if(wheel_position != ILLEGAL_SLIDER_WHEEL_POSITION)
      {
    	  int sliderDelta;
          if(firstTouch == 1)
          {
        	  firstTouchLocation = wheel_position;
        	  lastPosition = wheel_position;
        	  firstTouch = 0;
          }

          if(abs(wheel_position - firstTouchLocation) > 3)
        	  useAsSlider = 1;

          sliderDelta = lastPosition - wheel_position;
          if(powerOn == MODE_FIXED || useAsSlider == 0)
        	  sliderDelta = 0;

          lastPosition = wheel_position;
          voltage +=  1 * sliderDelta;
          if(voltage < 0 )
        	  voltage = 0;
          if( voltage > 1023)
        	  voltage = 1023;


          dataDACMsg[0] = 0xA0;  //CODE and LOAD Command
          dataDACMsg[1] = (voltage >> 2) & 0x00FF; //top 10 bytes are the only ones used, so this is full on )which equalls puppy power output at minimum
          dataDACMsg[2] = (voltage << 6) & 0x00FF;
          I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);
      }
      else  /* no slider position was detected */
      {
    	  if(powerOn == MODE_ON &&  firstTouch == 0 && ((firstTouchLocation - lastPosition) < 3 && (firstTouchLocation - lastPosition) > -3) ) //finger was lifted
    	  {
    		  if(firstTouchLocation < 8)
    		  {
    			  voltage +=  4;
    			  if(voltage < 0 )
    				  voltage = 0;
      		      if( voltage > 1023)
      		    	  voltage = 1023;


      		    dataDACMsg[0] = 0xA0;  //CODE and LOAD Command
      		  dataDACMsg[1] = (voltage >> 2) & 0x00FF; //top 10 bytes are the only ones used, so this is full on )which equalls puppy power output at minimum
      		dataDACMsg[2] = (voltage << 6) & 0x00FF;
      		    	I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);

    		  }
    		  else if(firstTouchLocation > 16)
    		  {
    			  voltage -=  4;
    			  if(voltage < 0 )
    				  voltage = 0;
      		      if( voltage > 1023)
      		    	  voltage = 1023;


      		    dataDACMsg[0] = 0xA0;  //CODE and LOAD Command
      		          		  dataDACMsg[1] = (voltage >> 2) & 0x00FF; //top 10 bytes are the only ones used, so this is full on )which equalls puppy power output at minimum
      		          		dataDACMsg[2] = (voltage << 6) & 0x00FF;
      		          		    	I2C_Master_Transmit(DAC_SLV_ADDR,dataDACMsg,3);
    		  }
    	  }
    	  firstTouch = 1;
      }
      if(TI_CAPT_Button(&on_off_button))
      { /* on_off button was touched */
    	  if (measureButtonTouched==0)
    	  {
    		  measureButtonTouched = 1;
    		  if(powerOn == MODE_OFF)
    			  powerOn = MODE_ON;
    		  else if(powerOn == MODE_ON)
    			  powerOn = MODE_FIXED;
    		  else if(powerOn == MODE_FIXED)
    			  powerOn = MODE_OFF;
    	  }
      }
      else
      {
    	  measureButtonTouched = 0;
      }
      if(TI_CAPT_Button(&measurement_button))
      { /* on_off button was touched */
    	  if (powerButtonTouched==0)
    	  {
    		  powerButtonTouched = 1;
    		  switch(mode)
    		  {
    		  case MODE_VOLTS:
    			  mode = MODE_AMPS;
                  break;
    		  case MODE_AMPS:
    			  mode = MODE_WATTS;
    			  break;
    		  case MODE_WATTS:
    			  mode = MODE_VOLTS;
    			  break;
    		  }
    	  }
      }
      else
      {
    	  powerButtonTouched = 0;
      }
        // Reset all touch conditions
        last_wheel_position= ILLEGAL_SLIDER_WHEEL_POSITION;

    /* ------------------------------------------------------------------------
     * Option:
     * Add delay/sleep cycle here to reduce active duty cycle. This lowers power
     * consumption but sacrifices slider responsiveness. Additional timing
     * refinement must be taken into consideration when interfacing with PC
     * applications GUI to retain proper communication protocol.
     * -----------------------------------------------------------------------*/
  }
}



void Contruct_Send_Message(int valueV, int valueA)
{
	int a;
	long dvalue;
	int decimal;
	milidelay++;
	if (powerOn == MODE_OFF)
	{
		valueV = 0;
		valueA = 0;
	}
	if(valueA <0)
		valueA = 0;
	if(milidelay > 3)
		milidelay = 0;
	if(mode == MODE_VOLTS)
	{
		valueV = 4 * valueV;
		dataLedMsg[4] = valueV/10000;
		a = valueV/10000;
		valueV = valueV - 10000*(a);
		dataLedMsg[3] = valueV/1000;
		a = valueV/1000;
		valueV = valueV - 1000*(a);
		dataLedMsg[2] = valueV/100;
		a = valueV/100;
		valueV = valueV - 100*(a);
		dataLedMsg[1] = valueV/10;
		a = valueV/10;
		valueV = valueV - 10*(a);
		decimalPlace = 0x40;
		miliDecimal = 0;
		MSB_Off();

		//dataLedSegments[1] |= 0x80;
	}
	else if(mode == MODE_WATTS)
	{

		decimal = 2;
		if(milidelay >1)
			decimal = 0;
		dvalue = (long)valueA * 667;  	//I = V/R
										//for .18ohm   I = V * (1/.18) * 10  =  I = V * 556
										//for .15ohm   I = V * (1/.15) * 10  =  I = V * 667
		dvalue = (dvalue / 100);
		dvalue = dvalue * (long)4 * (long)valueV;
		dvalue = (dvalue / 100);
		if( dvalue > 9999999)
		{
			dvalue = (dvalue / 10000);
			decimal = 3;
			if(milidelay >1)
							decimal = 0;
		}
		else if( dvalue > 999999)
		{
			dvalue = (dvalue / 1000);
			decimal = 2;
		}
		else if( dvalue > 99999)
		{
			dvalue = (dvalue / 100);
			decimalPlace = 0x80;
						miliDecimal = 1;
			decimal = 4;
			if(milidelay >1)
				decimal = 0;
		}
		else if( dvalue > 9999)
		{
			dvalue = (dvalue / 10);
			decimalPlace = 0x40;
			miliDecimal = 1;
			//decimal = 3;
			//if(milidelay >1)
				//decimal = 0;
		}
		valueA = (int)dvalue;
		//if(valueA > 999)
		dataLedMsg[4] = valueA/1000;
		//else
		//	dataSAAMsg[2] = ledNumbers[0]; // Turn off first LED
		a = valueA/1000;
		valueA = valueA - 1000*(a);
		dataLedMsg[3] = valueA/100;
		a = valueA/100;
		valueA = valueA - 100*(a);
		dataLedMsg[2] = valueA/10;
		a = valueA/10;
		valueA = valueA - 10*(a);
		dataLedMsg[1] = valueA/1;
		a = valueA/1;
		valueA = valueA - 1*(a);
		if(decimal != 0)
			dataLedSegments[1] |= 0x80;
		MSB_Off();
	}
	else if(mode == MODE_AMPS)
	{
		MSB_Off();
		decimal = 3;
		if(milidelay >1)
								decimal = 0;
		//if(milidelay >1)
		//	decimal = 3;
		dvalue = (long)valueA * 667;
		dvalue = (dvalue / 100);
		if( dvalue > 99999)
		{
			dvalue = (dvalue / 100);
			decimal = 2;
		}
		else if( dvalue > 9999)
		{
			dvalue = (dvalue / 10);
			decimal = 4;
			if(milidelay >1)
						decimal = 0;
		}
		valueA = (int)dvalue;
		if(valueA > 999)
			dataLedMsg[2] = valueA/1000;
		//else
		//	dataSAAMsg[2] = ledNumbers[10]; // Turn off first LED
		a = valueA/1000;
		valueA = valueA - 1000*(a);
		dataLedMsg[3] = valueA/100;
		a = valueA/100;
		valueA = valueA - 100*(a);
		dataLedMsg[4] = valueA/10;
		a = valueA/10;
		valueA = valueA - 10*(a);
		dataLedMsg[5] = valueA/1;
		a = valueA/1;
		valueA = valueA - 1*(a);
		if(decimal != 0)
			dataLedMsg[decimal] |= 0x80;
		MSB_Off();

		//DEBUG READ DAQ SETTINGS
		/*
				valueV = voltage * 10;
				dataLedMsg[4] = valueV/10000;
				a = valueV/10000;
				valueV = valueV - 10000*(a);
				dataLedMsg[3] = valueV/1000;
				a = valueV/1000;
				valueV = valueV - 1000*(a);
				dataLedMsg[2] = valueV/100;
				a = valueV/100;
				valueV = valueV - 100*(a);
				dataLedMsg[1] = valueV/10;
				a = valueV/10;
				valueV = valueV - 10*(a);

				dataLedSegments[1] |= 0x80;
				*/
	}
	//Blink Decimal Place if units are mili
	dataLedSegments[1] &= 0x0F;
	if(miliDecimal == 0 || milidelay > 1)
	dataLedSegments[1] |= decimalPlace;
	//update display
	dataLedMsg[5] = dataLedSegments[1];
	I2C_Master_Transmit(LED_SLV_ADDR,dataLedMsg,6);
}

void MSB_Off(void)
{
	if(msbOn)
	{
		//Turn off first segment
				dataINAValue[0] = 0x01; 	//Decode Mode Register
				dataINAValue[1] = 0x07;		//Decode Digits 1,2,3,4 (defaults to 0x00, for no decode)
				I2C_Master_Transmit(LED_SLV_ADDR,dataINAValue,2);
	}
}

void MSB_On(void)
{
	if(!msbOn)
	{
		//Turn off first segment
				dataINAValue[0] = 0x01; 	//Decode Mode Register
				dataINAValue[1] = 0x08;		//Decode Digits 1,2,3,4 (defaults to 0x00, for no decode)
				I2C_Master_Transmit(LED_SLV_ADDR,dataINAValue,2);
	}
}

