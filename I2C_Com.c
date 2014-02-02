/*
 * I2C_Com.c
 *
 *  Created on: Mar 24, 2013
 *      Author: Jeff
 */

/******************************************************
// USI interrupt service routine
// Data Transmit : state 0 -> 2 -> 4 -> 10 -> 12 -> 14
// Data Recieve  : state 0 -> 2 -> 4 -> 6 -> 8 -> 14
******************************************************/
#include "I2C_Com.h"

int I2C_State, Bytecount, Transmit = 0;
int number_of_bytes = 6;

int* I2C_data;
char SLV_Addr;

void I2C_Master_Transmit(char address, int* data, int dataLength)
{
	SLV_Addr = address;
	I2C_State = 0;
	number_of_bytes = dataLength;
	I2C_data = data;
	Master_Transmit();
}

void I2C_Master_Recieve(char address, int* data, int dataLength)
{
	SLV_Addr = address;
	I2C_State = 0;
	number_of_bytes = dataLength;
	I2C_data = data;
	Master_Recieve();
}

#pragma vector = USI_VECTOR
__interrupt void USI_TXRX (void)
{
  switch(__even_in_range(I2C_State,14))
    {
      case 0: // Generate Start Condition & send address to slave
              //P1OUT |= 0x01;                // LED on: sequence start
              Bytecount = 0;
              USISRL = 0x00;                // Generate Start Condition...
              USICTL0 |= USIGE+USIOE;
              USICTL0 &= ~USIGE;
              if (Transmit == 1){
                USISRL = SLV_Addr;              // Address is 0x48 << 1 bit + 0 (rw)
              }
               if (Transmit == 0){
               USISRL = SLV_Addr + 1;               // 0x91 Address is 0x48 << 1 bit
                                            // + 1 for Read
               }
              USICNT = (USICNT & 0xE0) + 0x08; // Bit counter = 8, TX Address
              I2C_State = 2;                // next state: rcv address (N)Ack
              break;

      case 2: // Receive Address Ack/Nack bit
              USICTL0 &= ~USIOE;            // SDA = input
              USICNT |= 0x01;               // Bit counter=1, receive (N)Ack bit
              I2C_State = 4;                // Go to next state: check (N)Ack
              break;

      case 4: // Process Address Ack/Nack & handle data TX

 if(Transmit == 1){
              USICTL0 |= USIOE;             // SDA = output
              if (USISRL & 0x01)            // If Nack received...
              { // Send stop...
                USISRL = I2C_data[Bytecount];
                USICNT |=  0x01;            // Bit counter=1, SCL high, SDA low
                I2C_State = 14;             // Go to next state: generate Stop
                //P1OUT |= 0x01;              // Turn on LED: error
              }
              else
              { // Ack received, TX data to slave...
              USISRL = I2C_data[Bytecount];            // Load data byte
              USICNT |=  0x08;              // Bit counter = 8, start TX
              I2C_State = 10;               // next state: receive data (N)Ack
              Bytecount++;
              //P1OUT &= ~0x01;               // Turn off LED
              break;
              }
 } if(Transmit == 0){

               if (USISRL & 0x01)            // If Nack received
              { // Prep Stop Condition
                USICTL0 |= USIOE;
                USISRL = 0x00;
                USICNT |=  0x01;            // Bit counter= 1, SCL high, SDA low
                I2C_State = 8;              // Go to next state: generate Stop
                //P1OUT |= 0x01;              // Turn on LED: error
              }
              else{ Data_RX();}             // Ack received


}
              break;

case 6: // Send Data Ack/Nack bit
              USICTL0 |= USIOE;             // SDA = output
              I2C_data[Bytecount] = USISRL;
              if (Bytecount <= number_of_bytes-2)
              {                             // If this is not the last byte

                USISRL = 0x00;              // Send Ack
                //P1OUT &= ~0x01;             // LED off
                I2C_State = 4;              // Go to next state: data/rcv again
                Bytecount++;
                }

              else //last byte: send NACK
              {
                USISRL = 0xFF;              // Send NAck
                //P1OUT |= 0x01;              // LED on: end of comm
                I2C_State = 8;              // stop condition
              }
              USICNT |= 0x01;               // Bit counter = 1, send (N)Ack bit
              break;

      case 8: // Prep Stop Condition
              USICTL0 |= USIOE;             // SDA = output
              USISRL = 0x00;
              USICNT |=  0x01;              // Bit counter= 1, SCL high, SDA low
              I2C_State = 14;               // Go to next state: generate Stop
              break;

      case 10: // Receive Data Ack/Nack bit
              USICTL0 &= ~USIOE;            // SDA = input
              USICNT |= 0x01;               // Bit counter = 1, receive (N)Ack bit
              I2C_State = 12;               // Go to next state: check (N)Ack
              break;

      case 12: // Process Data Ack/Nack & send Stop
              USICTL0 |= USIOE;
              if (Bytecount == number_of_bytes){// If last byte
              USISRL = 0x00;

              I2C_State = 14;               // Go to next state: generate Stop
              //P1OUT |= 0x01;
              USICNT |=  0x01;     }        // set count=1 to trigger next state
              else{
                //P1OUT &= ~0x01;             // Turn off LED
                Data_TX();                  // TX byte
              }
              break;

      case 14:// Generate Stop Condition
              USISRL = 0x0FF;               // USISRL = 1 to release SDA
              USICTL0 |= USIGE;             // Transparent latch enabled
              USICTL0 &= ~(USIGE+USIOE);    // Latch/SDA output disabled
              I2C_State = 0;                // Reset state machine for next xmt
              LPM0_EXIT;                    // Exit active for next transfer
              break;
    }

  USICTL1 &= ~USIIFG;                       // Clear pending flag
}


void Data_TX (void){

              USISRL = I2C_data[Bytecount];          // Load data byte
              USICNT |=  0x08;              // Bit counter = 8, start TX
              I2C_State = 10;               // next state: receive data (N)Ack
              Bytecount++;
}

void Data_RX (void){
	USICTL0 &= ~USIOE;                  // SDA = input --> redundant

        USICNT |=  0x08;                    // Bit counter = 8, RX data
        I2C_State = 6;                      // Next state: Test data and (N)Ack
        //P1OUT &= ~0x01;                     // LED off
        }


void Setup_USI_Master_TX (void)
{
  _DINT();
  Bytecount = 0;
  Transmit = 1;
  USICTL0 = USIPE6+USIPE7+USIMST+USISWRST;  // Port & USI mode setup
  USICTL1 = USII2C+USIIE;                   // Enable I2C mode & USI interrupt
  USICKCTL = USIDIV_7+USISSEL_2+USICKPL;    // USI clk: SCL = SMCLK/128
  USICNT |= USIIFGCC;                       // Disable automatic clear control
  USICTL0 &= ~USISWRST;                     // Enable USI
  USICTL1 &= ~USIIFG;                       // Clear pending flag
  _EINT();
}


void Setup_USI_Master_RX (void)
{
  _DINT();
  Bytecount = 0;
  Transmit = 0;
  USICTL0 = USIPE6+USIPE7+USIMST+USISWRST;  // Port & USI mode setup
  USICTL1 = USII2C+USIIE;                   // Enable I2C mode & USI interrupt
  USICKCTL = USIDIV_7+USISSEL_2+USICKPL;    // USI clks: SCL = SMCLK/128
  USICNT |= USIIFGCC;                       // Disable automatic clear control
  USICTL0 &= ~USISWRST;                     // Enable USI
  USICTL1 &= ~USIIFG;                       // Clear pending flag
  _EINT();

}

void Master_Transmit(void){
Setup_USI_Master_TX();
    USICTL1 |= USIIFG;                      // Set flag and start communication
    LPM0;                                   // CPU off, await USI interrupt
   // __delay_cycles(1000);                  // Delay between comm cycles
}
void Master_Recieve(void){
  Setup_USI_Master_RX();
  USICTL1 |= USIIFG;                        // Set flag and start communication
  LPM0;                                     // CPU off, await USI interrupt
 // __delay_cycles(1000);                    // Delay between comm cycles
}


