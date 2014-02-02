/*
 * I2C_Com.h
 *
 *  Created on: Mar 24, 2013
 *      Author: Jeff
 */
#include <msp430.h>

#ifndef I2C_COM_H_
#define I2C_COM_H_



void Master_Transmit(void);
void Master_Recieve(void);
void I2C_Master_Transmit(char address,int* data, int dataLength);
void I2C_Master_Recieve(char address, int* data, int dataLength);

void Setup_USI_Master_TX(void);
void Setup_USI_Master_RX(void);

void Data_TX (void);
void Data_RX (void);


#endif /* I2C_COM_H_ */
