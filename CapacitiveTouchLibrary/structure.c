/*******************************************************************************
 *
 *  structure.c    
 *
 *  Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 ******************************************************************************/
//******************************************************************************
// structure.c
//
// 3 elements configured as a slider.
// 1 element configured as a button. [Center Button] 
// 1 element configured as a proximity sensor.
// This file contains the structure names and the variable assingments.
//
// Revision 1.00 Baseline
// Developed with IAR 5.10.4 [Kickstart] (5.10.4.30168)
//
// 09/24/10 Rev1.00 Pre-Alpha Version. Added "max_cnts" slider variable in 
//                  CT_Handler object.
//
// 09/29/10 0.02    Update naming convention for elements and sensors.
//
// 10/11/10 0.03    Update
//
//******************************************************************************

#include "structure.h"

//PinOsc Slider right element P2.3
const struct Element slider_right_element = {

              .inputPxselRegister = (uint8_t *)&P2SEL,
              .inputPxsel2Register = (uint8_t *)&P2SEL2,
              .inputBits = BIT3,
              // measured for a 1Mhz SMCLK
              .maxResponse = 280, // actual measure was 980
              .threshold = 5

};      

//PinOsc Slider middle element P2.4
const struct Element slider_middle_element = {

              .inputPxselRegister = (uint8_t *)&P2SEL,
              .inputPxsel2Register = (uint8_t *)&P2SEL2,
              .inputBits = BIT4,
              // 1Mhz SMCLK
              .maxResponse = 250,
              .threshold = 10
};

//PinOsc Volume up, P2.5
const struct Element slider_left_element = {

              .inputPxselRegister = (uint8_t *)&P2SEL,
              .inputPxsel2Register = (uint8_t *)&P2SEL2,
              .inputBits = BIT5,
              // 1Mhz SMCLK
              .maxResponse = 280,
              .threshold = 5
};      


//PinOsc On Off Button P2.1
const struct Element on_off_element = {

              .inputPxselRegister = (uint8_t *)&P2SEL,
              .inputPxsel2Register = (uint8_t *)&P2SEL2,
              .inputBits = BIT1,
              // When using an abstracted function to measure the element
              // the 100*(maxResponse - threshold) < 0xFFFF
              // ie maxResponse - threshold < 655
              .maxResponse = 350+655,
              .threshold = 350 
}; 

//PinOsc Measurement Button P2.0
const struct Element measurement_element = {

              .inputPxselRegister = (uint8_t *)&P2SEL,
              .inputPxsel2Register = (uint8_t *)&P2SEL2,
              .inputBits = BIT0,
              .maxResponse = 350+655,
              .threshold = 350
};      

//*** CAP TOUCH HANDLER *******************************************************/
// This defines the grouping of sensors, the method to measure change in
// capacitance, and the function of the group

const struct Sensor voltage_slider =
               { 
                  //.halDefinition = fRO_PINOSC_TA0_SW,
                  .halDefinition = RO_PINOSC_TA0_WDTp,
                  .numElements = 3,
                  .points = 24,
                  .sensorThreshold = 20,
                  .baseOffset = 0,
                  // Pointer to elements
                  .arrayPtr[0] = &slider_left_element,  // point to first element
                  .arrayPtr[1] = &slider_middle_element,
                  .arrayPtr[2] = &slider_right_element,
                  //.arrayPtr[3] = &left,
                  // Timer Information
                  //.measGateSource= GATE_WDT_SMCLK,     //  0->SMCLK, 1-> ACLK
                  //.accumulationCycles= WDTp_GATE_32768             //32768
                  //.accumulationCycles= WDTp_GATE_8192               // 8192
                  //.accumulationCycles= WDTp_GATE_512             //512
                  .accumulationCycles= WDTp_GATE_64             //64
                  //.accumulationCycles = 32                                          
               };

const struct Sensor on_off_button =
               { 
                  .halDefinition = RO_PINOSC_TA0_WDTp,
                  .numElements = 1,
                  .baseOffset = 3,
                  // Pointer to elements
                  .arrayPtr[0] = &on_off_element,  // point to first element
                  // Timer Information
                  .measGateSource= GATE_WDT_SMCLK,     //  0->SMCLK, 1-> ACLK
                  //.accumulationCycles= WDTp_GATE_32768             //32768
                  //.accumulationCycles= WDTp_GATE_8192               // 8192
                  .accumulationCycles= WDTp_GATE_512             //512
                  //.accumulationCycles= WDTp_GATE_64             //64                  
               };

const struct Sensor measurement_button =
               { 
                  .halDefinition = RO_PINOSC_TA0_WDTp,
                  .numElements = 1,
                  .baseOffset = 4,
                  // Pointer to elements
                  .arrayPtr[0] = &measurement_element,  // point to first element
                  // Timer Information
                  .measGateSource= GATE_WDT_SMCLK,     //  0->SMCLK, 1-> ACLK
                  //.accumulationCycles= WDTp_GATE_32768             //32768
                  //.accumulationCycles= WDTp_GATE_8192               // 8192
                  .accumulationCycles= WDTp_GATE_512             //512
                  //.accumulationCycles= WDTp_GATE_64             //64                  
               };
