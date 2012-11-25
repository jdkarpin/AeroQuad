/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#ifndef _AQ_FOURTH_ORDER_FILTER_H_
#define _AQ_FOURTH_ORDER_FILTER_H_

////////////////////////////////////////////////////////////////////////////////
//
//
////////////////////////////////////////////////////////////////////////////////

#include <GlobalDefined.h>


struct fourthOrderData
{
  float  inputTm1,  inputTm2,  inputTm3,  inputTm4;
  float outputTm1, outputTm2, outputTm3, outputTm4;
} fourthOrder[4];

float computeFourthOrder(float currentInput, struct fourthOrderData *filterParameters)
{
  // cheby2(4,60,12.5/50)
  #define _b0  0.001893594048567
  #define _b1 -0.002220262954039
  #define _b2  0.003389066536478
  #define _b3 -0.002220262954039
  #define _b4  0.001893594048567
  
  #define _a1 -3.362256889209355
  #define _a2  4.282608240117919
  #define _a3 -2.444765517272841
  #define _a4  0.527149895089809
  
  float output;
  
  output = _b0 * currentInput                + 
           _b1 * filterParameters->inputTm1  + 
           _b2 * filterParameters->inputTm2  +
           _b3 * filterParameters->inputTm3  +
           _b4 * filterParameters->inputTm4  -
           _a1 * filterParameters->outputTm1 -
           _a2 * filterParameters->outputTm2 -
           _a3 * filterParameters->outputTm3 -
           _a4 * filterParameters->outputTm4;

  filterParameters->inputTm4 = filterParameters->inputTm3;
  filterParameters->inputTm3 = filterParameters->inputTm2;
  filterParameters->inputTm2 = filterParameters->inputTm1;
  filterParameters->inputTm1 = currentInput;
  
  filterParameters->outputTm4 = filterParameters->outputTm3;
  filterParameters->outputTm3 = filterParameters->outputTm2;
  filterParameters->outputTm2 = filterParameters->outputTm1;
  filterParameters->outputTm1 = output;
    
  return output;
}

void setupFourthOrder(float longitudinalAccel = 0,  float lateralAccel = 0,  float verticalAccel = 0) //4th order assumes that quad on flat surface
{
  fourthOrder[XAXIS].inputTm1 = longitudinalAccel;
  fourthOrder[XAXIS].inputTm2 = longitudinalAccel;
  fourthOrder[XAXIS].inputTm3 = longitudinalAccel;
  fourthOrder[XAXIS].inputTm4 = longitudinalAccel;
  
  fourthOrder[XAXIS].outputTm1 = longitudinalAccel;
  fourthOrder[XAXIS].outputTm2 = longitudinalAccel;
  fourthOrder[XAXIS].outputTm3 = longitudinalAccel;
  fourthOrder[XAXIS].outputTm4 = longitudinalAccel;
  
  //////////
  fourthOrder[YAXIS].inputTm1 = lateralAccel;
  fourthOrder[YAXIS].inputTm2 = lateralAccel;
  fourthOrder[YAXIS].inputTm3 = lateralAccel;
  fourthOrder[YAXIS].inputTm4 = lateralAccel;
  
  fourthOrder[YAXIS].outputTm1 = lateralAccel;
  fourthOrder[YAXIS].outputTm2 = lateralAccel;
  fourthOrder[YAXIS].outputTm3 = lateralAccel;
  fourthOrder[YAXIS].outputTm4 = lateralAccel;
  
  //////////
  fourthOrder[ZAXIS].inputTm1 = verticalAccel;//-9.8065;
  fourthOrder[ZAXIS].inputTm2 = verticalAccel;
  fourthOrder[ZAXIS].inputTm3 = verticalAccel;
  fourthOrder[ZAXIS].inputTm4 = verticalAccel;
  
  fourthOrder[ZAXIS].outputTm1 = verticalAccel;
  fourthOrder[ZAXIS].outputTm2 = verticalAccel;
  fourthOrder[ZAXIS].outputTm3 = verticalAccel;
  fourthOrder[ZAXIS].outputTm4 = verticalAccel;
}

////////////////////////////////////////////////////////////////////////////////

#endif