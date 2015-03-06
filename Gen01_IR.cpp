/* mbed ROBOT Library, for SUTD evobot project, Generation 1
 * Copyright (c) 2014, SUTD
 * Author: Yashwanth Tumuluri
 *
 * Apr 15, 2014
 *
 * This library allows the Evobot Generation 1 to extract data from the IR sensor
 * 
 */
#include "mbed.h"
#include "robot.h"

Robot IR;

int begin,end;

int main()
{
    int C = 14;
    double D = -1.1;
    while(!IR.bt.readable()) IR.bt.printf("Type something\n\r");
    while(1){ 
        double IR_distance = (pow(IR.IRsensor(),D)* C )+ 4;
        double IR_distance2 = (pow(IR.IRsensor2(),D)* C )+ 4;
        double IR_distance3 = (pow(IR.IRsensor3(),D)* C )+ 4;
        IR.bt.printf("\nIR1 reading is = %lf \n\r",IR_distance); 
        IR.bt.printf("IR2 reading is = %lf \n\r",IR_distance2);
        IR.bt.printf("IR3 reading is = %lf \n\r",IR_distance3);
        wait(2);
   
    } 

 }