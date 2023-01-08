#ifndef MODBUS_H
#define MODBUS_H

//Librarys
#include <stdint.h>
#include <stdbool.h>

//Defines
#define BUFFERSIZE 50

//Variables
uint8_t incomingByte = 0;   // for incoming serial data
uint8_t modbusCommandBuffer[BUFFERSIZE];
uint8_t modbusBufferindex = 0;

//functions
bool handleNewChar(uint8_t incomingByte);

bool isCharStart(uint8_t);
bool isCharEnd(uint8_t);

bool detectFunction();

bool readOutput();      //01
bool readInput();       //02
bool readHoldingReg();  //03
bool readInputReg();    //04
bool writeOutput();     //05
bool writeReg();        //06
bool writeMultiOutput();//15
bool writeMultiReg();   //16

#endif
