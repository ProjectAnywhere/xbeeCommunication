/**************
* Authors:      Eric Refour (XBee Communication)
*               Ekta Bindlish (Motor Control) 
* Date:         April 20th 2015
* Description:  Code to send and receive data from XBee Series 2 radios. Using the XBee libary
* Useful Links: http://www.digi.com/support/kbase/kbaseresultdetl?id=3345
                http://www.farnell.com/datasheets/27606.pdf
                http://www.kobakant.at/DIY/?p=204
         7E 00 12 A2 00 40 CA AB 13
***************/

#include <SoftwareSerial.h>
#include <Stepper.h>
#include <Servo.h>

//Global Class variables for XBee Communications
boolean dataReceivedFlag =false;
unsigned char parsedData[100];
unsigned char receiveBuffer[100];
unsigned char frameDataMSB[4];
unsigned char frameDataLSB[4];
unsigned char frameDataAddr[2];
unsigned int currentReceiveIndex =0;
unsigned int receivedDataLength=0;

unsigned int xbeeRxPin =2;          //Using the Digital Pin 2 of the Arduino Uno as the Rx for Xbee through Software serial
unsigned int xbeeTxPin =3;          //Digital Pin 3 will be the Xbee TX Pin
unsigned int servoPin =9;

//If the XBee was connected using the XBee Shield, it would be automatically connected to the hardware serial RX and TX pins
//of the Arduino Uno board, which are pins 0 and 1 respectively.
SoftwareSerial xbeeArd(xbeeRxPin, xbeeTxPin);    //Using software serial of the arduino since I am connecting the xbee without the shield to the Arduino. Need Software serial since the Arduino Rx and Tx is already used




/*************************************************************/
//Global variables for the Motor Control
/*
 Created on 18th April 2015
 by Ekta Bindlish
 
 This program drives a bipolar stepper motor.
 The 1st motor is attached to digital pins 8 - 11 of the Arduino.
 The 2nd motor is attached to digital pins 4 -  7 of the Arduino.
 The motor will step one step at a time. 
 Each step covers 7.5 degrees.
 One rotation takes 48 steps.
*/
int stepA;
int stepB;
int angleA;
int angleB;

char directionA;
char directionB;

char buffer1[100];

// the stepsPerRevolution and degreesPerStep were taken from the motor specs
const int stepsPerRevolution = 48;  // motor takes 48 steps per minute
const double degreesPerStepA = 10;
const double degreesPerStepB = 7.5;  // motor takes 7.5 degrees per step

// initialize stepper motor A on pins 8 through 11:
//for clockwise rotation
//brown:  pin 8
//black:  pin 9
//orange: pin 10 
//yellow: pin 11 
//no other connections needed
//Stepper myStepperA(stepsPerRevolution, 8, 9, 10, 11);

// initialize stepper motor A on pins 8 through 11:
//for anti-clockwise rotation
//brown:  pin 8
//black:  pin 9
//orange: pin 10 
//yellow: pin 11 
//no other connections needed
//Stepper myStepperA1(stepsPerRevolution, 10, 11, 8, 9);

//Using servo instead
Servo myServoA;

// initialize stepper motor B on pins 4 through 7:
//for clockwise rotation
//brown:  pin 4
//black:  pin 5
//orange: pin 6 
//yellow: pin 7 
//no other connections needed
Stepper myStepperB(stepsPerRevolution, 4, 5, 6, 7);

// initialize stepper motor B on pins 4 through 7:
//for anti-clockwise rotation
//brown:  pin 4
//black:  pin 5
//orange: pin 6 
//yellow: pin 7 
//no other connections needed
Stepper myStepperB1(stepsPerRevolution, 6, 7, 4, 5);


/**********************************************************************************************************************/






void setup(){
  
  Serial.begin(9600);
  xbeeArd.begin(9600);
  myServoA.attach(servoPin);
  myServoA.write(100);
}




/************************                     *******************************
                  XBEE COMMUNICATION FUNCTIONS 
************************                     *******************************/                 


void sendPacket(){
  //Max Payload size is 92 bytes if using broadcast
  
  unsigned char packet[22] = {0};  //actual size will change (initialize to all zeros)
  unsigned int packetLength = 18;  //actual size will change
  packet[0] = 0x7E;
  packet[1] = (packetLength >> 8) & 0xFF; //get MSB
  packet[2] = (packetLength >> 0) & 0xFF; //get LSB
  //frame specific data
  packet[3] = 0x10; //send data identifier (tx request)
  packet[4] = 0x00; //packet id
  
  //When using the 64-bit address of a specific Xbee to send to 
  /*
  packet[5] = 0x00;
  packet[6] = 0x13;
  packet[7] = 0xA2;
  packet[8] = 0x00;
  packet[9] = 0x40;
  packet[10] = 0x90;
  packet[11] = 0x2D;
  packet[12] = 0xE8; 
  */
  
  //Broadcasting to any nearby Xbee instead
  //broadcast
  packet[5] = 0x00;
  packet[6] = 0x00;
  packet[7] = 0x00;
  packet[8] = 0x00;
  packet[9] = 0x00;
  packet[10] = 0x00;
  packet[11] = 0xFF;
  packet[12] = 0xFF;
  
  //
   //16 bit address, set to 0xFFFE if not known
  packet[13] = 0xFF;
  packet[14] = 0xFE;
  
 
  
  packet[15] = 0x00; //10 hops (maximum)
  packet[16] = 0x01; //disable ack
  
  //Now insert the actual 
  packet[17] = 'T';
  packet[18] = 'E';
  packet[19] = 'S';
  packet[20] = 't';
  
  //Now calculate the checksum value 
  unsigned int byteSum = 0;
  for(int i=3;i < 21;i++){
    byteSum += packet[i];
  }
  packet[21] = 0xFF - (byteSum & 0xFF);
   
  //Now writing the packet data onto the Software Serial
  for(int i=0; i<22; i++){
    xbeeArd.write(packet[i]); 
  }
   
}


//////////
/////////
int parsePacket(){
  if(receiveBuffer[0] != 0x7E){
    currentReceiveIndex = 0;
    return 0;
  }
  if(currentReceiveIndex < 3){
    return 0;
  }
  unsigned int length = ((receiveBuffer[1] << 8) & 0xFF00) | receiveBuffer[2];
  
  if(currentReceiveIndex < 4 + length){
    return 0;
  }
  
  unsigned char * frameData = receiveBuffer + 3;
  int checksum = frameData[length+1];
  
  switch(frameData[0]){
      case 0x90: //receive packet
        Serial.println("received an rx packet");
        currentReceiveIndex = 0;
        break;
      
      default:
        Serial.println("Unknown packet received");
        currentReceiveIndex = 0;
        break;  
  }
 //Getting the MSB & LSB of the frame data (framedata(1-8))
  frameDataMSB[0] =frameData[1];
  frameDataMSB[1] =frameData[2];
  frameDataMSB[2] =frameData[3];
  frameDataMSB[3] =frameData[4];
  
  frameDataLSB[0] =frameData[5];
  frameDataLSB[1] =frameData[6];
  frameDataLSB[2] =frameData[7];
  frameDataLSB[3] =frameData[8];
  
  Serial.print("FrameData MSB: ");
  for(int i=0; i<4;i++){
    Serial.print(frameDataMSB[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");
  
  Serial.print("FrameData LSB: ");
  for(int i=0; i<4;i++){
    Serial.print(frameDataLSB[i], HEX);
    Serial.print( " ");
  }
  Serial.println(" ");
  
  
  //16-bit address of sender
  frameDataAddr[0] = frameData[9];
  frameDataAddr[1] = frameData[10];
  
  
  //Options =frameData[11]
  
  //RF Data, the actual DATA THAT WAS SENT -frameData[12 --n] where n=byte length of entire RF packet frame
  unsigned char frameRFData[length -11]; //the RF data should be size 12th byte -nth byte
  
  Serial.println("The actual data (RF Data) of the packet was: ");
  for(int i =0; i<length -12; i++){
    frameRFData[i] = frameData[i+12];
    parsedData[i] = frameRFData[i];
    if(i ==length -11){
      Serial.write(frameRFData[i] + '\n');
    }
    else{
      Serial.write(frameRFData[i]);//, HEX);
    }
  }
  
  //Making sure bytes 4-n = FF for correct CheckSum
 // for(int i=4; i<length 
 
   int dataLength = length -12;
   Serial.println();
   Serial.print("Data length == ");
   Serial.println(dataLength);
   return dataLength;
} //Parsing Data


///////////////////////////////////////////////////////////////////////////////







///////////////////////////////////////////////////////////////////////////////////
// Method: ReadSerial()
///////////////////////////////////////////////////////////////////////////////////
void readSerial(){
    if(xbeeArd.available()){
      int b =  xbeeArd.read();
      if(b >= 0){
        receiveBuffer[currentReceiveIndex++] = (unsigned char) b;
        
        //Here I should probably add a parsing function 'parsePacket();' 
        //that we could use to parse the data and do an action based on it
        receivedDataLength = parsePacket();
        dataReceivedFlag = true;  
      }
      if(currentReceiveIndex >= 100){
        Serial.println("Error, receive buffer overflowed");
        currentReceiveIndex = 0; //reset our buffer
      }
      Serial.print(b,HEX);
      Serial.print(" ");
    }
    else{
      Serial.println("No serial data received");
      delay(1000);
    }
}

/////////////////////////////////////////////////////////////////////////////////

/**        END OF XBEE COMMUNICATION FUNCTIONS         **/

/////////////////////////////////////////////////////////////////////////////////






//&&&&&&&&&&&&&&&&&&&&&&   MOTOR CONTROL CODE    &&&&&&&&&&&&&&&&&&&&&&

// rotate motor A with n number of steps clockwise
/*void rotateStepperAclockwise(int numStepsA)
{
  int a;
  for (a = 0; a < numStepsA; a++)
  {
    myStepperA.step(1);
    delay(500);
  }
}
*/
// rotate motor A with n number of steps anti-clockwise
/*void rotateStepperAanticlock(int numStepsA1)
{
  int a;
  for (a = 0; a < numStepsA1; a++)
  {
    myStepperA1.step(1);
    delay(500);
  }
}
*/
//rotate servo A with n number of steps clockwise
void rotateServoAclockwise(int numStepsA)
{
  int a;
  for(a =0; a < numStepsA; a++)
  {
    Serial.print("Inside Rotate A CW. Steps = ");
    Serial.println(a);
    myServoA.write(0);
    delay(40);                //Delay of 40ms corresponds to a 10 degree rotate per step
    myServoA.write(100);
  }
}

//rotate servo A with n number of steps clockwise
void rotateServoAanticlock(int numStepsA)
{
  int a;
  for(a =0; a < numStepsA; a++)
  {
    myServoA.write(180);
    delay(40);                //Delay of 40ms corresponds to a 10 degree rotate per step
    myServoA.write(100);
  }
}


//rotate motor B with n number of steps clockwise
void rotateStepperBclockwise(int numStepsB)
{
  int b;
  for (b = 0; b < numStepsB; b++)
  {
    myStepperB.step(1);
    delay(500);
  }  
}

//rotate motor B with n number of steps anti-clockwise
void rotateStepperBanticlock(int numStepsB1)
{
  int b;
  for (b = 0; b < numStepsB1; b++)
  {
    myStepperB1.step(1);
    delay(500);
  }  
}

//&&&&&&&&&&&&&&&&&&&&&&   END OF MOTOR CONTROL CODE    &&&&&&&&&&&&&&&&&&&&&&






void loop(){
 
   //sendPacket();
   readSerial();
   if(dataReceivedFlag ==true && receivedDataLength ==8){
     //If data was sent, the readSerial method will call the parseData method to parse the data
     //from the XBee RF packet and store it in the global array variable 'parsedData[]'. The data recieved flag is set
  
    //Expecting the Data to be in the follow format:  01230123  
    //  parsedData[0-7] = 01230123
    //  The first four digits ('0123')->Motor A     &  last four digits('0123') -> Motor B
    //  Motor A and B rotates clockwise (0) by 123 degress/7.5 (d/step) = steps
    
    directionA = parsedData[0] - 48;
    directionA = directionA % 2;
    directionB = parsedData[4] - 48;
    directionB = directionB % 2;
    
    angleA = ((parsedData[1] - 48)*100) + ((parsedData[2] - 48)*10) + (parsedData[3] - 48);
    angleB = ((parsedData[5] - 48)*100) + ((parsedData[6] - 48)*10) + (parsedData[7] - 48);
    Serial.print("AngleA = ");
    Serial.println(angleA);
    Serial.print("AngleB = ");
    Serial.println(angleB);
    stepA = angleA/degreesPerStepA;
    stepB = angleB/degreesPerStepB;
     
    Serial.print("turn motor A by: ");
    Serial.print(stepA,DEC);
    Serial.print(" steps, direction: ");
    (directionA == 0)? Serial.print("clockwise"):Serial.print("anticlockwise");
    Serial.println();

    Serial.print("turn motor B by: ");
    Serial.print(stepB,DEC);
    Serial.print(" steps, direction: ");
    (directionB == 0)? Serial.print("clockwise"):Serial.print("anticlockwise");
    Serial.println();
    
    if (stepA>0)
    {
      if (!directionA)
      {
        rotateServoAclockwise(stepA);
      }
      else
      {
        rotateServoAanticlock(stepA);
      }
    }
    
    if (stepB > 0)    
    {
      if (!directionB)
      {
        rotateStepperBclockwise(stepB);
      }
      else
      {
        rotateStepperBanticlock(stepB);
      }
    }
     
     //Testing purposes
     for(int i=0; i <8; i++){
       Serial.print("Data[");
       Serial.print(i);
       Serial.print("] = ");
       Serial.write(parsedData[i]);
       Serial.println("\n\n");
       parsedData[i] = '0';
     }
     dataReceivedFlag =false;
   }
  
}
