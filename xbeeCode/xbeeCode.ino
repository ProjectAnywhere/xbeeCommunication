/**************
* Author:       Eric Refour
* Date:         April 20th 2015
* Description:  Code to send and receive data from XBee Series 2 radios. Using the XBee libary
* Useful Links: http://www.digi.com/support/kbase/kbaseresultdetl?id=3345
                http://www.farnell.com/datasheets/27606.pdf
                http://www.kobakant.at/DIY/?p=204

         7E 00 12 A2 00 40 CA AB 13
***************/

#include <SoftwareSerial.h>

//Global Class variables
unsigned char receiveBuffer[100];
unsigned char frameDataMSB[4];
unsigned char frameDataLSB[4];
unsigned char frameDataAddr[2];
unsigned int currentReceiveIndex =0;

unsigned int xbeeRxPin =2;          //Using the Digital Pin 2 of the Arduino Uno as the Rx for Xbee through Software serial
unsigned int xbeeTxPin =3;          //Digital Pin 3 will be the Xbee TX Pin

//If the XBee was connected using the XBee Shield, it would be automatically connected to the hardware serial RX and TX pins
//of the Arduino Uno board, which are pins 0 and 1 respectively.
SoftwareSerial xbeeArd(xbeeRxPin, xbeeTxPin);    //Using software serial of the arduino since I am connecting the xbee without the shield to the Arduino. Need Software serial since the Arduino Rx and Tx is already used


void setup(){
  
  Serial.begin(9600);
  xbeeArd.begin(9600);
}



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



/////////////////////////////////////////////////////////////////////////////////
void parsePacket(){
  if(receiveBuffer[0] != 0x7E){
    currentReceiveIndex = 0;
    return;
  }
  if(currentReceiveIndex < 3){
    return;
  }
  unsigned int length = ((receiveBuffer[1] << 8) & 0xFF00) | receiveBuffer[2];
  
  if(currentReceiveIndex < 4 + length){
    return;
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
  for(int i =0; i<length -11; i++){
    frameRFData[i] = frameData[i+12]; 
    Serial.println(frameRFData[i], HEX);
  }
  
  //Making sure bytes 4-n = FF for correct CheckSum
 // for(int i=4; i<length 
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
      }
      if(currentReceiveIndex >= 100){
        Serial.println("Error, receive buffer overflowed");
        currentReceiveIndex = 0; //reset our buffer
      }
      Serial.print(b,HEX);
      Serial.print(" ");
    }
}






void loop(){
 
   sendPacket();
   readSerial();
  
}
    
    
    
    
 

