/**************
* Authors:      Eric Refour (XBee Communication)
* Date:         April 28th 2015
* Description:  Code to send send the degree command data from Unity3D to the
*               Arduino Uno that controls the motors. The data is sent using
*               the XBee Series 2 radios
* Useful Links: 
***************/


#include <SoftwareSerial.h> 


//Global Variable
/*****************************************************************/
int pos = 0;
String comdata = "";
int cmd[2] = {0}, mark = 0;

unsigned int xbeeRxPin =2;          //Using the Digital Pin 2 of the Arduino Uno as the Rx for Xbee through Software serial
unsigned int xbeeTxPin =3;          //Digital Pin 3 will be the Xbee TX Pin

//If the XBee was connected using the XBee Shield, it would be automatically connected to the hardware serial RX and TX pins
//of the Arduino Uno board, which are pins 0 and 1 respectively.
SoftwareSerial xbeeArd(xbeeRxPin, xbeeTxPin);    //Using software serial of the arduino since I am connecting the xbee without the shield to the Arduino. Need Software serial since the Arduino Rx and Tx is already used

char dataToSend[8];
int dataIsReady=0;
/****************************************************************/


void setup() {  
   Serial.begin (9600);  
   xbeeArd.begin(9600); 
}

/**************************************************************
* Method is used to send the degree command data from Unity3D
* to the Arduino Uno that controls the motors. The data is sent
* using the XBee Series 2 radios
*************************************************************/
void sendPacket(char sendData[8]){
  //Max Payload size is 92 bytes if using broadcast
  
  unsigned char packet[26] = {0};  //actual size will change (initialize to all zeros)
  unsigned int packetLength = 22;  //actual size will change
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
  packet[17] = sendData[0];//'0';
  packet[18] = sendData[1];//'1';
  packet[19] = sendData[2];//'2';
  packet[20] = sendData[3];//'3';
  
  packet[21] = sendData[4];//'0';
  packet[22] = sendData[5];//'1';
  packet[23] = sendData[6];//'2';
  packet[24] = sendData[7];//'3';
  
  //Now calculate the checksum value 
  unsigned int byteSum = 0;
  for(int i=3;i < 25;i++){
    byteSum += packet[i];
  }
  packet[25] = 0xFF - (byteSum & 0xFF);
   
  //Now writing the packet data onto the Software Serial
  Serial.print("Sending data: ");
  for(int i=0; i<26; i++){
    Serial.print(packet[i], HEX);
    Serial.print(" ");
    xbeeArd.write(packet[i]); 
  }
  Serial.println(" <<");
}
/******************************** END of the SendPacket() Method ***********************************/







void loop() {
  
  char angle1[4]={0};
  char angle2[4]={0};
  int angle1DigitCnt =0;
  int angle2DigitCnt =0;
  
  int switchFlag=0; //Used to switch between storing data for Angle 1 to Angle 2 once the "," is parsed from the Unity3D data sent
  int i=0;
  int j=0;
  
  while(Serial.available() >0){
    if(switchFlag ==0)  {  //Storing data for the 1st angle
      char temp1 = angle1[i];
      angle1[i] = char(Serial.read());
      delay(3);
      Serial.print("Incoming char data: ");
      Serial.println(angle1[i]);
      if(angle1[i] ==','){
          Serial.println("COMMAS");
          switchFlag =1;
          angle1[i] = temp1;
          i--;
      }
      i++;
    }
    else{
      char temp2 = angle2[j];
      angle2[j] = char(Serial.read());
      delay(3);
      Serial.print("Incoming char data: ");
      Serial.println(angle2[j]);
      if(angle2[j] =='.'){
          switchFlag =0;
          angle2[j] = temp2;
          j--;
      }
      j++;
    }
     dataIsReady =1;
  }
 
 if(dataIsReady ==1){
    angle1DigitCnt =i;
    angle2DigitCnt =j;
    
    Serial.print("Angle 1 Digit Count = ");
    Serial.println(i);
    Serial.print("Angle 2 Digit Count = ");
    Serial.println(j);
    
    i=0;
    j=0;
    //Storing Angle 1 into the sendData Packet 
   switch(angle1DigitCnt){
     
     case 0:
       Serial.println("Error in Angle 1 data");
       break;
       
     case 1:
       if(angle1[0] == '-'){
         Serial.println("Error in Angle 1 data - One Digit = '-' and no number");
       }else{
         dataToSend[0] = '0'; //Negative degrees correspond to counterclock wise rotations so the command format for the motor code should be Motor A: 1001 or 1007 ->  -001 degress or -007 degrees
         dataToSend[1] = '0';
         dataToSend[2] = '0';
         dataToSend[3] = angle1[0];
       }
       break;
       
     case 2:
       //Angle 1 has two digit: Either a negative 1 digit such as 'Ex: -1 or -6'     or   positive two digits 'Ex: 12 or 68'
       if(angle1[0] == '-'){
         dataToSend[0] = '1'; //Negative degrees correspond to counterclock wise rotations so the command format for the motor code should be Motor A: 1001 or 1007 ->  -001 degress or -007 degrees
         dataToSend[1] = '0';
         dataToSend[2] = '0';
         dataToSend[3] = angle1[1];
       }else{
         dataToSend[0] = '0';  //Clockwise rotation for angle 1
         dataToSend[1] = '0';
         dataToSend[2] = angle1[0];
         dataToSend[3] = angle1[1];
       }
       break;
     
     case 3:
        //Angle 1 has 3 digits: Either a negative 2 digit such as 'Ex: -12 or -63'     or   positive 3 digits 'Ex: 123 or 268 degrees'
       if(angle1[0] == '-'){
         dataToSend[0] = '1'; //Negative degrees correspond to counterclock wise rotations so the command format for the motor code should be Motor A: 1012 or 1087 ->  -012 degress or -087 degrees
         dataToSend[1] = '0';
         dataToSend[2] = angle1[1];
         dataToSend[3] = angle1[2];
       }else{
         dataToSend[0] = '0';  //Clockwise rotation for angle 1
         dataToSend[1] = angle1[0];
         dataToSend[2] = angle1[1];
         dataToSend[3] = angle1[2];
       }
       break;
     
     case 4:
       //Angle 1 has 4 digits: MUST be a negative 3 digit such as 'Ex: -120 or -163'     or   its an error because you can have more than 3 digits for a postive angle 'Ex: 123 or 268 degrees'
       if(angle1[0] == '-'){
         dataToSend[0] = '1'; //Negative degrees correspond to counterclock wise rotations so the command format for the motor code should be Motor A: 1012 or 1087 ->  -012 degress or -087 degrees
         dataToSend[1] = angle1[1];
         dataToSend[2] = angle1[2];
         dataToSend[3] = angle1[3];
       }else{
         Serial.println("Error in Angle 1 -- Positive 4 digits, which isnt possible");
       }
       break;
   }
    
    
   //Storing Angle 2 into the sendData Packet 
   switch(angle2DigitCnt){
     
     case 0:
       Serial.println("Error in Angle 2 data");
       break;
       
     case 1:
       if(angle2[0] == '-'){
         Serial.println("Error in Angle 2 data - One Digit = '-' and no number");
       }else{
         dataToSend[4] = '0'; //Negative degrees correspond to counterclock wise rotations so the command format for the motor code should be Motor A: 1001 or 1007 ->  -001 degress or -007 degrees
         dataToSend[5] = '0';
         dataToSend[6] = '0';
         dataToSend[7] = angle2[0];
       }
       break;
       
     case 2:
       //Angle 2 has two digit: Either a negative 1 digit such as 'Ex: -1 or -6'     or   positive two digits 'Ex: 12 or 68'
       if(angle2[0] == '-'){
         dataToSend[4] = '1'; //Negative degrees correspond to counterclock wise rotations so the command format for the motor code should be Motor A: 1001 or 1007 ->  -001 degress or -007 degrees
         dataToSend[5] = '0';
         dataToSend[6] = '0';
         dataToSend[7] = angle2[1];
       }else{
         dataToSend[4] = '0';  //Clockwise rotation for angle 2
         dataToSend[5] = '0';
         dataToSend[6] = angle2[0];
         dataToSend[7] = angle2[1];
       }
       break;
     
     case 3:
        //Angle 2 has 3 digits: Either a negative 2 digit such as 'Ex: -12 or -63'     or   positive 3 digits 'Ex: 123 or 268 degrees'
       if(angle2[0] == '-'){
         dataToSend[4] = '1'; //Negative degrees correspond to counterclock wise rotations so the command format for the motor code should be Motor A: 1012 or 1087 ->  -012 degress or -087 degrees
         dataToSend[5] = '0';
         dataToSend[6] = angle2[1];
         dataToSend[7] = angle2[2];
       }else{
         dataToSend[4] = '0';  //Clockwise rotation for angle 2
         dataToSend[5] = angle2[0];
         dataToSend[6] = angle2[1];
         dataToSend[7] = angle2[2];
       }
       break;
     
     case 4:
       //Angle 2 has 4 digits: MUST be a negative 3 digit such as 'Ex: -120 or -163'     or   its an error because you can have more than 3 digits for a postive angle 'Ex: 123 or 268 degrees'
       if(angle2[0] == '-'){
         dataToSend[4] = '1'; //Negative degrees correspond to counterclock wise rotations so the command format for the motor code should be Motor A: 1012 or 1087 ->  -012 degress or -087 degrees
         dataToSend[5] = angle2[1];
         dataToSend[6] = angle2[2];
         dataToSend[7] = angle2[3];
       }else{
         Serial.println("Error in Angle 2 -- Positive 4 digits, which isnt possible");
       }
       break;
   }
   
   Serial.println("About to send the Unity Data to the Motor"); 
   for(int i =0; i <8; i++){
     Serial.write(dataToSend[i]);
     Serial.println();
   }
   sendPacket(dataToSend);
   dataIsReady=0;
} //If data is ready
  
  
  
  
  
  
  
  
  
  
  /*int j = 0;
  while (Serial.available() > 0) {
    comdata += char(Serial.read());
    delay(2);
    mark = 1;
  }

  if(mark == 1) {
    for(int i = 0; i < comdata.length() ; i++) {
      if(comdata[i] == ',') {
        j++;
      }
      else {
        cmd[j] = (comdata[i] - '0');
      }
    }
    comdata = String("");    
  }
  
  if(cmd[0]!=0) {
    for(pos = 0; pos <= cmd[0]*10; pos += 1) { // goes from 0 degrees to 180 degrees, in steps  of 1 degree 
     
      //myservo.write(pos);
      
      
      //sendPacket( Passs in the parameters here)
      delay(15);  // waits 15ms for the servo to reach the position 
    }
  }
  */
}

