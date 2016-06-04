/*
NRF24 Based Rc transmitter and telemetry receiver
*/

#include <SPI.h>
#include <RF24.h>
#include "nRF24L01.h"

RF24 radio(7,8);
int ID=1;
int Max_angle=12;
void setup()
{ 
    radio.begin();
    radio.setPayloadSize(10);
    radio.setChannel(80); 
    radio.openWritingPipe(0xF0F0F0F0F0);
    //radio.openReadingPipe(1,0xF0F0F0F0AA);
    radio.stopListening();
    Serial.begin(115200);
}
  
void loop()
{
  int datatosend[5];
  datatosend[0]=ID;
  datatosend[1]=(analogRead(0)-500)*2;
  datatosend[2]=0;
  datatosend[3]=-1*(analogRead(1)-500)*Max_angle/500;
  datatosend[4]=(analogRead(2)-500)*Max_angle/500;
  radio.write( datatosend, sizeof(datatosend) );
  delay(50);
}


