/*
In this section, 1 Rc_Update function and 1 Radio must be chosen 
It must contain Telemetry_Start And Telemetry_Update fuctions
If you don't need telemetry you can uncomment NoTelemetry
*/


///////// Nrf24 ///////////

#include <RF24.h>
#include "nRF24L01.h"

RF24 radio(8,10);



void Telemetry_Start()
{ 
    radio.begin();
    radio.setPayloadSize(10);
    radio.setChannel(80); 
    radio.openWritingPipe(0xF0F0F0F0F0);
    radio.openReadingPipe(1,0xF0F0F0F0AA);
    radio.startListening();
}
  
void Telemetry_Update()
{
  
  int datatosend[4];
  datatosend[0]=1;
  datatosend[1]=23;
  datatosend[2]=45;
  datatosend[3]=1532;
  radio.stopListening();
  radio.write( datatosend, sizeof(datatosend) );
  radio.startListening();
  
}


/*
////////NoTelemetry////////

void Telemetry_Start()
{ 
}
  
void Telemetry_Update()
{
}
*/

/////The function to use Nrf24 radio as RC input////////
void Rc_Update()
{
  int incomingdata[5]={0,0,0,0,0};
  radio.read( incomingdata, sizeof(incomingdata) );
  if(incomingdata[0]==ID)
  {
    ch_thr=incomingdata[1];
    ch_yaw=incomingdata[2];
    ch_pitch=incomingdata[3];
    ch_roll=incomingdata[4];
    desired_thr=ch_thr;
    desired_yaw=ch_yaw;
    desired_pitch=ch_pitch;
    desired_roll=ch_roll;
    
  }
  else
  {
    ch_thr=0;
    ch_yaw=0;
    ch_pitch=0;
    ch_roll=0;
    desired_thr=ch_thr;
    desired_yaw=ch_yaw;
    desired_pitch=ch_pitch;
    desired_roll=ch_roll;
  }
    
  Serial.print(fl_output);  Serial.print("      "); Serial.println(fr_output);
  Serial.print(bl_output);  Serial.print("      "); Serial.println(br_output);
 
  Serial.print("RC=");
  Serial.print(ch_thr);
  Serial.print(",");
  Serial.print(ch_yaw);
  Serial.print(",");
  Serial.print(ch_pitch);
  Serial.print(",");
  Serial.print(ch_roll);
  Serial.println();
}

