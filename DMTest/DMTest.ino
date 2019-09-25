#include <Wire.h>
#include "DMREG.h"

void setup() 
{
  Serial.begin(9600);
  Serial3.begin(9600);
}

void loop() 
{
//  Serial.write(DM0_Speed1_Position_90, 10);
//  delay(1000);
//  Serial.write(DM0_Speed2_Position_0, 10);
//  delay(1000);
  Serial3.write(DM_Action0, 5);
  Serial.write(DM_Action0, 5);
  Serial.println('DM_Action0');
  delay(4000);
  
}
