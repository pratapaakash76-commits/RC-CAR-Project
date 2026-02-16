#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup(){
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
}

void loop(){
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax,&ay,&az);

  char cmd='S';

  if(ay > 6000) cmd='F';
  else if(ay < -6000) cmd='B';
  else if(ax > 6000) cmd='R';
  else if(ax < -6000) cmd='L';

  Serial.println(cmd);

  delay(100);
}
