#include <MPU6050.h>

MPU6050 mpu;

double z,zz;
unsigned long timer = 0;

float timeStep = 0.01, gyaw = 0,pi = 3.143;

void setup() 
{
    Serial.begin(9600);
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
      Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
      delay(500);
    }
  
    mpu.calibrateGyro();
    mpu.setThreshold(3);
    
    pinMode(10, OUTPUT);
    pinMode(6, OUTPUT);
    delay(2000);
}

void loop()
{ 
   timer = millis();
   Vector norm = mpu.readNormalizeGyro();
   z  = norm.ZAxis;
   
   delay((timeStep*1000) - (millis() - timer));
   
   for(int x = 0;x < 10; x++)
   {
      zz += z;
   } 
   zz = zz/10;
 
   analogWrite(10,100);
   analogWrite(6,100);
   
   Serial.println(zz);
   
}
