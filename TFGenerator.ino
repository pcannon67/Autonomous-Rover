//--------------------------------------------------------------------------------------------------------------------
/*
 *                                            CLASS HEADER FILES
 */
//--------------------------------------------------------------------------------------------------------------------

#include <MPU6050.h>
#include <SPI.h>
#include <SD.h>

//--------------------------------------------------------------------------------------------------------------------
/*
 *                                         CLASS OBJECT INSTANTIATIONS
 */
//--------------------------------------------------------------------------------------------------------------------

MPU6050 mpu;

//---------------------------------------------------------------------------------------------------------------
/*
 *                                    VARIABLE/CONSTANT DEFINITIONS
 */
//---------------------------------------------------------------------------------------------------------------
 
double z,zz;

unsigned long timer = 0;

float timeStep = 0.01, gyaw = 0,pi = 3.143;

const int chipSelect = 4;

//--------------------------------------------------------------------------------------------------------------
/*
 *                                   COMPONENT INITIALISATION LOOP 
 */
//--------------------------------------------------------------------------------------------------------------

void setup() 
{
    Serial.begin(9600);
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
      Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
      delay(500);
    }
    
    if (!SD.begin(chipSelect)) 
    {
      Serial.println("Card failed, or not present");
      // don't do anything more:
      return;
    }
    Serial.println("card initialized.");

    if(SD.exists("TFData.txt"))
    {
      SD.remove("TFData.txt");
    }
  
    mpu.calibrateGyro();
    mpu.setThreshold(3);
    
    pinMode(10, OUTPUT);
    pinMode(6, OUTPUT);
    delay(2000);
}

//------------------------------------------------------------------------------------------------------------------
/*
 *                                           MAIN CONTROL LOOP
 */
//------------------------------------------------------------------------------------------------------------------
 
void loop()
{ 
   timer = millis();
   Vector norm = mpu.readNormalizeGyro(); // read in gyro data as a 3x1 vector
   z  = norm.ZAxis;                       // read in angular velociy
   
   delay((timeStep*1000) - (millis() - timer));
   
   for(int x = 0;x < 10; x++)
   {
      zz += z;                      // run it through a 10 point averaging filter
   } 
   zz = zz/10;
 
   analogWrite(10,100);
   analogWrite(6,100);              // Activate the motors
   
   String dataString = String(zz);
   
   File dataFile = SD.open("TFData.txt", FILE_WRITE);

                                                         // if the file is available, write to it:
  if (dataFile) 
  {
    dataFile.println(dataString);
    dataFile.close();
    Serial.println(dataString);                                 // print to the serial port too:
  }
                                               // if the file isn't open, pop up an error:
  else 
  {
    Serial.println("error opening datalog.txt");
  }
}
