#include <SoftwareSerial.h>
#include <Servo.h>
#include <NewPing.h>
#include <MPU6050.h>

#define TRIGGER_PIN  2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN   4 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN2  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     8  // Arduino pin tied to echo pin on the ultrasonic sensor.

#define MAX_DISTANCE 300 // Maximum distance ping (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
MPU6050 mpu;

double p,i,d,cont;
int e,eT,prevError;
double MR,ML;

float timeStep = 0.01;
float gyaw = 0;


void setup() 
{
    Serial.begin(9600); // HC-06 default serial speed is 9600
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
    {
      Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
      delay(500);
    }
  
    mpu.calibrateGyro();
    mpu.setThreshold(3);
    
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
}

void loop()
{ 
   int setpoint = 0;
  
   Vector norm = mpu.readNormalizeGyro();
   
   gyaw = gyaw + norm.ZAxis * timeStep;
   gyaw = gyaw*2.475;
   
   Serial.print("gyaw:");
   Serial.println(gyaw);
  
  e = error(sonar.ping_cm(),sonar2.ping_cm(),setpoint);
  if( e > -1)
  {
    eT += e;
    Serial.print("Error:");
    Serial.println(e);
    MR = pid(e,eT,1,1,1);
  }
  else if(e <= -1)
  {
    e = -1 * e;
    eT += e;
    Serial.print("Error2:");
    Serial.println(e);
    ML = pid(e,eT,1,1,1);
  }
}

int error(int a, int b, int c)
{
  int d;
  d = c - (a - b);
  return(d);
}

double pid(int InputError,int InputErrorTotal,double Kp,double Ki,double Kd)
{
  p = InputError*Kp;
  i = InputErrorTotal*Ki;
  d = Kd*(InputError-prevError);
  
  prevError = InputError;
  cont = p + i + d;
  return(cont);
}

