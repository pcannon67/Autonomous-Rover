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

double p = 0,i = 0,d = 0,z,cont;
int e,eT,a,aT,b,bT,prevError,E1,E2,setpoint = 0;
unsigned int MR,ML;
unsigned long timer = 0;
float timeStep = 0.01,gpitch = 0, groll = 0, gyaw = 0,pi = 3.143;

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
    pinMode(11, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(9, OUTPUT);
    delay(2000);
}

void loop()
{ 
   timer = millis();
   
   Vector norm = mpu.readNormalizeGyro();
   Vector normAccel = mpu.readNormalizeAccel();
   
   gyaw = gyaw + norm.ZAxis * timeStep;
  
   z = (2*pi)*((gyaw*5.25)/360);
   
   E1 = sonar.ping_cm()*cos(z);
   E2 = sonar2.ping_cm()*cos(z);
   
   for(int x = 0;x < 3; x++)
   {
      e += error(E1,E2,setpoint);
   } 
   e = e/3;
  
  eT += e;
  ML = 0;
  MR = 0;
  
  Serial.print("Error:");
  Serial.println(e);
  
  if (e > 0)
  {
    a = e;
    aT += a;
  }
  else if (e < 0)
  {
    b = -1*e;
    bT += b;
  }
  
  MR = pid(a,aT,1,1,1);
  RunMotors(9,MR/395,75);
  ML = pid(b,bT,1,1,1);
  RunMotors(10,ML/395,75);
  
  delay((timeStep*5000) - (millis() - timer));
}

int error(int a, int b, int c)
{
  int d;
  d = c - (a - b);
  return(d);
}

int pid(int InputError,int InputErrorTotal,double Kp,double Ki,double Kd)
{
  p = InputError*Kp;
  i = InputErrorTotal*Ki;
  d = Kd*(InputError-prevError);
  
  prevError = InputError;
  cont = p + i + d;
  return(cont);
}

void RunMotors(int Motor,int Gain,int Normal)
{
  int x;
  if((Gain+Normal) > 255)
  {
    x = 255;
    analogWrite(Motor,x);
  }
  else
  {
    x = Gain+Normal;
    analogWrite(Motor,x);
  }
  Serial.print("Motor:"+Motor);
  Serial.println(x);
    
}

