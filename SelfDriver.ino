#include <NewPing.h>
#include <MPU6050.h>

#define TRIGGER_PIN  2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN   4 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN2  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 500 // Maximum distance ping (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
MPU6050 mpu;

/*
 *                       VARIABLE/CONSTANT DEFINITIONS
 */
 
double z,zz;

int e,a,aT,b,bT,f,fT,g,gT,prevError,E1,E2;
int setpoint = 0,angleSetpoint = 0,initVel = 70;

unsigned int MR,ML;
unsigned long timer = 0;

float timeStep = 0.01, gyaw = 0,pi = 3.143;

/*
 *                       INITIALISE COMPONENTS
 */
 
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

/*
 *                      MAIN LOOP
 */
 
double prop = 5.2772,inte = 2.8537,deriv = 0;

void loop()
{ 
   timer = millis();
   Vector norm = mpu.readNormalizeGyro();
   gyaw = gyaw + norm.ZAxis * timeStep;
   
   z = gyaw*3.2;
   
   delay((timeStep*1000) - (millis() - timer));
   
   for(int x = 0;x < 10; x++)
   {
      zz += error(0,z,angleSetpoint);
   } 
   zz = zz/10;
   
   Serial.print("Angle: ");
   Serial.print(zz);
   Serial.print("\t | \t ");
   
   if (zz > 0)
  {
    f = zz;
    fT += f;
    MR = pid(f,fT,prop,inte,deriv);
    RunMotors(9,MR/200,initVel);
    RunMotors(10,-MR/200,initVel);
  }
  else if (zz < 0)
  {
    g = -1*zz;
    gT += g;
    ML = pid(g,gT,prop,inte,deriv);
    RunMotors(10,ML/200,initVel);
    RunMotors(9,-ML/200,initVel);
  }
  else
  {
    RunMotors(10,0,initVel);
    RunMotors(9,0,initVel);
  }
 
   E1 = sonar.ping_cm();
   E2 = sonar2.ping_cm();
   
   for(int x = 0;x < 10; x++)
   {
      e += error(E2,E1,setpoint);
   } 
   e = e/10;

   Serial.print("Error: ");
   Serial.println(e);
   
   ML = 0;
   MR = 0;
  
   if (e > 0)
   {
     a = e;
     aT += a;
     MR = pid(a,aT,prop,inte,deriv);
     RunMotors(9,MR/200,initVel);
     RunMotors(10,-MR/200,initVel);
   }
   else if (e < 0)
   {
     b = -1*e;
     bT += b;
     ML = pid(b,bT,prop,inte,deriv);
     RunMotors(10,ML/200,initVel);
     RunMotors(9,-ML/200,initVel);
   }
   else
   {
     RunMotors(10,0,initVel);
     RunMotors(9,0,initVel);
   }
}

/*
 *                                FUNCTIONS
 */
 
int error(int a, int b, int c)
{
  int d;
  d = c - (a - b);
  return(d);
}

double pid(int InputError,int InputErrorTotal,double Kp,double Ki,double Kd)
{
  double p,i,d,cont;
  
  p = InputError*Kp;
  i = InputErrorTotal*Ki;
  d = Kd*(InputError-prevError);
  
  prevError = InputError;
  
  cont = p + i + d;
  return(cont);
}

void RunMotors(int Motor,int Gain,int Normal)
{
  int x = 0;
  if((Gain+Normal) > 255)
  {
    x = 255;
  }
  else
  {
    x = Gain+Normal;
  }
  analogWrite(Motor,x);
}

