//--------------------------------------------------------------------------------------------------------------------
/*
 *                                            CLASS HEADER FILES
 */
//--------------------------------------------------------------------------------------------------------------------

#include <NewPing.h>
#include <MPU6050.h>

//--------------------------------------------------------------------------------------------------------------------
/*
 *                                         FIXED CONSTANTS DEFINITIONS
 */
//--------------------------------------------------------------------------------------------------------------------

#define TRIGGER_PIN  2  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN   4 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN2  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 500 // Maximum distance ping (in centimeters). Maximum sensor distance is rated at 400-500cm.

//--------------------------------------------------------------------------------------------------------------------
/*
 *                                         CLASS OBJECT INSTANTIATIONS
 */
//--------------------------------------------------------------------------------------------------------------------

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
MPU6050 mpu;

//---------------------------------------------------------------------------------------------------------------
/*
 *                                    VARIABLE/CONSTANT DEFINITIONS
 */
//---------------------------------------------------------------------------------------------------------------
 
double z = 0,zz = 0;

int e = 0,a  = 0,aT  = 0,b  = 0,bT  = 0;
int f  = 0,fT  = 0,g  = 0,gT  = 0,prevError  = 0,E1 = 0,E2  = 0;
int PositionSetpoint = 0,angleSetpoint = 0,initVel = 80;

unsigned int MR,ML;
unsigned long timer = 0;

float timeStep = 0.01, gyaw = 0,pi = 3.143;

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

//------------------------------------------------------------------------------------------------------------------
/*
 *                                           MAIN CONTROL LOOP
 */
//------------------------------------------------------------------------------------------------------------------
 
                      double prop = 5.3,inte = 2.8,deriv = 3;   // PID GAINS

void loop()
{ 
  
   ML = 0;
   MR = 0;
   
   timer = millis();
   Vector norm = mpu.readNormalizeGyro();            // read in gyro data as a 3x1 vector
   gyaw = gyaw + norm.ZAxis * timeStep;              // calculate angle from angular velociy
   z = gyaw*3.2;                                     // scale angle data
   
   delay((timeStep*1000) - (millis() - timer));
   
   for(int x = 0;x < 10; x++)
   {
      zz += error(0,z,angleSetpoint);              // calculate error and run it through a 10 point averaging filter
   } 
   zz = zz/10;
   
   //Serial.print("Angle: ");
   //Serial.println(zz);
   //Serial.print("\t | \t ");
   
  if (zz > 0)                    //if angle > 0....
  {
    f = zz;
    fT += f;                            // calculate sum error (for integral)
    MR = pid(f,fT,prop,inte,deriv);     // calculate PID gain
    RunMotors(9,MR/200,initVel);          
    RunMotors(10,-MR/200,initVel);      // run motors
  }
  else if (zz < 0)                //if angle < 0....
  {
    g = -1*zz;
    gT += g;                            // calculate sum error (for integral)
    ML = pid(g,gT,prop,inte,deriv);     // calculate PID gain
    RunMotors(10,ML/200,initVel);
    RunMotors(9,-ML/200,initVel);       // run motors
  }
  else
  {
    RunMotors(10,0,initVel);
    RunMotors(9,0,initVel);
  }
 
   E1 = sonar.ping_cm();
   E2 = sonar2.ping_cm();                        // read in distance from untrasonic sensors
   
   for(int x = 0;x < 10; x++)
   {
      e += error(E2,E1,PositionSetpoint);       // calculate error and run it through a 10 point averaging filter       
   } 
   e = e/10;

   //Serial.print("Error: ");
   //Serial.println(e);
  
   if (e > 0)                 //if angle > 0....
   {
     a = e;
     aT += a;                          // calculate sum error (for integral)
     MR = pid(a,aT,prop,inte,deriv);   // calculate PID gain
     RunMotors(9,MR/100,initVel);
     RunMotors(10,-MR/100,initVel);     // run motors
   }
   else if (e < 0)          //if angle < 0....
   {
     b = -1*e;
     bT += b;                         // calculate sum error (for integral)
     ML = pid(b,bT,prop,inte,deriv);  // calculate PID gain
     RunMotors(10,ML/100,initVel);     
     RunMotors(9,-ML/100,initVel);     // run motors
   }
   else
   {
     RunMotors(10,0,initVel);
     RunMotors(9,0,initVel);
   }
}

//-------------------------------------------------------------------------------------------------------------
/*
 *                                                 FUNCTIONS
 */
//-------------------------------------------------------------------------------------------------------------

/*
 *                                   CALCULATING THE ERROR FOR BOTH POSITION AND ANGLE                     
 */
  
int error(int a, int b, int c)
{
  int d;
  d = c - (a - b);
  return(d);
}

/*
 *                                      CALCULATING THE PID GAIN VALUES
 */
 
double pid(int InputError,int InputErrorTotal,double Kp,double Ki,double Kd)
{
  double p=0,i=0,d=0,cont=0;
  double N=10,NT;
  NT += N;      //Practical Derivitive Term components(Anti-High Frequency Noise Sensitvity)
  double ad = Kd/(Kd+NT),bd = Kp*N*ad;
  
  p = InputError*Kp;
  i = InputErrorTotal*Ki;
  d = ad*(d-bd)*(InputError-prevError);
  
  prevError = InputError;
  
  cont = p + i + d;
  return(cont);
}

/*
 *                                    CONTROLLING THE MOTORS
 */
 
void RunMotors(int Motor,int Gain,int Normal)
{
  int x = 0;
  if((Gain+Normal) > 255)
  {
    x = 255;                      // Actuator Limit Saturation 
  }
  else
  {
    x = Gain+Normal;              // add the PID gain to the initial velocity
  }
  analogWrite(Motor,x);
}

