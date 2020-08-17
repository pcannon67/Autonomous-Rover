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

#define TRIGGER_PIN  2   // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN   3     // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN2  7  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN2     8  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 500 // Maximum distance ping (in centimeters). Maximum sensor distance is rated at 400-500cm.

//--------------------------------------------------------------------------------------------------------------------
/*
 *                                         CLASS OBJECT INSTANTIATIONS
 */
//--------------------------------------------------------------------------------------------------------------------

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);    // NewPing setup of pins and maximum distance.
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
MPU6050 mpu;

//---------------------------------------------------------------------------------------------------------------
/*
 *                                    VARIABLE/CONSTANT DEFINITIONS
 */
//---------------------------------------------------------------------------------------------------------------
int e = 0,a  = 0,aT  = 0,b  = 0,bT  = 0,N = 1,NT = 0;
int f  = 0,fT  = 0,g  = 0,gT  = 0,prevError  = 0,prevInput = 0,E1 = 0,E2  = 0;
int PositionSetpoint = 0,angleSetpoint = 0,initVel = 150;
int PosLimit = 1, AngLimit = 1;

double z = 0,zz = 0;
double p = 0,i = 0,d = 0,cont = 0,tau = 0.1;
double iMin = 0,iMax = 255 - initVel,i2Min,i2Max;

unsigned int ML,MR;
unsigned long timer = 0;
unsigned long timeBetFrames = 100;;

float timeStep = 0.01, gyaw = 0,pi = 3.143;

int Pin = A0;
int Pin2 = A1;
int Distance,Distance2;
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
      Serial.println("No MPU");
      delay(500);
    }
    
    mpu.calibrateGyro();
    mpu.setThreshold(3);
    
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    pinMode(6, OUTPUT);   //9 - 11 forwards / 6 - 10 backwards
    pinMode(9, OUTPUT);
    delay(2000);
    
    ML = 0;
    MR = 0;
}

//------------------------------------------------------------------------------------------------------------------
/*
 *                                           MAIN CONTROL LOOP
 */
//------------------------------------------------------------------------------------------------------------------
                     // PID GAINS
                     //double prop = 5.3,inte = 2.9,deriv = 1;        // Original System model with initial PI
                     //double prop = 615.5,inte = 30.8,deriv = 5;     // Modified Original System With initial PI
                     //double prop = 2,inte = 288,deriv = 0.007;      // New Model With PID
                     double prop = 30,inte = 100,deriv = 10;          // New Model With PID
                     // double prop = 1.22,inte = 43.6,deriv = 0.003; // New Model With Matlab PID Tuner
                                     
void loop()
{  
  static int Boundary = 10;
  
  Distance = IRSensorDistance(Pin);
  Distance2 = IRSensorDistance(Pin2);

  Serial.print(Distance);
  Serial.print("\t");
  Serial.print(Distance2);
  Serial.println("\t");
  
  if(Distance2 < Boundary && (Distance == 0 || Distance > Boundary))
  {
    RunMotors(9,0,initVel);
    RunMotors(6,0,0);
    RunMotors(10,0,0);
    RunMotors(11,0,0);
    //MainLoop();
  }
  else if (Distance < Boundary && (Distance2 == 0 || Distance2 > Boundary))
  {
    RunMotors(11,0,initVel);
    RunMotors(6,0,0);
    RunMotors(9,0,0);
    RunMotors(10,0,0);
  }
  else if(Distance < Boundary && Distance2 < Boundary)
  {
     RunMotors(10,0,initVel);
     RunMotors(11,0,initVel);
  }
  else if((Distance == 0 && Distance2 == 0) || (Distance > Boundary && Distance2 > Boundary))
  {
    RunMotors(6,0,initVel);
    RunMotors(11,0,initVel);
    RunMotors(10,0,0);
    RunMotors(9,0,0);
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

void MainLoop()
{
   static int x;
   timer = millis();
   
   Vector norm = mpu.readNormalizeGyro();          // read in gyro data as a 3x1 vector
   gyaw = gyaw + norm.ZAxis * timeStep;            // calculate angle from angular velociy
   z = gyaw*8;                                     // scale angle data

   Serial.println(z);
   delay((timeStep*1000) - (millis() - timer));
   
   RunMotors(9,0,255);
   RunMotors(11,0,255);
   
   E1 = sonar.ping_cm();
   E2 = sonar2.ping_cm();                        // read in distance from untrasonic sensors
   
   for(x = 0;x < 10; x++)
   {
      zz += error(0,z,angleSetpoint);            // calculate error and run it through a 10 point averaging filter
   } 
   zz = zz/10;
   
   for(x = 0;x < 10; x++)
   {
      e += error(E2,E1,PositionSetpoint);        // calculate error and run it through a 10 point averaging filter       
   } 
   e = e/10;
   
   if (zz > 0)                    // if angle > 0....
   {
      f = zz;
      fT += f;                    // calculate sum error (for integral)
      NT += N;
      MR = pid(f,fT,z,prop,inte,deriv,timeBetFrames);     // calculate PID gain
      RunMotors(9,MR/AngLimit,initVel);          
      RunMotors(10,-MR/AngLimit,initVel);      // run motors
   }
   else if (zz < 0)                // if angle < 0....
   {
      g = -1*zz;
      gT += g;                      // calculate sum error (for integral)
      NT += N;
      ML = pid(g,gT,-z,prop,inte,deriv,timeBetFrames);     // calculate PID gain
      RunMotors(10,ML/AngLimit,initVel);
      RunMotors(9,-ML/AngLimit,initVel);       // run motors
   }
   else
   {
      RunMotors(10,0,initVel);
      RunMotors(9,0,initVel);
   }

   Serial.print("Error: ");
   Serial.println(e);
  
   if (e > 0)                 // if angle > 0....
   {
      a = e;
      aT += a;                // calculate sum error (for integral)
      NT += N;
      MR = pid(a,aT,E2-E1,prop,inte,deriv,timeBetFrames);   // calculate PID gain
      RunMotors(9,MR/PosLimit,initVel);
      RunMotors(10,-MR/PosLimit,initVel);     // run motors
   }
   else if (e < 0)          // if angle < 0....
   {
      b = -1*e;
      bT += b;              // calculate sum error (for integral)
      NT += N;
      ML = pid(b,bT,E1-E2,prop,inte,deriv,timeBetFrames);  // calculate PID gain
      RunMotors(10,ML/PosLimit,initVel);     
      RunMotors(9,-ML/PosLimit,initVel);     // run motors
   }
   else
   {
      RunMotors(10,0,initVel);
      RunMotors(9,0,initVel);
   }
   
   delay(timeBetFrames - millis() - timer);
}

int error(int a, int b, int c)
{
    static int d;
    d = c - (a - b);
    return(d);
}

/*
 *   CALCULATING THE PID GAIN VALUES
 */
 
double pid(int InputError,int InputErrorTotal,int Input,double Kp,double Ki,double Kd,unsigned long timeBetFrames)
{ 
    p = InputError*Kp;
  
    i = i + 0.5*InputErrorTotal*Ki*0.0001*timeBetFrames;
    
    d = ((d*(2*tau - 0.001*timeBetFrames) - (2*Kd*(Input-prevInput))/(2*tau + 0.001*timeBetFrames));

    // Dynamic integrator clamping
    if(iMax > p)
    {
      i2Max = iMax - p;
    }
    else
    {
      i2Max = 0;
    }
    if(iMin < p)
    {
      i2Min = iMin - p;
    }
    else
    {
      i2Min = 0;
    }

    if(i > i2Max)
    {
      i = i2Max;
    }
    else if(i < i2Min)
    {
      i = i2Min;
    }
    
    prevError = InputError;
    prevInput = Input;
    
    cont = p + i + d;
    
    if(cont > 255)
    {
      cont = 255;
    }
    else if(cont < 0)
    {
      cont = 0;
    }
    return(cont);
}

/*
 *                                    CONTROLLING THE MOTORS
 */
 
void RunMotors(int Motor,int Gain,int Normal)
{
    static int x = 0;
    
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

int IRSensorDistance(int Sensor)
{
  static int distance = 0, i;
  static float sensorValue = analogRead(Sensor)*0.0048828125;
  static float IR = 0,NewIR = 0;
  
  for(i = 0;i < 10;i++)
  {
    IR += sensorValue;
  }
  NewIR = IR/10;
  
  distance = 13*pow(NewIR,-1);
  
  if(distance <= 30)
  {
    return(distance);
  }
  else
  {
    return(0);
  }
}
