int Pin = A0;
int Pin2 = A1;
void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  int Distance,Distance2;
  
  Distance = IRSensorDistance(Pin);
  Distance2 = IRSensorDistance(Pin2);
}

int IRSensorDistance(int Sensor)
{
  int distance = 0;
  float sensorValue = analogRead(Sensor)*0.0048828125;
  float IR = 0,NewIR = 0;
  
  for(int i = 0;i < 10;i++)
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
