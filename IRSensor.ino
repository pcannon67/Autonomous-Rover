void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  int Distance,Distance2;
  
  Distance = IRSensorDistance(A0);
  Distance2 = IRSensorDistance(A1);
}

int IRSensorDistance(int Sensor)
{
  int distance;
  float sensorValue = analogRead(Sensor)*0.0048828125;
  float IR,NewIR;
  
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
