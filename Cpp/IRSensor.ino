int Pin = A0, Pin2 = A1, distance = 0;
float sensorValue = 0, IR = 0, NewIR = 0;

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
  distance = 0;
  sensorValue = analogRead(Sensor)*0.0048828125;
  IR = 0;
  NewIR = 0;
  
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
