void setup() 
{
  pinMode(6, OUTPUT); 
  pinMode(3, OUTPUT); 
  pinMode(4, OUTPUT); 
  pinMode(5, OUTPUT); 
  digitalWrite(6, HIGH); 
  digitalWrite(3, LOW); 
  digitalWrite(4, HIGH); 
  digitalWrite(5, LOW); 

  pinMode(9, OUTPUT); 
  pinMode(10, OUTPUT); 
  analogWrite(9, 180); 
  analogWrite(10, 180); 
  
}

void loop() 
{


}
