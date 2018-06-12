void setup() {
  pinMode(18,OUTPUT); 
  pinMode(19,OUTPUT);
}
void loop() {
  digitalWrite(19,HIGH); 
  for(int x = 0; x < 400; x++) {
    digitalWrite(18,HIGH); 
    //delay(1);
    delayMicroseconds(3000); 
    digitalWrite(18,LOW); 
    //delay(1);
    delayMicroseconds(3000); 
  }
  delay(1000); 
  
  digitalWrite(19,LOW); 
  for(int x = 0; x < 400; x++) {
    digitalWrite(18,HIGH);
    //delay(1);
    delayMicroseconds(300);
    digitalWrite(18,LOW);
    //delay(1);
    delayMicroseconds(300);
  }
  delay(1000);
}
