#include<FlexiTimer2.h>
#include<TimerOne.h>
const int Dir_pin1 = 5;
const int Dir_pin2 = 8;
const int Step_pin1 = 6;
const int Step_pin2 = 9;

int output1 = 1;
int output2 = 1;

void flash_timer2(){
  output2 = !output2;
  digitalWrite(Step_pin2, output2);
  delayMicroseconds(1);
  output2 = !output2;
  digitalWrite(Step_pin2, output2);
}

void flash_timer1(){
  output1 = !output1;
  digitalWrite(Step_pin1, output1);
  delayMicroseconds(1);
  output1 = !output1;
  digitalWrite(Step_pin1, output1);
}

void setup() {
  pinMode(Dir_pin1,OUTPUT);
  pinMode(Dir_pin2,OUTPUT);
  pinMode(Step_pin1,OUTPUT);
  pinMode(Step_pin2,OUTPUT);
  digitalWrite(Dir_pin1,LOW);
  digitalWrite(Dir_pin2,HIGH);
  digitalWrite(Step_pin1,output1);
  digitalWrite(Step_pin2,output2);
  
  FlexiTimer2::set(50, 1.0/10000, flash_timer2);//ms
  Timer1.initialize(5000);//ms
  Timer1.attachInterrupt(flash_timer1);
  FlexiTimer2::start();
 
}

void loop() {
  // put your main code here, to run repeatedly:

}
