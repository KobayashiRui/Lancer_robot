#include<FlexiTimer2.h>
#include<TimerOne.h>

volatile int i  = 0;
volatile int i2 = 0;
void flash_timer2(){
  i= i+1;
}

void flash_timer1(){
  i2 = i2 + 1;
  }

void setup() {
  Serial.begin(9600);
  FlexiTimer2::set(50, 1.0/1000, flash_timer2);
  Timer1.initialize(500000);
  Timer1.attachInterrupt(flash_timer1);
  FlexiTimer2::start();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(i);
  Serial.println(i2);
}
