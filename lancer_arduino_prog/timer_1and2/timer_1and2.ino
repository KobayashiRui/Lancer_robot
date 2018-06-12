#include<FlexiTimer2.h>
#include<TimerOne.h>
const int led_pin2 = 5;
const int led_pin1 = 3;

void flash_timer2(){
  static boolean output2 = HIGH;
  digitalWrite(led_pin2, output2);
  output2 = !output2;
}

void flash_timer1(){
  static boolean output1 = HIGH;
  digitalWrite(led_pin1, output1);
  output1 = !output1;
  }

void setup() {
  pinMode(led_pin1,OUTPUT);
  pinMode(led_pin2,OUTPUT);
  FlexiTimer2::set(50, 1.0/1000, flash_timer2);
  Timer1.initialize(500000);
  Timer1.attachInterrupt(flash_timer1);
  FlexiTimer2::start();
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
