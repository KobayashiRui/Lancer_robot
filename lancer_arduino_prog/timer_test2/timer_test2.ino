#include <FlexiTimer2.h>
float  i=0;

void test(){
  Serial.println(i);
  }
  
void flash()
{
  i= i+ 1;
}

void setup() {
  Serial.begin(9600);
  FlexiTimer2::set(50, 1.0/1000, flash); // call every 500 1ms "ticks"
  // FlexiTimer2::set(500, flash); // MsTimer2 style is also supported
  FlexiTimer2::start();
}

void loop() {
  test();
}
