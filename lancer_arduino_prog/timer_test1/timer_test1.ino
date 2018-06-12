#include <FlexiTimer2.h>
const int led_pin = 5;     // default to pin 5
int i=0;
void flash()
{
static boolean output = HIGH;
  
  digitalWrite(led_pin, output);
  output = !output;
  if(i == 10){
    FlexiTimer2::set(500, 1.0/1000, flash);
    }else if(i == 0){
      FlexiTimer2::set(50, 1.0/1000, flash);
      }
}

void setup() {
  pinMode(led_pin, OUTPUT);

  FlexiTimer2::set(50, 1.0/1000, flash); // call every 500 1ms "ticks"
  // FlexiTimer2::set(500, flash); // MsTimer2 style is also supported
  FlexiTimer2::start();
}

void loop() {
  for(i=0; i <= 10; i++){
    delay(1000);
  }
}
