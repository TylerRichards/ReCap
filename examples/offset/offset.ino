#include <ReCap.h>

void setup() {
  // put your setup code here, to run once:
  ReCap_init();  
}

void loop() {
  // put your main code here, to run repeatedly:
  start_sample();

  for(int i = 0; i<8; i++){
    buff1[i] = ADCBuff[i+8];
  }

  end_sample();
}
