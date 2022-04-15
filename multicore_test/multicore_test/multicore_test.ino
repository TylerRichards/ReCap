// Possibly introduce second ADCBuff in order to update ADC values to next sample while the other core is processing the previous sample.
#include "pico/multicore.h"

int32_t ADCBuff[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // buffer storing audio ADC values then potentiometer ADC values
int32_t buff0[8] = {0,0,0,0,0,0,0,0}; // buffer used by core1 to store processed values to be sent out to DAC
int32_t buff1[8] = {0,0,0,0,0,0,0,0}; // buffer used by core0 to store processed values as intermediary for buff0
int potCounter = 0;

void core1_SPI(){
  // setup SPI devices
  
  while(1){
    // read audio adc values for each channel
    // store in ADCBuff
    potCounter++;

    // indicate to core0 that ADC values are ready
    multicore_fifo_push_blocking(1);

    // output buff0 data to DAC while core0 is processing

    // wait for buff1 to be updated
    multicore_fifo_pop_blocking();
    memcpy(buff0, buff1, 8);
  }
}

void setup() {
  // Start core 1 (SPI Interaction)
  multicore_launch_core1(core1_SPI);

}

void loop() {
  // wait for ADC data to be ready
  multicore_fifo_pop_blocking();

  // process data
  // store processed data in buff1

  // indicate to core1 that buff1 is updated
  multicore_fifo_push_blocking(1);
  
}
