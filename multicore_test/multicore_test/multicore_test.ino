// Possibly introduce second ADCBuff in order to update ADC values to next sample while the other core is processing the previous sample.
#include "pico/multicore.h"
#include <SPI.h>

char sync0 = 0; 
char sync1 = 0; 

uint8_t dacOut[] = {0x20,0x00,0x00}; 
const uint8_t* dacP = dacOut; 

uint8_t adcOut[] = {0x08,0x12,0xFF}; 
const uint8_t* adcP = adcOut;

uint8_t adcIn[] = {0x00,0x00}; 
uint8_t* adcVal = adcIn;

int16_t ADCBuff[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // buffer storing audio ADC values then potentiometer ADC values
int16_t buff0[8] = {0,0,0,0,0,0,0,0}; // buffer used by core1 to store processed values to be sent out to DAC
int16_t buff1[8] = {0,0,0,0,0,0,0,0}; // buffer used by core0 to store processed values as intermediary for buff0

int potCounter = 0;

SPISettings parSPI(31250000, MSBFIRST, SPI_MODE0);

void core1_SPI(){
  // setup SPI devices
  adcSetup(1) ; //Audio ADC
  dacSetup() ; 
  
  while(1){
    // read audio adc values for each channel
    // store in ADCBuff
    for (int i = 0; i < 8; i++){
      adcRead(1, adcP, (uint8_t*)(&ADCBuff[i]) ) ; 
    }
    
    //potCounter++;

    // indicate to core0 that ADC values are ready
    sync0 = 1 ; //Core1 Done 

    // output buff0 data to DAC while core0 is processing
    for (uint8_t i = 0; i < 8; i++){
      dacOut[0] = i | 0x20 ; 
      
      memcpy(&dacOut[1], &buff0[i], 2) ; 
      
      dacWrite(dacP) ; 
    }   

    // wait for buff1 to be updated
    while( sync1 != 1 ) { } //Do nothing
    sync1 = 0; 

    memcpy(buff0, buff1, 8);
  }
}

void setup() {
  //Setup SPI devices
  pinMode(2, OUTPUT);
  pinMode(1, OUTPUT);

  SPI.begin(true);
  SPI.beginTransaction(parSPI);
  
  // Start core 1 (SPI Interaction)
  multicore_launch_core1(core1_SPI);
  

}

void loop() {
  // wait for ADC data to be ready
  while(sync0 != 1) {} //Do nothing

  // process data
  // store processed data in buff1
  memcpy(buff1, ADCBuff, 16) ; 

  // indicate to core1 that buff1 is updated   
  sync1 = 1; 
  sync0 = 0; 
  
}

void adcSetup(int gpio) {
  uint8_t adcOut[] = {0x08,0x12,0xFF}; 
  const uint8_t* adcP = adcOut;
  
  uint8_t adcIn[] = {0x00,0x00}; 
  uint8_t* adcVal = adcIn;
 
  //ADC Autosequencing (16 bit communications rather than 24 bit)
  gpio_put(gpio,0); //GPIO 0 LOW
  spi_write_blocking(spi0, adcP, 3); //SEND SPI COMMAND
  gpio_put(gpio, 1); //GPIO 0 HIGH

  adcOut[0] = 0x08;
  adcOut[1] = 0x10;
  adcOut[2] = 0x11;

  gpio_put(gpio,0); //GPIO 0 LOW
  spi_write_blocking(spi0, adcP, 3); //SEND SPI COMMAND
  gpio_put(gpio, 1); //GPIO 0 HIGH

  adcOut[0] = 0x00;
  adcOut[1] = 0x00;
  
  gpio_put(gpio,0); //GPIO 0 LOW
  spi_write_blocking(spi0, adcP, 2); //SEND SPI COMMAND
  gpio_put(gpio, 1); //GPIO 0 HIGH
}

//Puts DAC in external reference mode
void dacSetup() {
  uint8_t dacOut[] = {0x7F,0x00,0x00}; 
  const uint8_t* dacP = dacOut;
 
  //Set DAC to external ref mode
  gpio_put(2,0); //GPIO 2 LOW
  spi_write_blocking(spi0, dacP, 3); //SEND SPI COMMAND
  gpio_put(2, 1); //GPIO 2 HIGH

}

void adcRead(int gpio, const uint8_t* src, uint8_t* dst) {
  gpio_put(gpio,0); //GPIO LOW
  spi_write_read_blocking(spi0, src, dst, 2 );
  gpio_put(gpio, 1); //GPIO HIGH
}

void dacWrite(const uint8_t* src) {
  gpio_put(2,0); //GPIO 2 LOW
  spi_write_blocking(spi0, src, 3); //SEND SPI COMMAND
  gpio_put(2, 1); //GPIO 2 HIGH
}
