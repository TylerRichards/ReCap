// Possibly introduce second ADCBuff in order to update ADC values to next sample while the other core is processing the previous sample.
#include "pico/multicore.h"
#include <SPI.h>
#include "lookUpTable.h"
#include "pico/sync.h"

uint8_t sync0 = 0; 
mutex_t sync0_lock ; 


uint8_t dacOut[] = {0x20,0x00,0x00}; 
const uint8_t* dacP = dacOut; 

uint8_t adcOut[] = {0x08,0x12,0xFF}; 
const uint8_t* adcP = adcOut;

uint8_t adcIn[] = {0x00,0x00}; 
uint8_t* adcVal = adcIn;

uint16_t ADCBuff[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // buffer storing audio ADC values then potentiometer ADC values
uint16_t buff0[8] = {0,0,0,0,0,0,0,0}; // buffer used by core1 to store processed values to be sent out to DAC
uint16_t buff1[8] = {0,0,0,0,0,0,0,0}; // buffer used by core0 to store processed values as intermediary for buff0

int potCounter = 0;

uint16_t ADCavg[8] = {0,0,0,0,0,0,0,0};
uint16_t POTavg[8] = {0,0,0,0,0,0,0,0};

uint16_t currentTime = 0;

SPISettings parSPI(31250000, MSBFIRST, SPI_MODE0);

void core1_SPI(){
  
  // setup SPI devices
  adcSetup(1) ; //Audio ADC
  adcSetup(0) ; //Pot ADC
  dacSetup() ; 
  
  while(1){
    // read audio adc values for each channel
    // store in ADCBuff
    for (int i = 0; i < 8; i++){
      adcRead(1, adcP, &ADCBuff[i] ) ;    
    }

    //Iterate through the bottom 8 array positions in ADCBuff
    adcRead(0,adcP,&ADCBuff[potCounter + 8] ) ;

    //Increment to next position 
    if (potCounter < 7) {
      potCounter++;     
    } else {
      potCounter = 0 ; 
    }


    // indicate to core0 that ADC values are ready
    mutex_enter_blocking(&sync0_lock) ; 
    sync0 = 1 ; //Core1 Done 
    mutex_exit(&sync0_lock) ;

    // output buff0 data to DAC while core0 is processing
    for (uint8_t i = 0; i < 8; i++){
      
      dacOut[0] = i | 0x20 ; 
      dacOut[1] = (uint8_t) (buff0[i] >> 8) ; 
      dacOut[2] = (uint8_t) buff0[i] ; 

      dacWrite(dacP) ;

    }   

    uint8_t mutex_temp = 2;
    
    // wait for buff1 to be updated
    while( mutex_temp != 0 ) { 
      mutex_enter_blocking(&sync0_lock) ; 
      mutex_temp = sync0 ;
      mutex_exit(&sync0_lock) ;       
    }

    memcpy(buff0, buff1, 16);
  }
}

void setup() {
  //Setup SPI devices
  pinMode(2, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(0, OUTPUT);

  //Start SPI peripheral
  SPI.begin(true);
  SPI.beginTransaction(parSPI);

  Serial.begin(9600) ; 

  mutex_init(&sync0_lock);

  // Start core 1 (SPI Interaction)
  multicore_launch_core1(core1_SPI);
  
}

void loop() {
  // wait for ADC data to be ready
    uint8_t mutex_temp = 2;
    while( mutex_temp != 1 ) { 
      mutex_enter_blocking(&sync0_lock) ; 
      mutex_temp = sync0 ;
      mutex_exit(&sync0_lock) ;       
    }

    
  uint64_t currentTime = 0x0000FFFF & time_us_32() ; 

  // process data
  for (int i = 0; i < 8; i++){

    uint32_t POTtemp = (uint32_t)(ADCBuff[i+8] >> 6) ; 
    uint32_t ADCtemp =  (uint32_t)(ADCBuff[i] >> 7); 
    
    //De-noise Pot and ADC readings
    POTavg[i] -= (POTavg[i] / 12 );
    POTavg[i] += (POTtemp / 12);

    ADCavg[i] -= (ADCavg[i] / 12);
    ADCavg[i] += (ADCtemp / 12);

    uint32_t outputTemp = (uint32_t)ADCavg[i] * (uint32_t)currentTime ; 
    outputTemp = 0x0000FFFF & (outputTemp >> 6) ; 

    
/*
    uint32_t outputTemp = ADCavg[i] * POTavg[i] ; 
    outputTemp = 0x0000FFFF & (outputTemp >> 4) ; 
    
    uint64_t roundTemp = (uint64_t)(outputTemp>>4) * currentTime ;  
/*
    if (!i) {
      Serial.println(roundTemp) ; 
      delay(10) ; 
    }

    roundTemp = 0x0000FFFF & (roundTemp >> 4) ;
*/
    //buff1[i] = sine[ uint16_t(roundTemp) ] ; //Go to value of sine in look-up table and place in buffer
    buff1[i] = sine[ uint16_t((ADCavg[i]>>2) * currentTime) ] ;
  }
    
  // indicate to core1 that buff1 is updated   
    mutex_enter_blocking(&sync0_lock) ; 
    sync0 = 0 ; //Core1 Done 
    mutex_exit(&sync0_lock) ;
  
}

inline void adcSetup(int gpio) {
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
inline void dacSetup() {
  uint8_t dacOut[] = {0x7F,0x00,0x00}; 
  const uint8_t* dacP = dacOut;
 
  //Set DAC to external ref mode
  gpio_put(2,0); //GPIO 2 LOW
  spi_write_blocking(spi0, dacP, 3); //SEND SPI COMMAND
  gpio_put(2, 1); //GPIO 2 HIGH

}

inline void adcRead(int gpio, const uint8_t* src, uint16_t* dst) {
  uint8_t adcIn[] = {0x00,0x00}; 
  
  gpio_put(gpio,0); //GPIO LOW
  spi_write_read_blocking(spi0, src, &adcIn[0], 2 );
  gpio_put(gpio, 1); //GPIO HIGH

  *dst = ( ((uint16_t)adcIn[0]) << 8) | (uint16_t)adcIn[1];    
}

inline void dacWrite(const uint8_t* src) {
  gpio_put(2,0); //GPIO 2 LOW
  spi_write_blocking(spi0, src, 3); //SEND SPI COMMAND
  gpio_put(2, 1); //GPIO 2 HIGH
}
