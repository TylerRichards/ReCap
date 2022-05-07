#include "ReCap.h"
#include "pico/multicore.h"
#include <SPI.h>
#include "pico/sync.h"

uint16_t ADCBuff[16];
uint16_t buff0[8];
uint16_t buff1[8];
uint8_t currentPot;
SPISettings _parSPI;
mutex_t _sync0_lock;
char _sync0;
uint8_t _dacOut[3];
const uint8_t* _dacP;
uint8_t _adcOut[3];
const uint8_t* _adcP;
uint8_t _adcIn[3];
uint8_t* _adcVal;

void ReCap_init(){

  for(int i = 0; i<8;i++){
    ADCBuff[i] = 0;
    ADCBuff[i+8] = 0;
    buff0[i]=0;
    buff1[i]=0;
  }

  _sync0 = 0;
  currentPot = 0;

  SPISettings _parSPI(31250000, MSBFIRST, SPI_MODE0);
  //Setup SPI devices
  pinMode(2, OUTPUT);
  pinMode(1, OUTPUT);

  // Initiate core syncing lock
  mutex_init(&_sync0_lock);

  //Start SPI peripheral
  SPI.begin(true);
  SPI.beginTransaction(_parSPI);
  
  // Start core 1 (SPI Interaction)
  multicore_launch_core1(core1_SPI);
}

inline void _adcSetup(int gpio){
     _adcP = _adcOut;
    _adcVal = _adcIn;
    
    //ADC Autosequencing (16 bit communications rather than 24 bit)
    gpio_put(gpio,0); //GPIO 0 LOW
    spi_write_blocking(spi0, _adcP, 3); //SEND SPI COMMAND
    gpio_put(gpio, 1); //GPIO 0 HIGH

    _adcOut[0] = 0x08;
    _adcOut[1] = 0x10;
    _adcOut[2] = 0x11;

    gpio_put(gpio,0); //GPIO 0 LOW
    spi_write_blocking(spi0, _adcP, 3); //SEND SPI COMMAND
    gpio_put(gpio, 1); //GPIO 0 HIGH

    _adcOut[0] = 0x00;
    _adcOut[1] = 0x00;
    
    gpio_put(gpio,0); //GPIO 0 LOW
    spi_write_blocking(spi0, _adcP, 2); //SEND SPI COMMAND
    gpio_put(gpio, 1); //GPIO 0 HIGH
}

inline void _dacSetup() {
  _dacOut[0] = 0x7F;
  _dacOut[1] = 0x00;
  _dacOut[2] = 0x00; 
  _dacP = _dacOut;
 
  //Set DAC to external ref mode
  gpio_put(2,0); //GPIO 2 LOW
  spi_write_blocking(spi0, _dacP, 3); //SEND SPI COMMAND
  gpio_put(2, 1); //GPIO 2 HIGH

}

inline void readInput(){
  // read audio adc values for each channel
    // store in ADCBuff
    for (int i = 0; i < 8; i++){
      adcRead(1, _adcP, &ADCBuff[i] ) ;    
    }

    //Iterate through the bottom 8 array positions in ADCBuff
    adcRead(0,_adcP,&ADCBuff[currentPot + 8] ) ;

    //Increment to next position 
    if (currentPot < 7) {
      currentPot++;     
    } else {
      currentPot = 0 ; 
    }
}

inline void adcRead(int gpio, const uint8_t* src, uint16_t* dst) {
  uint8_t _adcIn[] = {0x00,0x00}; 
  
  gpio_put(gpio,0); //GPIO LOW
  spi_write_read_blocking(spi0, src, &_adcIn[0], 2 );
  gpio_put(gpio, 1); //GPIO HIGH

  *dst = ( ((uint16_t)_adcIn[0]) << 8) | (uint16_t)_adcIn[1];    
}

void writeOutput(){
  // output buff0 data to DAC while core0 is processing
  for (uint8_t i = 0; i < 8; i++){
    _dacOut[0] = i | 0x20 ; 
    _dacOut[1] = (uint8_t) (buff0[i] >> 8); 
    _dacOut[2] = (uint8_t) buff0[i]; 
    dacWrite(_dacP) ;
  } 
}

inline void dacWrite(const uint8_t* src) {
  gpio_put(2,0); //GPIO 2 LOW
  spi_write_blocking(spi0, src, 3); //SEND SPI COMMAND
  gpio_put(2, 1); //GPIO 2 HIGH
}

void setupSPI(){
  // setup SPI devices
    _adcSetup(1) ; //Audio ADC
    _adcSetup(0) ; //Pot ADC
    _dacSetup() ; 
}

void syncSet(char val){
  mutex_enter_blocking(&_sync0_lock) ; 
  _sync0 = val ; //Core1 Done 
  mutex_exit(&_sync0_lock) ;
}

void syncWait(char val){
  uint8_t mutex_temp = 2;
  // wait for buff1 to be updated
  while( mutex_temp != val ) { 
    mutex_enter_blocking(&_sync0_lock) ; 
    mutex_temp = _sync0 ;
    mutex_exit(&_sync0_lock) ;       
  }
}

void moveBuff(){
  memcpy(buff0, buff1, 16);
}

void start_sample(){
  // wait for ADC data to be ready
  uint8_t mutex_temp = 2;
  while( mutex_temp != 1 ) { 
    mutex_enter_blocking(&_sync0_lock) ; 
    mutex_temp = _sync0 ;
    mutex_exit(&_sync0_lock) ;       
  }
}

void end_sample(){
  // indicate to core1 that buff1 is updated   
  mutex_enter_blocking(&_sync0_lock) ; 
  _sync0 = 0 ; //Core1 Done 
  mutex_exit(&_sync0_lock) ;
}


void core1_SPI(){
  
  setupSPI();
  
  while(1){
    readInput();

    // indicate to core0 that ADC values are ready
    syncSet(1);

    // output buff0 data to DAC while core0 is processing
    writeOutput();

    syncWait(0);

    moveBuff();
  }
}
