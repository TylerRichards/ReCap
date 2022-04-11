uint8_t dacOut[] = {0x20,0x00,0x00}; //DAC Command Registers
const uint8_t* dacP = dacOut; 

uint8_t adcOut[] = {0x08,0x12,0xFF}; //ADC Command Register
const uint8_t* adcP = adcOut;

uint8_t adcIn[] = {0x00,0x00}; //ADC Read Register (In sequencing mode ADC only reads out 16 bit value)
uint8_t* adcVal = adcIn;

//HUGE NOTE: SPI SCK and SS pins are flipped //
#include <SPI.h>
SPISettings parSPI(31250000, MSBFIRST, SPI_MODE0);

uint32_t i = 0;
uint8_t channel = 0;

#define bufferSize 30000

uint8_t delayBuffer[8][bufferSize];

void setup() {
  pinMode(2, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(0, OUTPUT);

  SPI.begin(true);
  SPI.beginTransaction(parSPI);

  adcSetup(0) ; //Potentiometer Setup
  adcSetup(1) ; //Audio Setup

  dacSetup() ;   

  for (int i = 0; i < 8; i++) {
    for (int z = 0; z < bufferSize; z++) {
      delayBuffer[i][z] = 0;
    }
  }
}

void loop() {
  adcRead(1, adcP, adcVal) ;

  delayBuffer[channel][i] = adcIn[0] ; 
  delayBuffer[channel][i+1]  = adcIn[1] ; 
  
  //if (channel == 0) {
  //Serial.println((delayBuffer[channel][i]<<8) + delayBuffer[channel][i+1]);
  //}
  
  dacOut[0] = (0x07 & channel) | (0xF0 & dacOut[0]) ; 

  if (i == bufferSize-2){
    dacOut[1] = delayBuffer[channel][0] ; 
    dacOut[2] = delayBuffer[channel][1] ; 
  } else {
    dacOut[1] = delayBuffer[channel][i+2] ; 
    dacOut[2] = delayBuffer[channel][i+3] ; 
  }
  
  dacWrite(dacP) ; 
  
  /*
  if (channel == 0) {
    if (i == bufferSize-2){
      //Serial.println(i);
      Serial.println((delayBuffer[channel][0]<<8) + delayBuffer[channel][1]) ; 
    } else {
      //Serial.println(i);
      Serial.println((delayBuffer[channel][i+2]<<8) + delayBuffer[channel][i+3]) ; 
    }    
  }
  */
  
  channel++ ;
  
  if (channel > 7) {
    i+= 2 ; 
    channel = 0;
    
    if (i > bufferSize - 2) {
      i = 0;
    }
    
  }

  
}

//Setup ADC so it is in sequencing mode. Allows for faster SPI transactions of 16 bits rather than 24
void adcSetup(int gpio) {
  uint8_t adcOut_setup[] = {0x08,0x12,0xFF}; 
  const uint8_t* adcP_setup = adcOut_setup;
  
  uint8_t adcIn_setup[] = {0x00,0x00}; 
  uint8_t* adcVal_setup = adcIn_setup;
 
  //ADC Autosequencing (16 bit communications rather than 24 bit)
  gpio_put(gpio,0); //GPIO 0 LOW
  spi_write_blocking(spi0, adcP_setup, 3); //SEND SPI COMMAND
  gpio_put(gpio, 1); //GPIO 0 HIGH

  adcOut_setup[0] = 0x08;
  adcOut_setup[1] = 0x10;
  adcOut_setup[2] = 0x11;

  gpio_put(gpio,0); //GPIO 0 LOW
  spi_write_blocking(spi0, adcP_setup, 3); //SEND SPI COMMAND
  gpio_put(gpio, 1); //GPIO 0 HIGH

  adcOut_setup[0] = 0x00;
  adcOut_setup[1] = 0x00;
  
  gpio_put(gpio,0); //GPIO 0 LOW
  spi_write_blocking(spi0, adcP_setup, 2); //SEND SPI COMMAND
  gpio_put(gpio, 1); //GPIO 0 HIGH
}

//Read out data from sequenced ADC
inline void adcRead(int gpio, const uint8_t* output, uint8_t* input) {
  gpio_put(gpio,0); //GPIO 2 LOW
  spi_write_read_blocking(spi0, output, input, 2 );
  gpio_put(gpio, 1); //GPIO 2 HIGH
}

//Write data to DAC from output command
inline void dacWrite(const uint8_t* output) {
  gpio_put(2,0); //GPIO 2 LOW
  spi_write_blocking(spi0, dacP, 3); //SEND SPI COMMAND
  gpio_put(2, 1); //GPIO 2 HIGH
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
