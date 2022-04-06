uint8_t dacOut[] = {0x20,0x00,0x00}; 
const uint8_t* dacP = dacOut; 

uint8_t adcOut[] = {0x08,0x12,0xFF}; 
const uint8_t* adcP = adcOut;

uint8_t adcIn[] = {0x00,0x00}; 
uint8_t* adcVal = adcIn;
 
//HUGE NOTE: SPI SCK and SS pins are flipped //
#include <SPI.h>
SPISettings parSPI(31250000, MSBFIRST, SPI_MODE0);

int i = 0 ; 

void setup() {
  
  pinMode(2, OUTPUT);
  pinMode(0, OUTPUT);

  SPI.begin(true);
  SPI.beginTransaction(parSPI);

  //Serial.begin(9600);

  adcSetup(0) ; //Audio ADC
  dacSetup() ; 

}

void loop() {
  adcRead(0, adcP, adcVal) ; 

  dacOut[0] = (0x07 & i) | (0xF0 & dacOut[0]) ; 
  dacOut[1] = adcIn[0] ; 
  dacOut[2] = adcIn[1] ; 

  dacWrite(dacP) ; 
  
  i++ ; 
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

void adcRead(int gpio, const uint8_t* output, uint8_t* input) {
  gpio_put(gpio,0); //GPIO 2 LOW
  spi_write_read_blocking(spi0, output, input, 2 );
  gpio_put(gpio, 1); //GPIO 2 HIGH
}

void dacWrite(const uint8_t* output) {
  gpio_put(2,0); //GPIO 2 LOW
  spi_write_blocking(spi0, dacP, 3); //SEND SPI COMMAND
  gpio_put(2, 1); //GPIO 2 HIGH
}
