uint8_t dacOut[] = {0x20,0x00,0x00}; //DAC Command Registers
const uint8_t* dacP = dacOut; 

uint8_t adcOut[] = {0x08,0x12,0xFF}; //ADC Command Register
const uint8_t* adcP = adcOut;

uint8_t adcIn[] = {0x00,0x00}; //ADC Read Register (In sequencing mode ADC only reads out 16 bit value)
uint8_t* adcVal = adcIn;
 
//HUGE NOTE: SPI SCK and SS pins are flipped //
#include <SPI.h>
SPISettings parSPI(31250000, MSBFIRST, SPI_MODE0);

#define mixChannels 4

void setup() {
  pinMode(2, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(0, OUTPUT);

  SPI.begin(true);
  SPI.beginTransaction(parSPI);

  Serial.begin(9600);

  adcSetup(0) ; //Potentiometer Setup
  adcSetup(1) ; //Audio Setup
  dacSetup(); // DAC Setup 

}

void loop() {
  // read audio inputs
  // read potentiometers (less frequently)
  // scale and sum inputs
  // ouput to single channel
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

//Setup ADC so it is in sequencing mode. Allows for faster SPI transactions of 16 bits rather than 24
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

//Read out data from sequenced ADC
void adcRead(int gpio, const uint8_t* output, uint8_t* input) {
  gpio_put(gpio,0); //GPIO 2 LOW
  spi_write_read_blocking(spi0, output, input, 2 );
  gpio_put(gpio, 1); //GPIO 2 HIGH
}

//Write data to DAC from output command
void dacWrite(const uint8_t* output) {
  gpio_put(2,0); //GPIO 2 LOW
  spi_write_blocking(spi0, dacP, 3); //SEND SPI COMMAND
  gpio_put(2, 1); //GPIO 2 HIGH
}
