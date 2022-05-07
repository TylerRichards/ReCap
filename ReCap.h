#ifndef ReCap_h
#define ReCap_h

#include "Arduino.h"
#include "pico/multicore.h"
#include <SPI.h>
#include "pico/sync.h"


void ReCap_init();
extern uint16_t ADCBuff[16];
extern uint16_t buff0[8];
extern uint16_t buff1[8];
void start_sample();
void end_sample();
void readInput();
void writeOutput();
void setupSPI();
extern uint8_t currentPot;
void syncSet(char val);
void syncWait(char val);
void moveBuff();
extern SPISettings _parSPI;
extern mutex_t _sync0_lock;
extern char _sync0;
extern uint8_t _dacOut[3]; 
extern const uint8_t* _dacP; 
inline void adcRead(int gpio, const uint8_t* src, uint16_t* dst);
inline void dacWrite(const uint8_t* src);
extern uint8_t _adcOut[3]; 
extern const uint8_t* _adcP;
extern uint8_t _adcIn[3]; 
extern uint8_t* _adcVal;
inline void _adcSetup(int gpio);
inline void _dacSetup();
void core1_SPI();

#endif