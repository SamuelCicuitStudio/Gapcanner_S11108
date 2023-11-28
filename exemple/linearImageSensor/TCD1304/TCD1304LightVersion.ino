#include <Arduino.h>
#include "MegunoLink.h"
#include <ADC.h>
#include <ADC_util.h>
#include <SPI.h> 
#define PIXELS 3648
#define MCLK 4 

 /*
 (ICG)  - pin 5
 (MCLK) - pin 6
 (SH)   - pin 7
 OS)    - pin A9
 */



int CLKPMCLK = ceil((float)F_BUS_ACTUAL/(MCLK*1000000));
int MCLKPF = ceil((float)(32+PIXELS+14)*4/128);
int CNTPF = MCLKPF * CLKPMCLK; 
int CLKPF = 128 * CNTPF; 
int USPF = CLKPF / (F_BUS_ACTUAL / 1000000); 
int CNT_ICG = (6.0 * (F_BUS_ACTUAL / 1000000))/128; 
int ES = 128;
int CNT_SH = (1.0 * (F_BUS_ACTUAL / 1000000))/ (128 / ES); 
int off = (0.5 * (F_BUS_ACTUAL / 1000000))/(128 / ES); 
int adc_off = (0.0 * (F_BUS_ACTUAL / 1000000));

ADC *adc; 

int i = 0;
int ii = 0;
int s = 1;
byte vals[2][3*PIXELS/2];
int j = 0;
int f = 0;
uint16_t sample = 0;
uint16_t sample_prev = 0;

void setup() {
   adc = new ADC();
  adc->adc0->setAveraging(1);  
  adc->adc0->setResolution(12);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
  adc->adc0->startContinuous(A9); 
  
  Serial.begin(57600);
  // set up clocks  
  CCM_CCGR4 |= CCM_CCGR4_PWM2(CCM_CCGR_ON);
  
  //FLEXPWM2_MCTRL = 0; // make sure RUN flags are off
  FLEXPWM2_FCTRL0 = FLEXPWM_FCTRL0_FLVL(15); // logic high = fault
  FLEXPWM2_FSTS0 = 0x000F; // clear fault status
  FLEXPWM2_FFILT0 = 0;
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_CLDOK(15);

  setup_ICG();
  setup_SH();
  setup_MCLK();
  setup_ADCCLK();
   
  FLEXPWM2_MCTRL |= FLEXPWM_MCTRL_LDOK(15);
  // attach interrupts
  attachInterruptVector(IRQ_FLEXPWM2_1, flexpwm2_1_isr);
  NVIC_ENABLE_IRQ(IRQ_FLEXPWM2_1);
  attachInterruptVector(IRQ_FLEXPWM2_3, flexpwm2_3_isr);
  NVIC_ENABLE_IRQ(IRQ_FLEXPWM2_3);
  NVIC_SET_PRIORITY(IRQ_FLEXPWM2_3, 128);

}


void loop(){
  
  TimePlot Plot;
  Plot.SendData("Gapwidth",analogRead(A0));
  delay(50);

}


void flexpwm2_1_isr() {
  FLEXPWM2_SM1STS = 1;
  while (FLEXPWM2_SM1STS == 1);
  i = 0;
  ii = 0;
}

void flexpwm2_3_isr() {
  FLEXPWM2_SM3STS |= 12; 

  if ((i >=32) && (i < (32 + PIXELS))) {
    sample_prev = sample;
    while (!adc->adc0->isComplete()); // wait for conversion
    sample = (uint16_t)adc->adc0->analogReadContinuous();
    if (i%2 == 1) { // only odd indices
      // bit shift and stuff to fit 2 samples into 3 bytes, MSB first
      vals[j][ii] = highByte(sample_prev) << 4 | lowByte(sample_prev) >> 4;
      vals[j][ii+1] = lowByte(sample_prev) << 4 | highByte(sample);
      vals[j][ii+2] = lowByte(sample);
      ii += 3;
      if (ii >= (3*PIXELS/2)){ 
        j = !j;  
        f = 1; 
      }
    }
  }
  i += 1;
  
}

void setup_ICG() {
  IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_08 = 1;
  FLEXPWM2_SM1CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
  FLEXPWM2_SM1CTRL2 |= 128;

  FLEXPWM2_SM1CTRL = 112; 
  FLEXPWM2_SM1CTRL |= 2048;
  
  FLEXPWM2_SM1OCTRL = 0; 
  //FLEXPWM2_SM2OCTRL |= 1024; // set A polarity to 1, should swap high and low
  FLEXPWM2_SM1DTCNT0 = 0;
  FLEXPWM2_SM1INIT = 0; // set counter initialization to 0

  // we think of the waveform 0 as beginning when ICG goes high. We want all clocks to be synchronized relative to this moment
  FLEXPWM2_SM1VAL0 = CNTPF - 2; // set VAL0 to trigger an interrupt that will rest pixel count 128 clock cycles before ICG goes high; this should occur between ADC reads (which at max happen once ever 38*4 = 172 bus clock cycles), be careful if you have a large adc offset but i think that will stay near 0
  FLEXPWM2_SM1VAL1 = CNTPF - 1; 
  FLEXPWM2_SM1VAL2 = 0; 
  FLEXPWM2_SM1VAL3 = CNTPF - CNT_ICG;
  FLEXPWM2_SM1VAL4 = 0;
  FLEXPWM2_SM1VAL5 = 0;

  FLEXPWM2_OUTEN |= 512; // set output enable for A, module 1 

  // set up interrupt
  FLEXPWM2_SM1STS |= 0; // set val0 compare flag to 0
  FLEXPWM2_SM1INTEN = 1; // enable interrupts for val0 compare

}

void setup_SH() {
  //IOMUXC_SW_MUX_CTL_PAD_GPIO_B1_01 = 1; // set pin 7 to be PWM1 B SM 03
  //IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_06 = 1; // set pin 4 to be PWM A SM 0
  IOMUXC_SW_PAD_CTL_PAD_GPIO_B1_00 = 1;// set pin 8

  FLEXPWM2_SM0CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
  FLEXPWM2_SM0CTRL2 |= 128; // set force enable

  if (ES == 1) FLEXPWM2_SM0CTRL = 112; // set prescaler to 128
  if (ES == 2) FLEXPWM2_SM0CTRL = 96; // set prescaler to 64
  if (ES == 4) FLEXPWM2_SM0CTRL = 80; // set prescaler to 32
  if (ES == 8) FLEXPWM2_SM0CTRL = 64; // set prescaler to 16
  if (ES == 16) FLEXPWM2_SM0CTRL = 48; // set prescaler to 8
  if (ES == 32) FLEXPWM2_SM0CTRL = 32; // set prescaler to 4
  if (ES == 64) FLEXPWM2_SM0CTRL = 16; // set prescaler to 2

  FLEXPWM2_SM0CTRL |= 2048; // reload on half count (actually reloads immediately since val0 is set to 0)
  
  FLEXPWM2_SM0OCTRL = 0; // output control register, bits 10 and 9 are A, B polarity
  //FLEXPWM2_SM0OCTRL |= 1024; // set A polarity to 1, should swap high and low
  FLEXPWM2_SM0DTCNT0 = 0;
  FLEXPWM2_SM0INIT = 0; // set counter initialization to 0

  FLEXPWM2_SM0VAL0 = 0; 
  FLEXPWM2_SM0VAL1 = CNTPF - 1; 
  FLEXPWM2_SM0VAL2 = CNTPF - ES*CNT_ICG + off; 
  FLEXPWM2_SM0VAL3 = CNTPF - ES*CNT_ICG + off + CNT_SH; // pulse SH high for CNT_SH counts
  FLEXPWM2_SM0VAL4 = 0;
  FLEXPWM2_SM0VAL5 = 0;
  FLEXPWM2_OUTEN |= 256; // set output enable for A, module 0

}

void setup_MCLK() {
  IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_10 = 2; // set pin 6 to be PWM2 A SM 02
  
  FLEXPWM2_SM2CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
  FLEXPWM2_SM2CTRL2 |= 128; // set force enable
  FLEXPWM2_SM2CTRL |= 2048; // reload on half count
  
  FLEXPWM2_SM2OCTRL = 0; // output control register, bits 10 and 9 are A, B polarity
  FLEXPWM2_SM2DTCNT0 = 0;
  FLEXPWM2_SM2INIT = 0; // set counter initialization to 0

  FLEXPWM2_SM2VAL0 = 0; // set VAL0 to be halfway through counter??
  FLEXPWM2_SM2VAL1 = CLKPMCLK - 1; // each clock tick is 2/3us with 16x prescaler
  FLEXPWM2_SM2VAL2 = 0; // for now, no phase shift
  FLEXPWM2_SM2VAL3 = CLKPMCLK/2; // pulse ICG low for CNT_ICG counts
  FLEXPWM2_SM2VAL4 = 0;
  FLEXPWM2_SM2VAL5 = 0;
  FLEXPWM2_OUTEN |= 1024; // set output enable for A, module 2 

}

void setup_ADCCLK() {

  FLEXPWM2_SM3CTRL2 = FLEXPWM_SMCTRL2_INDEP | FLEXPWM_SMCTRL2_WAITEN | FLEXPWM_SMCTRL2_DBGEN;
  FLEXPWM2_SM3CTRL2 |= 128; // set force enable
  FLEXPWM2_SM3CTRL |= 2048; // reload on half count
  
  FLEXPWM2_SM3OCTRL = 0; // output control register, bits 10 and 9 are A, B polarity
  FLEXPWM2_SM3DTCNT0 = 0;
  FLEXPWM2_SM3INIT = 0; // set counter initialization to 0

  FLEXPWM2_SM3VAL0 = 0; // set VAL0 to be halfway through counter??
  FLEXPWM2_SM3VAL1 = 8*CLKPMCLK - 1; // each clock tick is 2/3us with 16x prescaler
  FLEXPWM2_SM3VAL2 = adc_off; 
  FLEXPWM2_SM3VAL3 = adc_off + 4*CLKPMCLK; // set up an edge ever 4 MCLK cycles
  FLEXPWM2_SM3VAL4 = 0; // for now, no phase shift
  FLEXPWM2_SM3VAL5 = 0; // remember B uses 4 and 5
  FLEXPWM2_SM3STS |= 0<<3; // set val3 compare flag to 0
  FLEXPWM2_SM3STS |= 0<<2; // set val2 compare flag to 0
  FLEXPWM2_SM3INTEN = 1<<3; // enable interrupts for val3 compare
  FLEXPWM2_SM3INTEN |= 1<<2; // enable interrupts for val2 compare

}
