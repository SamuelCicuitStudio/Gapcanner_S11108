
#include <Arduino.h>
#include "MegunoLink.h"
#include "Filter.h"

// Exponential filters for data smoothing
ExponentialFilter<long> ADCFilter(10, 0);
ExponentialFilter<long> ADCFilter1(3, 0);

// Define pin assignments
#define TCD_CLK             7       // TCD Clock Pin (FlexPWM1.3)
#define TCD_SH              4       // TCD SH (Sample and Hold) Pin (FlexPWM2.0)
#define TCD_ICG             5       // TCD ICG (Integration Control Gate) Pin (FlexPWM2.1)
#define DATA_PIN            A0      // Sensor Analog Output Pin
#define TCD_ADC_Trigger     6       // TCD ADC Trigger Pin (FlexPWM2.2)


// Define constants
#define NUM_PIXELS 1546
#define PIXELSIZE 5.5 // Pixel size in µm (micrometers)

uint16_t PrevgapWidth = 0; 

// System parameters
int thresholdPercentage = 30; // For example, 10% of the range
bool IsLonger = false;        // Parameter to determine if the strip width is longer than the sensor width
#define steadyError 50        // output error

// Filtered data array
uint16_t discretData[NUM_PIXELS];
// Arrays for filtered data
uint16_t CCDPixelBuffer[NUM_PIXELS]; // Array to store pixel data


// Volatile variables
volatile uint16_t pixelCount = 0; // Count of pixels
volatile float gapWidth = 0;      // Gap width in µm (micrometers)

// Variables for threshold and filtering
uint16_t minVal = 1024;             // Initialize minVal with the maximum possible value
uint16_t maxVal = 0;                // Initialize maxVal with 0
#define thresholdPercentage 30      // Threshold percentage for gap calculation


// Function prototypes
void TCD1304Init();
void TCD1304Clk();
void showData(uint16_t sensorData[NUM_PIXELS]);
uint16_t calculateGapWidth(uint16_t SensorData[NUM_PIXELS]);
void showMeasuredGap(uint16_t SensorData[NUM_PIXELS]);


void setup(){
 Serial.begin(460800);
 TCD1304Init();
 TCD1304Clk();
}

void loop(){
showMeasuredGap(CCDPixelBuffer);
 delay(1000);
}
void TCD1304Init(){

 pinMode(TCD_CLK,OUTPUT);
 analogWriteFrequency(TCD_CLK,2000000);             // Set the PWM frequency to 2MHz on Pin Clock
 analogWriteResolution(6);                          // Set the analog write resolution to 6 bits (64 levels)
 

 pinMode(TCD_ADC_Trigger,OUTPUT);
 analogWriteFrequency(TCD_ADC_Trigger,1000000);      // Set the PWM frequency to 1MHz on  to trigger ADC conversion
 analogWriteResolution(8);                          // Set the analog write resolution to 8 bits (256 levels)


 pinMode(TCD_ICG,OUTPUT);
 analogWriteFrequency(TCD_ICG,133);                 // Set the PWM frequency to 133Hz on  to trigger ADC conversion
 analogWriteResolution(15);                         // Set the analog write resolution to 15 bits (32768 levels)

//20µs integration time
 pinMode(TCD_SH,OUTPUT);
 analogWriteFrequency(TCD_SH,50000);                 // Set the PWM frequency to 50kHz on  to trigger ADC conversion
 analogWriteResolution(11);                          // Set the analog write resolution to 11 bits (2048 levels)

 
}

void TCD1304Clk(){
 analogWrite(TCD_CLK, 32);                           // Set a duty cycle of 50%
 analogWrite(TCD_ADC_Trigger,64);                    // Set a duty cycle of 25% 
 analogWrite(TCD_ICG,4369);                          // Set a duty cycle of 0.133% 
 delayNanoseconds(500);
 analogWrite(TCD_SH, 410);                           // Set a duty cycle of 20% 
}

void showData(uint16_t sensorData[NUM_PIXELS]) {
  for (int i = 0; i < NUM_PIXELS+1; i++) {
    Serial.print("[");
    Serial.print(sensorData[i]);
    Serial.print("] ");

    // Print a new line after every 100 values
    if ((i + 1) % 100 == 0) {
      Serial.println();
    }
  }
}

// Function to calculate the gap width between plastic flat strips
uint16_t calculateGapWidth(uint16_t SensorData[]) {
  // Initialize minVal and maxVal with the first element of the array
  uint16_t minVal = SensorData[0];
  uint16_t maxVal = SensorData[0];

  // Calculate the minimum value in SensorData[]
  for (int i = 0; i < NUM_PIXELS; i++) {
    if (SensorData[i] < minVal) {
      minVal = SensorData[i];
    }
  }

  // Calculate the maximum value in SensorData[]
  for (int i = 0; i < NUM_PIXELS; i++) {
    if (SensorData[i] > maxVal) {
      maxVal = SensorData[i];
    }
  }

  // Calculate the thresholds based on a percentage of the range
  int highThreshold = maxVal - (maxVal - minVal) * thresholdPercentage / 100;
  int lowThreshold = minVal + (maxVal - minVal) * thresholdPercentage / 100;

  discretData[0] = minVal;

  // Iterate through SensorData to apply Schmitt trigger and store values
  for (int i = 0; i < NUM_PIXELS; i++) {
    // Apply Schmitt trigger logic
    if (SensorData[i] > highThreshold) {
      discretData[i] = maxVal; // Store maxVal for high region
    } else if (SensorData[i] < lowThreshold) {
      discretData[i] = minVal; // Store minVal for low region
    } else {
      // If not clearly high or low, keep the previous state
      discretData[i] = discretData[i - 1];
    }
  }

  // Iterate through SensorData to find edges and calculate gaps
  uint8_t count = 0;
  int gapCount = 0;

  for (int j = 0; j < NUM_PIXELS; j++) {
    if (discretData[j - 1] == maxVal && discretData[j] == minVal) {
      count++;
    }
    count = count / 2;
  }

  if (!IsLonger) {
    for (int i = 50; i < 800; i++) {
      // Skip until the first maxVal is encountered
      while (discretData[i] <= minVal && discretData[i] < maxVal) i++;

      // Skip until the following minVal is encountered (inside the gap)
      while (discretData[i] >= maxVal && discretData[i] > minVal) i++;

      // Count minVal elements inside the gap
      while (discretData[i] <= minVal && discretData[i] < maxVal) {
        gapCount++;
        i++;
      }
      return gapCount;
    }
  } else {
    for (int i = 50; i < 800; i++) {
      if (discretData[i - 1] >= maxVal && discretData[i] <= minVal) {
        gapCount++;
      }
      return gapCount;
    }
  }
}

// Calculate the gap width and return it
uint16_t GetGapWidth(uint16_t SensorData[]) {
  uint16_t gap = calculateGapWidth(CCDPixelBuffer);
  if (gap < 60 && gap > 3) {
    PrevgapWidth = gap;
    return (gap * 100);
  } else {
    return PrevgapWidth * 100;
  }
}


void showMeasuredGap(uint16_t SensorData[NUM_PIXELS]){
pixelCount =0;
  gapWidth=0;
  while(digitalRead(TCD_ICG)){
    while(digitalRead(TCD_CLK)){
      ///Serial.print("OS =");
      //Serial.println(analogRead(DATA_PIN));
      pixelCount++;
      if (pixelCount > 32 && pixelCount <= NUM_PIXELS + 32) {
        CCDPixelBuffer[pixelCount - 88] = analogRead(DATA_PIN);
        //Serial.println(sensorData[pixelCount - 33]);
        };
      if (pixelCount > NUM_PIXELS + 32){ 
        //showData(sensorData);
        // Apply exponential filtering to the calculated gap width
      ADCFilter.Filter(GetGapWidth(CCDPixelBuffer));
      int bot1 = ADCFilter.Current();
      ADCFilter1.Filter(bot1);

       // Create a TimePlot object and send the filtered data
      TimePlot Plot;
      Plot.SendData("Gapwidth", ADCFilter1.Current() + steadyError);
      delay(50);
        gapWidth = calculateGapWidth(CCDPixelBuffer);
        if(gapWidth>0){
          Serial.print("Estimated Gap Width: ");
          Serial.print(gapWidth);
          Serial.println(" µm");
        } ;break;
      }
    };break;
  }

}
