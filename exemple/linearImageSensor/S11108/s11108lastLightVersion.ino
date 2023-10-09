#include <Arduino.h>
#include "MegunoLink.h"
#include "Filter.h"

// Exponential filters for data smoothing
ExponentialFilter<long> ADCFilter(10, 0);
ExponentialFilter<long> ADCFilter1(3, 0);

// Pin Definitions
#define CLOCK_PIN       5     // Clock Signal PWM Pin (FlexPWM1.1_Channel_1)
#define TRIGG_PIN       9     // Trigger Signal Pin
#define ST_PIN          29    // ST Signal PWM Pin (FlexPWM3.1_Channel_2)
#define EOS_PIN         4     // EOS Signal Pin
#define DATA_PIN        A0    // Sensor Analog Output Pin
#define SAVE_PIN        28    // Save Signal PWM Pin (FlexPWM3.1_Channel_1)
#define TRIGG_SAVE_PIN  11    // Data Storage Trigger interrupt pin linked to SAVE_PIN physically

// Constants Related to S11108
#define NUM_PIXELS      2048  // Number of pixels in the sensor
#define PIXEL_SIZE      14.0  // Pixel size in micrometers (14 μm)

// Variables
uint16_t pixelCount = 0;
uint16_t PrevgapWidth = 0; 

// System parameters
int thresholdPercentage = 30; // For example, 10% of the range
bool IsLonger = false;        // Parameter to determine if the strip width is longer than the sensor width
#define steadyError 50        // output error

// Filtered data array
uint16_t discretData[NUM_PIXELS];
// Sensor Data Array
uint16_t sensorData[NUM_PIXELS];

// Function Prototypes
uint16_t calculateGapWidth(uint16_t sensorData[]);
uint16_t showMeasuredGap();
uint16_t GetGapWidth(uint16_t SensorData[]);
void configureAndStartTimers();
void PinConfiguration();

void setup() {
  Serial.begin(460800);  // Initialize serial communication
  while (!Serial && millis() < 5000); // Wait for the serial port to open (for debugging)
  delay(5000); // Delay for stability  
  configureAndStartTimers();  // Configure and start timers for Clock, ST, and SAVE pins
  PinConfiguration();// Configure TRIGG_PIN, TRIGG_SAVE_PIN, and EOS_PIN 
}

void loop() {

 showMeasuredGap();

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
  uint16_t gap = calculateGapWidth(sensorData);
  if (gap < 60 && gap > 3) {
    PrevgapWidth = gap;
    return (gap * 100);
  } else {
    return PrevgapWidth * 100;
  }
}

// Function to display sensor data (not used in loop)
uint16_t showMeasuredGap() {
   
  pixelCount = 0;
  while (!digitalRead(EOS_PIN)) {
    while (digitalRead(TRIGG_PIN)) {
      pixelCount++;
      if (pixelCount > 87 && pixelCount <= NUM_PIXELS + 87) {
        sensorData[pixelCount - 88] = analogRead(DATA_PIN);
      }
      if (pixelCount > NUM_PIXELS + 87) {
        goto out;
      }
    }
  }
out:;
  
  // Apply exponential filtering to the calculated gap width
  ADCFilter.Filter(GetGapWidth(sensorData));
  int bot1 = ADCFilter.Current();
  ADCFilter1.Filter(bot1);

  // Create a TimePlot object and send the filtered data
  TimePlot Plot;
  Plot.SendData("Gapwidth", ADCFilter1.Current() + steadyError);
  delay(50);
  return ADCFilter1.Current() + steadyError;

}

// Configure and start timers for Clock, ST, and SAVE pins
void configureAndStartTimers() {
  pinMode(CLOCK_PIN, OUTPUT);
  analogWriteFrequency(CLOCK_PIN, 375000);
  analogWriteResolution(10);

  pinMode(ST_PIN, OUTPUT);
  analogWriteFrequency(ST_PIN, 175);
  analogWriteResolution(10);

  pinMode(SAVE_PIN, OUTPUT);
  analogWriteFrequency(SAVE_PIN, 175);
  analogWriteResolution(10);

  analogWrite(ST_PIN, 999);
  analogWrite(CLOCK_PIN, 512);
  analogWrite(SAVE_PIN, 999);
}

// Configure  pins
void PinConfiguration() {
  pinMode(TRIGG_PIN, INPUT_PULLUP);
  pinMode(TRIGG_SAVE_PIN, INPUT);
  pinMode(EOS_PIN, INPUT);
}
