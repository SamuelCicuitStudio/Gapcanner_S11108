#include <Arduino.h>

// Define pin assignments
#define TCD_CLK             7       // TCD Clock Pin (FlexPWM1.3)
#define TCD_SH              4       // TCD SH (Sample and Hold) Pin (FlexPWM2.0)
#define TCD_ICG             5       // TCD ICG (Integration Control Gate) Pin (FlexPWM2.1)
#define DATA_PIN            A0      // Sensor Analog Output Pin
#define TCD_ADC_Trigger     6       // TCD ADC Trigger Pin (FlexPWM2.2)


// Define constants
#define NUM_PIXELS 3648
#define PIXELSIZE 8 // Pixel size in µm (micrometers)

// Arrays for filtered data
uint16_t CCDPixelBuffer[NUM_PIXELS]; // Array to store pixel data
uint16_t filteredData[NUM_PIXELS];

// Volatile variables
volatile uint16_t pixelCount = 0; // Count of pixels
volatile bool SaveFlag = false;   // Save flag
volatile float gapWidth = 0;      // Gap width in µm (micrometers)

// Variables for threshold and filtering
uint16_t minVal = 1024;             // Initialize minVal with the maximum possible value
uint16_t maxVal = 0;                // Initialize maxVal with 0
#define thresholdPercentage 30      // Threshold percentage for gap calculation
#define alpha 0.7                   // Alpha value for low-pass filter



// Default sampling value
#define DefaultSampling 200
uint8_t sampling = DefaultSampling;

// Function prototypes
void TCD1304Init();
void TCD1304Clk();
void showData(uint16_t sensorData[NUM_PIXELS]);
uint16_t calculateGapWidth(uint16_t SensorData[NUM_PIXELS]);
void applyLowPassFilter(uint16_t sensorData[NUM_PIXELS], uint16_t filteredData[NUM_PIXELS]);
void getTCD1304Data(uint16_t SensorData[NUM_PIXELS]);


void setup(){
 Serial.begin(115200);
 TCD1304Init();
 TCD1304Clk();
}

void loop(){
getTCD1304Data(CCDPixelBuffer);
 delay(1000);
}
void TCD1304Init(){

 pinMode(TCD_CLK,OUTPUT);
 analogWriteFrequency(TCD_CLK,2000000);             // Set the PWM frequency to 2MHz on Pin Clock
 analogWriteResolution(6);                          // Set the analog write resolution to 6 bits (64 levels)
 

 pinMode(TCD_ADC_Trigger,OUTPUT);
 analogWriteFrequency(TCD_ADC_Trigger,500000);      // Set the PWM frequency to 500kHz on  to trigger ADC conversion
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
uint16_t calculateGapWidth(uint16_t SensorData[NUM_PIXELS]) {
    // Initialize minVal and maxVal with the first element of the array
    uint16_t minVal = SensorData[0];
    uint16_t maxVal = SensorData[0];

    // Calculate the minimum and maximum values in SensorData[]
    for (int i = 0; i < NUM_PIXELS; i++) {
        if (SensorData[i] < minVal) {
            minVal = SensorData[i];
        }
        if (SensorData[i] > maxVal) {
            maxVal = SensorData[i];
        }
    }



    int highThreshold = maxVal - (maxVal - minVal) * thresholdPercentage / 100;
    int lowThreshold = minVal + (maxVal - minVal) * thresholdPercentage / 100;

    // Initialize variables for gap width calculation

    bool firstRisingEdgeDetected = false; // Flag to track the first rising edge
    int firstRisingEdgeIndex = 0; // Index of the first rising edge
     int secondRisingEdgeIndex = 0; // Index of the first rising edge
    int startingPoint=0;
   // Minimum and maximum gap widths to consider as valid gaps
    int minValidGapWidth = 71; // Minimum gap width in pixels
    int maxValidGapWidth = 428; // Maximum gap width in pixels


    // Iterate through SensorData
    for (int i = 0; i < NUM_PIXELS; i++) {
        if (!firstRisingEdgeDetected && SensorData[i] < lowThreshold && SensorData[i+1] > highThreshold) {
            // First rising edge detected
            firstRisingEdgeDetected = true;
            firstRisingEdgeIndex = i;
        };
         if (firstRisingEdgeDetected && SensorData[i] < lowThreshold && SensorData[i+1] > highThreshold) {
            // Second rising edge detected
             secondRisingEdgeIndex = i;
             startingPoint = secondRisingEdgeIndex - 100;
             Serial.print("startingPoint:");
             Serial.println(startingPoint);

            
        }
    }
          if(startingPoint == 0) return 0;
    // Initialize variables to count low pixels to the left and right
    int lowPixelsLeft = 0;
    int lowPixelsRight = 0;

    // Initialize flag to track if we have encountered the first high pixel
    bool firstHighPixelEncountered = false;
    // Iterate through SensorData starting from the middle
    for (int i = startingPoint; i >= 0; i--) {
        if (SensorData[i] > highThreshold) {
            // High pixel value detected
            firstHighPixelEncountered = true;
            break; // Exit the loop
        } else {
            // Low pixel value detected
            lowPixelsLeft++;
        }
    }

    // Reset flag for the right side
    firstHighPixelEncountered = false;

    // Iterate through SensorData starting from the middle
    for (int i = startingPoint + 1; i < NUM_PIXELS; i++) {
        if (SensorData[i] > highThreshold) {
            // High pixel value detected
            firstHighPixelEncountered = true;
            break; // Exit the loop
        } else {
            // Low pixel value detected
            lowPixelsRight++;
        }
    }

    // Calculate the gap width in micrometers (µm)
    int gapWidthMicrometers =( (lowPixelsLeft + lowPixelsRight) * PIXELSIZE);

    return gapWidthMicrometers; // Return the gap width in µm
}

void applyLowPassFilter(uint16_t sensorData[NUM_PIXELS], uint16_t filteredData[NUM_PIXELS]) {
  if (NUM_PIXELS == 0) {
    // Handle the case of an empty array
    return;
  }

  // Initialize the filtered data with the first element of the input data
  filteredData[0] = sensorData[0];

  // Initialize previous values for the filter
  float prevFilteredData = sensorData[0];
  float prev2FilteredData = sensorData[0];

  // Apply the second-order low-pass filter
  for (size_t i = 1; i < NUM_PIXELS; i++) {
    float input = sensorData[i];

    // Calculate the filtered value using the second-order low-pass filter formula
    float filteredValue = alpha * input + (1 - alpha) * (2 * prevFilteredData - prev2FilteredData);

    // Store the filtered value in the output array
    filteredData[i] = static_cast<uint16_t>(filteredValue);

    // Update previous values for the next iteration
    prev2FilteredData = prevFilteredData;
    prevFilteredData = filteredValue;
  }
}

void getTCD1304Data(uint16_t SensorData[NUM_PIXELS]){
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
        applyLowPassFilter(CCDPixelBuffer, filteredData);
        gapWidth = calculateGapWidth(filteredData);
        if(gapWidth>0){
          Serial.print("Estimated Gap Width: ");
          Serial.print(gapWidth);
          Serial.println(" µm");
        } ;break;
      }
    };break;
  }

}
