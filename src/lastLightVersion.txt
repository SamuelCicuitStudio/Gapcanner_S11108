#include <Arduino.h>

// S11108 Pin  physical Connexion
#define CLOCK_PIN       5                                 // Clock Signal PWM Pin (FlexPWM1.1_Channel_1)
#define TRIGG_PIN       9                                 // Trigger Signal Pin
#define ST_PIN          29                                // ST Signal PWM Pin (FlexPWM3.1_Channel_2)
#define EOS_PIN         4                                 // EOS Signal Pin
#define DATA_PIN        A0                                // Sensor Analog Output Pin
#define BSW_PIN         GND                               // Connected to GND to set 2048 pixel reading

#define SAVE_PIN        28                                // Save Signal PWM Pin (FlexPWM3.1_Channel_1),
#define TRIGG_SAVE_PIN  11                                // Data Storage Trigger interrupt pin linked to SAVE_PIN physically, used as a interrupt pin

// Constants Related to S11108
#define NUM_PIXELS              2048                      // Number of pixels in the sensor
#define PIXEL_SIZE              14.0                      // Pixel size in micrometers (14 μm)

uint16_t sensorData[NUM_PIXELS];                          // Sensor Data Array

// Volatile variables
volatile uint16_t pixelCount = 0;                         // Count of pixels
volatile bool SaveFlag = false;                           // Save flag
volatile uint16_t gapWidth = 0;                              // Gap width

uint16_t minVal = 1024; // Initialize minVal with the maximum possible value
uint16_t maxVal = 0;          // Initialize maxVal with 0
#define thresholdPercentage  30; // For example, 10% of the range
#define alpha  0.7;  // Adjust this value as needed
uint16_t filteredData[NUM_PIXELS];

// Function Prototypes
uint16_t calculateGapWidth(uint16_t sensorData[]);        // Function to estimate the gap width in µm
void showData(uint16_t sensorData[]);                     // Function to show stored sensor data
void configureAndStartTimers();                           // Configure and start timers for Clock, ST, and SAVE pins
void configureInterrupts();                               // Configure TRIGG_PIN, TRIGG_SAVE_PIN, and EOS_PIN interrupts
void getS11108Data();



void setup() {
  Serial.begin(115200);                                                     // Initialize serial communication
  while (!Serial && millis() < 5000);                                       // Wait for the serial port to open (for debugging)
  delay(500);                                                               // Delay for stability
  //Serial.println("Configuring PWM for Clock and ST Signals");          // Print a message
  configureAndStartTimers();                                                // Configure and start timers for Clock, ST, and SAVE pins
  configureInterrupts();                                                    // Configure TRIGG_PIN, TRIGG_SAVE_PIN, and EOS_PIN interrupts
}

void loop() {
 getS11108Data();

}

// Function to calculate the gap width between plastic flat strips
uint16_t calculateGapWidth(uint16_t SensorData[]) {
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

    // Calculate the thresholds based on a percentage of the range
    

    int highThreshold = maxVal - (maxVal - minVal) * thresholdPercentage / 100;
    int lowThreshold = minVal + (maxVal - minVal) * thresholdPercentage / 100;

    // Initialize variables for gap width calculation
    int gapWidth = 0;
    bool insideGap = false;
    int middlePixel = NUM_PIXELS / 2; // Middle pixel index

    // Keep track of the gaps and their widths
    int largestGapWidth = 0;
    int largestGapStart = 0;
    int currentGapStart = 0;

    // Minimum and maximum gap widths to consider as valid gaps
    int minValidGapWidth = 71; // Minimum gap width in pixels
    int maxValidGapWidth = 428; // Maximum gap width in pixels

    // Filter out gaps smaller than the minimum gap width
    for (int i = 0; i < NUM_PIXELS; i++) {
        if (SensorData[i] < lowThreshold) {
            gapWidth++; // Increment the gap width counter
        } else if (gapWidth < minValidGapWidth) {
            // Replace values within smaller gaps with max or min values
            if (SensorData[i] > highThreshold) {
                // Replace with maximum value for positive gap
                SensorData[i] = maxVal;
            } else {
                // Replace with minimum value for negative gap
                SensorData[i] = minVal;
            }
        } else {
            gapWidth = 0; // Reset gap width if a valid gap is encountered
        }
    }

    // Iterate through SensorData to find edges and calculate gaps
    for (int i = 0; i < NUM_PIXELS; i++) {
        if (SensorData[i] > highThreshold && !insideGap) {
            // Rising edge detected (beginning of a strip or gap)
            insideGap = true;
            currentGapStart = i;
        } else if (SensorData[i] < lowThreshold && insideGap) {
            // Falling edge detected (beginning of the gap)
            gapWidth++; // Increment the gap width counter
        } else if (SensorData[i] > highThreshold && insideGap) {
            // Second rising edge detected (end of a strip or gap)
            insideGap = false;
            if (gapWidth > largestGapWidth && gapWidth >= minValidGapWidth && gapWidth <= maxValidGapWidth) {
                largestGapWidth = gapWidth;
                largestGapStart = currentGapStart;
            }
            gapWidth = 0; // Reset gap width
        }
    }

    // Calculate the middle gap length in micrometers (µm)
    int pixelSize = 14; // 14 µm per pixel
    int middleGapLength = 0;

    if (largestGapWidth > 0) {
        middleGapLength = largestGapWidth * pixelSize;
    }

    return middleGapLength; // Return the middle gap length in µm
}

void showData(uint16_t sensorData[]) {
  for (int i = 0; i < 2049; i++) {
    Serial.print("[");
    Serial.print(sensorData[i]);
    Serial.print("] ");

    // Print a new line after every 100 values
    if ((i + 1) % 100 == 0) {
      Serial.println();
    }
  }
}

void configureAndStartTimers() {
  //Serial.println("Configuring timers...");  // Print a message
  // Configure Clock pin
  pinMode(CLOCK_PIN, OUTPUT);
  analogWriteFrequency(CLOCK_PIN, 375000);  // Set the PWM frequency to 375 KHz on Pin Clock
  analogWriteResolution(10);  // Set the analog write resolution to 10 bits (1024 levels)

  // Configure ST pin
  pinMode(ST_PIN, OUTPUT);
  analogWriteFrequency(ST_PIN, 175);  // Set the PWM frequency to 175Hz on Pin ST
  analogWriteResolution(10);  // Set the analog write resolution to 10 bits (1024 levels)

  // Configure SAVE pin
  pinMode(SAVE_PIN, OUTPUT);
  analogWriteFrequency(SAVE_PIN, 175);  // Set the PWM frequency to 175Hz on Pin SAVE
  analogWriteResolution(10);  // Set the analog write resolution to 10 bits (1024 levels)
  //Serial.println("Starting timers ST - CLOCK -SAVE");  // Print a message
  
  // Start Timers
  analogWrite(ST_PIN, 999);  // Set a duty cycle of 95.7% on Pin ST
  analogWrite(CLOCK_PIN, 512);  // Set a duty cycle of 50% on Pin Clock
  analogWrite(SAVE_PIN, 999);  // Set a duty cycle of 99.81% on Pin SAVE
}

void configureInterrupts() {
  //Serial.println("Interrupt pin Configuration....");  // Print a message
  // Configure TRIGG_PIN Pin
  pinMode(TRIGG_PIN, INPUT_PULLUP);  // Configure TRIGG_PIN as INPUT with an internal pull-up resistor
 
  // Attach Save Pulse Interrupt
  pinMode(TRIGG_SAVE_PIN, INPUT);  // Configure TRIGG_SAVE_PIN as INPUT
  
  // Attach EOS Pulse Interrupt
  pinMode(EOS_PIN, INPUT);  // Configure EOS_PIN as INPUT
  }

void getS11108Data(){
  pixelCount =0;
 while(!digitalRead(TRIGG_SAVE_PIN)){
  while(digitalRead(TRIGG_PIN)){
 ///Serial.print("video =");
 //Serial.println(analogRead(DATA_PIN));
 pixelCount++;
 if (pixelCount > 87 && pixelCount <= NUM_PIXELS + 87) {
    sensorData[pixelCount - 88] = analogRead(DATA_PIN);
    //Serial.println(sensorData[pixelCount - 88]);
    
  };
   if (pixelCount > NUM_PIXELS + 87)break;
   };break;
}
 //showData(sensorData);
applyLowPassFilter(sensorData, filteredData, alpha);
//showData(filteredData);
 gapWidth = calculateGapWidth(filteredData);
 Serial.print("Estimated Gap Width: ");
 Serial.print(gapWidth);
 Serial.println(" µm");
 delay(100);
}

void applyLowPassFilter(uint16_t sensorData[], uint16_t filteredData[], float alpha) {
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


