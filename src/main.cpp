/*
   S11108 Sensor Control Code
   Author: Samuel Tshibangu
   Description: This code interfaces with the S11108 CMOS linear image sensor
                to read and process data from the sensor. It controls the sensor's
                configuration, collects data, calculates gap width, and controls a
                stepper motor based on the measured gap.

   Pin Configuration:
   - StepperDir: Stepper motor direction pin
   - StepperStep: Stepper motor step pin
   - CLOCK_PIN: Clock signal PWM pin
   - TRIGG_PIN: Trigger signal pin
   - ST_PIN: ST signal PWM pin
   - SAVE_PIN: Save signal PWM pin
   - EOS_PIN: EOS (End of Scan) signal pin
   - DATA_PIN: Sensor's analog output pin
   - TRIGG_SAVE_PIN: Pin for generating data storage interrupt

   Constants:
   - PROPORTIONAL_GAIN: Proportional gain for control
   - TOLERANCE: Error tolerance for stepper control (mm)
   - NUM_PIXELS: Number of pixels in the sensor
   - PIXEL_SIZE: Pixel size in micrometers (14 μm)
   - THRESHOLD: Desired gap width threshold (mm)
   - STEPS_PER_REVOLUTION: Number of steps per revolution for the stepper motor
   - DEGREES_PER_STEP: Degrees per step for the stepper motor

   Functionality:
   1. Configures PWM signals for clock, ST, and save pins.
   2. Sets up a stepper motor for gap control.
   3. Collects data from the S11108 sensor.
   4. Calculates the gap width using optical triangulation.
   5. Controls the stepper motor based on the measured gap.


 Saving algorithm operates as follows:

1. It utilizes three interrupts:

   - **EOS (CHANGE):** Triggered when there is a change in the EOS signal.
   - **Trigg Save (FALLING edge):** Triggered on the falling edge of the Trigg Save signal.
   - **Trigg pin (RISING edge):** Triggered on the rising edge of the Trigg pin signal.

2. The process unfolds step by step:

   - **First Step:** Wait for the EOS signal to go low and set the `eosflag` to false.

   - **Second Step:** Wait for the Trigg Save signal to go low and set the `saveflag` to true.

3. After completing the two steps above, the following actions occur at each Trigg pin interrupt:

   - Check if `!eosflag && saveflag` is true. If this condition is met, save data.
   - Otherwise, perform calculations for specific measurements and exit the process.
*/

#include <Arduino.h>
#include <AccelStepper.h>
#include <cmath>
#include "USBHost_t36.h"

// USB Host Setup
USBHost myusb;
MouseController mouse(myusb);

// Stepper  Pin  physical Connexion
#define StepperDir      3                                 // Stepper Direction Pin
#define StepperStep     7                                 // Stepper Step Pin

// button & led Pin  physical Connexion
#define buttonPin       10                                // Actual button pin
#define ledPin          13                                // Built-in LED pin

// S11108 Pin  physical Connexion
#define CLOCK_PIN       5                                 // Clock Signal PWM Pin (FlexPWM1.1_Channel_1)
#define TRIGG_PIN       9                                 // Trigger Signal Pin
#define ST_PIN          29                                // ST Signal PWM Pin (FlexPWM3.1_Channel_2)
#define EOS_PIN         4                                 // EOS Signal Pin
#define DATA_PIN        A0                                // Sensor Analog Output Pin
#define BSW_PIN         GND                               // Connected to GND to set 2048 pixel reading

#define SAVE_PIN        28                                // Save Signal PWM Pin (FlexPWM3.1_Channel_1),
#define TRIGG_SAVE_PIN  11                                // Data Storage Trigger interrupt pin linked to SAVE_PIN physically, used as a interrupt pin

#define calibrationBlinkCount    3                        // Calibration blink count
#define calibrationBlinkDuration 300                      // Blink delay in milliseconds

// Constants for PI control stepper
#define KP                      0.5                       // Proportional gain
#define KI                      0.01                      // Integral gain
#define MAX_INTEGRAL            10                        // MAX_INTEGRAL value
float integralError = 0;                                  // Initialize the integral error

// Constants related to Stepper
#define TOLERANCE               1                         // Error tolerance for stepper control (mm)
#define THRESHOLD               10.0                      // Desired gap width threshold (mm)
#define STEPS_PER_REVOLUTION    200                       // Number of steps per revolution for the stepper motor
#define DEGREES_PER_STEP        360.0 / STEPS_PER_REVOLUTION // Degrees per step for the stepper motor
AccelStepper stepper(1, StepperStep, StepperDir);         // Create an AccelStepper object for stepper motor control

// Constants related to S11108
#define NUM_PIXELS              2048                      // Number of pixels in the sensor
#define PIXEL_SIZE              14.0                      // Pixel size in micrometers (14 μm)
#define DistanceToSensor        1                         // Actual distance from the sensor to the plastic flat strips in mm
#define photosensitivity        50.0                      // Sensor photosensitivity in V/(lx·s)
#define integrationTime         0.010                     // Integration time in seconds (10 ms)
#define photosensitiveAreaLength 28.672                   // Effective photosensitive area length in mm
uint16_t sensorData[NUM_PIXELS];                          // Sensor Data Array

// Volatile variables
volatile bool eosFlag = false;                            // End of Scan (EOS) flag
volatile uint16_t pixelCount = 0;                         // Count of pixels
volatile bool SaveFlag = false;                           // Save flag
volatile float gapWidth = 0;                              // Gap width

// Setting up mouse for measurement
unsigned long lastButtonPressTime = 0;
float xCalibrationFactor = 0.0;                           // Calibration factor for X-axis
float yCalibrationFactor = 0.0;                           // Calibration factor for Y-axis
volatile  float distanceX_mm = 0;                         // Distance in X-axis (in mm)
volatile  float distanceY_mm = 0;                         // Distance in Y-axis (in mm)
enum State {
  IDLE,
  CALIBRATION,
  EXIT,
};
State currentState = IDLE;

// Function Prototypes
void controlStepper(float measuredGap);                   // Function to control the stepper motor
float calculateGapWidth(uint16_t sensorData[], float distanceToSensor); // Function to estimate the gap width
void eosUpdate();                                         // Interrupt function executed on EOS Falling edge
void SaveUpdateInterrupt();                               // Interrupt function executed on TRIGG_SAVE Falling edge
void SavePulseInterrupt();                                // Interrupt function executed on TRIGG Rising edge
void CalibrationMouse(char axe);                          // Function to calibrate the mouse for measurement
int16_t mouseXData();                                     // Function to get mouse X data
int16_t mouseYData();                                     // Function to get mouse Y data
float measureLengthX(int16_t xMouseData, float xCalibrationFactor); // Function to calculate X distance in mm
float measureLengthY(int16_t yMouseData, float yCalibrationFactor); // Function to calculate Y distance in mm
void blinkLED(int count);                                 // Function to blink LED
void showData(uint16_t sensorData[]);                     // Function to show stored sensor data
void initializeMouseAndCalibration();                     // Initialize USB Mouse & Calibration related pins and USB Host
void initializeStepper();                                 // Initialize the stepper motor parameters
void configureAndStartTimers();                           // Configure and start timers for Clock, ST, and SAVE pins
void configureInterrupts();                               // Configure TRIGG_PIN, TRIGG_SAVE_PIN, and EOS_PIN interrupts



void setup() {
  Serial.begin(115200);  // Initialize serial communication
  while (!Serial && millis() < 5000);  // Wait for the serial port to open (for debugging)
  delay(500);  // Delay for stability
  Serial.println(F("\nConfiguring PWM for Clock and ST Signals"));  // Print a message

  initializeMouseAndCalibration();                        // Initialize USB Mouse & Calibration related pins and USB Host
  initializeStepper();                                    // Initialize the stepper motor parameters
  configureAndStartTimers();                              // Configure and start timers for Clock, ST, and SAVE pins
  configureInterrupts();                                  // Configure TRIGG_PIN, TRIGG_SAVE_PIN, and EOS_PIN interrupts
}

void loop() {

  // Calculate the gap width using optical triangulation
  Serial.print("Estimated Gap Width: ");
  Serial.print(gapWidth);
  Serial.println(" mm");

  // Control the stepper motor based on the measured gap
  controlStepper(gapWidth);

  // Control the stepper motor based on the measured gap
  Serial.print("X Distance (mm): ");
  Serial.println(distanceX_mm);
  Serial.print("Y Distance (mm): ");
  Serial.println(distanceY_mm);
  delay(1000);
}
 // EOS flag interrupt
void eosUpdate(){
  eosFlag = digitalRead(EOS_PIN);                           // update ESOflag
  if(eosFlag) {                                             // decide when to reset pixelCount and turn false saveflag
    pixelCount = 0;
    SaveFlag = false;}
}
// TRIGG SAVE flag interrupt
void SaveUpdateInterrupt(){
 SaveFlag = true;                                           // update Saveflag
}

 
// function to controll stepper motor feedback loop
void controlStepper(float measuredGap) {
  float error = THRESHOLD - measuredGap;                    // Calculate error
  float proportionalControl = KP * error;                   // Calculate the proportional control component
  integralError += KI * error;                              // Accumulate the integral error component
  if (integralError > MAX_INTEGRAL) integralError = MAX_INTEGRAL;  // Limit the integral component to prevent wind-up
  else if (integralError < -MAX_INTEGRAL) integralError = -MAX_INTEGRAL;
  stepper.move(proportionalControl + integralError);        // Calculate the total control signal (proportional + integral) and move the stepper motor
  while (stepper.isRunning()) stepper.run();                // Wait until the stepper motor has finished moving
}


// Function to calculate the gap width between plastic flat strips
float calculateGapWidth(uint16_t sensorData[], float distanceToSensor) {
    uint32_t sumPixelValues = 0;                            // Calculate the average pixel value within the gap region
    for (int i = 0; i < NUM_PIXELS; i++) {
      sumPixelValues += sensorData[i];
      }
    float averagePixelValue = static_cast<float>(sumPixelValues) / NUM_PIXELS;
    // Calculate illuminance in lux using sensor sensitivity and average pixel value
    float illuminance = (averagePixelValue * 1000000.0) / (photosensitivity * integrationTime); // Converting from μs to s
    // Calculate the gap width using the illuminance, pixel size, and photosensitive area length
    float gapWidth = (2.0 * sqrt(illuminance) * PIXEL_SIZE) / photosensitiveAreaLength;
    return gapWidth;
}

void SavePulseInterrupt() {
if(!eosFlag && SaveFlag){                                   // Check for EOS_Pin  goes LOW
    if(pixelCount <= NUM_PIXELS){                           // Read and store sensor data when a save pulse interrupt occurs
      sensorData[pixelCount] = analogRead(DATA_PIN);
      pixelCount++;
    }  
} else {
    distanceX_mm = measureLengthX(mouseXData(), xCalibrationFactor);
    distanceY_mm = measureLengthY(mouseYData(), yCalibrationFactor);
    gapWidth = calculateGapWidth(sensorData, DistanceToSensor);
    controlStepper(gapWidth);
}
}

void CalibrationMouse(char axe) {
  int16_t xMouseData;
  int16_t yMouseData;
    switch (currentState) {
    case IDLE:
      if (!digitalRead(buttonPin)) {
        // Button is pressed
        lastButtonPressTime = millis();
        delay(20); // Debounce delay
         while(true) {
          // Button is still pressed          
          if (digitalRead(buttonPin))lastButtonPressTime = millis();
          if (millis() - lastButtonPressTime >= 3000){
            blinkLED(calibrationBlinkCount);
            currentState = CALIBRATION;
            goto IdleEnd;
          }
        }
      }
      IdleEnd:
      break;
    
    case CALIBRATION:
    Serial.println("Calibration Mode.");
    Serial.println("Move straight 5cm");
    Serial.println("Press And hold to finish.");
    lastButtonPressTime = millis();
    while(true){
        if(axe == 'X') xMouseData = mouseXData();// Read and process mouse data for the chosen axis (X-axis)
        if(axe == 'Y') yMouseData = mouseYData();// Read and process mouse data for the chosen axis (y-axis)
        if (digitalRead(buttonPin))lastButtonPressTime = millis();
        if (millis() - lastButtonPressTime >= 3000) goto CalibrationEnd;         
      } 
      CalibrationEnd:
      blinkLED(calibrationBlinkCount);
      Serial.println("Calibration ended Successfully.");
      if(axe == 'X')xCalibrationFactor = 50.0 / xMouseData ; // Corresponds 5 cm to X-axis data
      if(axe == 'Y')yCalibrationFactor = 50.0 / yMouseData ; // Corresponds 5 cm to Y-axis data
      Serial.println("Storing Calibration Data.");
      break;

    case EXIT:
      currentState = EXIT;
      break;

  }
}

int16_t mouseXData() {
  int16_t accumulatedMouseX = 0;
  myusb.Task();
  if (mouse.available()) {
    int16_t deltaX = mouse.getMouseX();
    accumulatedMouseX += deltaX;
    return accumulatedMouseX;
  }
  return accumulatedMouseX; // Return the accumulated value
}

int16_t mouseYData() {
  int16_t accumulatedMouseY = 0;
  myusb.Task();
  if (mouse.available()) {
    int16_t deltaY = mouse.getMouseY();
    accumulatedMouseY += deltaY;
    return accumulatedMouseY;
  }
  return accumulatedMouseY; // Return the accumulated value
}

float measureLengthX(int16_t xMouseData, float xCalibrationFactor) {
  return xMouseData * xCalibrationFactor;
}

float measureLengthY(int16_t yMouseData, float yCalibrationFactor) {
  return yMouseData * yCalibrationFactor;
}

void blinkLED(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(ledPin, HIGH);
    delay(calibrationBlinkDuration);
    digitalWrite(ledPin, LOW);
    delay(calibrationBlinkDuration);
  }
}

void showData(uint16_t sensorData[]) {
  // Iterate through the sensorData array and print its elements
  for (int i = 0; i < 2049; i++) {
    Serial.println(sensorData[i]);
  }
}

void initializeMouseAndCalibration() {
  Serial.println("initializing Mouse...");  // Print a message
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);  // Turn off the LED initially
  myusb.begin();  // Begin USB Host
  Serial.println("initializing Mouse done!!!!!");  // Print a message
}

void initializeStepper() {
  Serial.println("initializing Stepper...");  // Print a message
  stepper.setMaxSpeed(1000.0);  // Set the maximum speed in steps per second
  stepper.setAcceleration(500.0);  // Set the acceleration in steps per second^2
  stepper.setCurrentPosition(0);  // Set the initial position to zero
  Serial.println("initializing Stepper donne!!!!");  // Print a message
}

void configureAndStartTimers() {
  Serial.println("Configuring timers...");  // Print a message
  // Configure Clock pin
  pinMode(CLOCK_PIN, OUTPUT);
  analogWriteFrequency(CLOCK_PIN, 375000);  // Set the PWM frequency to 375 KHz on Pin Clock
  analogWriteResolution(10);  // Set the analog write resolution to 10 bits (1024 levels)

  // Configure ST pin
  pinMode(ST_PIN, OUTPUT);
  analogWriteFrequency(ST_PIN, 175);  // Set the PWM frequency to 175Hz on Pin ST

  // Configure SAVE pin
  pinMode(SAVE_PIN, OUTPUT);
  analogWriteFrequency(SAVE_PIN, 175);  // Set the PWM frequency to 175Hz on Pin SAVE
  Serial.println("Starting timers ST - CLOCK -SAVE");  // Print a message
  // Start Timers
  analogWrite(ST_PIN, 999);  // Set a duty cycle of 95.7% on Pin ST
  analogWrite(CLOCK_PIN, 512);  // Set a duty cycle of 50% on Pin Clock
  analogWrite(SAVE_PIN, 1022);  // Set a duty cycle of 99.81% on Pin SAVE
}

void configureInterrupts() {
  Serial.println("Interrupt pin Configuration....");  // Print a message
  // Configure TRIGG_PIN Pin
  pinMode(TRIGG_PIN, INPUT_PULLUP);  // Configure TRIGG_PIN as INPUT with an internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(TRIGG_PIN), SavePulseInterrupt, RISING);  // Attach clock pulse interrupt for the rising edge

  // Attach Save Pulse Interrupt
  pinMode(TRIGG_SAVE_PIN, INPUT);  // Configure TRIGG_SAVE_PIN as INPUT
  attachInterrupt(digitalPinToInterrupt(TRIGG_SAVE_PIN), SaveUpdateInterrupt, FALLING);  // Attach clock pulse interrupt for the falling edge

  // Attach EOS Pulse Interrupt
  pinMode(EOS_PIN, INPUT);  // Configure EOS_PIN as INPUT
  attachInterrupt(digitalPinToInterrupt(EOS_PIN), eosUpdate, CHANGE);  // Attach clock pulse interrupt for any change
  Serial.println("nterrupt pin Configuration done!!!!!!!!");  // Print a message
}