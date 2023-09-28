

#include <Arduino.h>
#include <AccelStepper.h>
#include <cmath>



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
float integralError = 0 ;                                 // Initialize the integral error

// Constants related to Stepper
#define DEGREES_PER_STEP        360.0 / STEPS_PER_REVOLUTION // Degrees per step for the stepper motor
AccelStepper stepper(1, StepperStep, StepperDir);         // Create an AccelStepper object for stepper motor control

// Constants Related to S11108
#define NUM_PIXELS              2048                      // Number of pixels in the sensor
#define PIXEL_SIZE              14.0                      // Pixel size in micrometers (14 μm)
#define integrationTime         0.010                     // Integration time in seconds (10 ms)


#define RaspiCommandHandeler    serialEvent               // SerialEvent
#define SerialRaspiCommand      Serial4                   // serial port for commands

#define thresholdPercentage     30                        // For example, 10% of the range histeresis 
#define alpha                  0.7                        // Adjust this value as needed for the low pass filter

uint16_t sensorData[NUM_PIXELS];                          // Sensor Data Array
uint16_t minVal = 1024; // Initialize minVal with the maximum possible value
uint16_t maxVal = 0;          // Initialize maxVal with 0
uint16_t filteredData[NUM_PIXELS];

// Volatile variables
uint16_t pixelCount = 0;                         // Count of pixels
uint64_t gapWidth = 0;                              // Gap width
#define DefaultSampling 200
uint8_t sampling = DefaultSampling;

// Setting up ADNS9500 for measurement
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


//Enable Variables default states
volatile bool STEPPER_CORRECTION_STATE = false;
volatile bool CMOS_WIDTH_MEASUREMENT_STATE = false;
volatile bool OPTICAL_LENGTH_MEASUREMENT_STATE = false;
volatile uint16_t STEPS_PER_REVOLUTION = 200;             // Number of steps per revolution for the stepper motor
volatile uint16_t SET_TARGET_GAP_WIDTH_ = 10;
volatile uint16_t SET_GAP_FORWARD_TOLERANCE_ = 10;
volatile uint16_t SET_GAP_BACKWARDS_TOLERANCE_ = 10;
volatile uint16_t MANUAL_STEP_BACKWARDS_ = 1;
volatile uint16_t MANUAL_STEP_FORWARD_ = 1;

// Function Prototypes
void STEPPER_CORRECTION(float measuredGap);               // Function to control the stepper motor
uint16_t calculateGapWidth(uint16_t sensorData[]);        // Function to estimate the gap width in µm
void CalibrationADNS9500(char axe);                       // Function to calibrate the mouse for measurement
int32_t ADNS9500XData();                                  // Function to get mouse X data
int32_t ADNS9500YData();                                  // Function to get mouse Y data
float measureLengthX(int32_t xADNS9500Data, float xCalibrationFactor); // Function to calculate X distance in mm
float measureLengthY(int32_t yADNS9500Data, float yCalibrationFactor); // Function to calculate Y distance in mm
void blinkLED(int count);                                 // Function to blink LED
void showData(uint16_t sensorData[]);                     // Function to show stored sensor data
void initializeADNS9500();                                // Initialize USB Mouse & Calibration related pins and USB Host
void initializeStepper();                                 // Initialize the stepper motor parameters
void configureAndStartTimers();                           // Configure and start timers for Clock, ST, and SAVE pins
void configureInterrupts();                               // Configure TRIGG_PIN, TRIGG_SAVE_PIN, and EOS_PIN interrupts
void getS11108Data();                                     // function to get, filter and calculata the gap width
void applyLowPassFilter(uint16_t sensorData[], uint16_t filteredData[]); // Function to apply a 2 order lowpass to data
//Custom Command
int parseValueFromCommand(String command);
void executeCommand(String command, int parsedValue);
void RaspiCommandHandeler();


void ENABLE_STEPPER_CORRECTION(){STEPPER_CORRECTION_STATE = true;}          //Enables the automatic stepper correction for when the CMOS Sensor detects a gap that is outside of the allowed tolerances.
void DISABLE_STEPPER_CORRECTION() {STEPPER_CORRECTION_STATE = false;}       //Disables the automatic stepper correction and enables manual override of the stepper systems.
void ENABLE_CMOS_WIDTH_MEASUREMENT(){CMOS_WIDTH_MEASUREMENT_STATE = true;}  //Enables reading of CMOS gap measurement. Needs to be called at least once before Enabling stepper Correction.
void DISABLE_CMOS_WIDTH_MEASUREMENT(){CMOS_WIDTH_MEASUREMENT_STATE = false;}//Disables reading of CMOS gap measurement. Can be called at any time but disables the stepper correction functionality.
void ENABLE_OPTICAL_LENGTH_MEASUREMENT(){OPTICAL_LENGTH_MEASUREMENT_STATE = true;}//Enables reading of the ADNS-9500 length measurements. Can be called at any time.
void DISABLE_OPTICAL_LENGTH_MEASUREMENT(){OPTICAL_LENGTH_MEASUREMENT_STATE = false;}//Disables reading of the ADNS-9500 length measurements. Can be called at any time.
int  SET_MAX_STEPS(uint16_t x){STEPS_PER_REVOLUTION = x;return 1;}          //Sets the maximum number of forward steps that can be performed by the motor from it's zero position.
int  MANUAL_STEP_FORWARD(uint16_t x){MANUAL_STEP_FORWARD_ = x;return 1;}    //Perform 1 forward step with the stepper motor, or perfrom %arg1% number of forwards steps.
int  MANUAL_STEP_BACKWARDS(uint16_t x){MANUAL_STEP_BACKWARDS_ = x;return 1;}//Perform 1 backwards step with the stepper motor, or perfrom %arg1% number of backwards steps.
int  SET_TARGET_GAP_WIDTH(uint16_t x){SET_TARGET_GAP_WIDTH_ = x;return 1;}  //Sets the required width of the gap between the pieces of tape as %arg1% mm. 
int  SET_GAP_FORWARD_TOLERANCE(uint16_t x) {SET_GAP_FORWARD_TOLERANCE_ = x;return 1;}//Sets the positive tolerance for the gap. If MEASURED_GAP - TARGET_GAP > FORWARD_TOLERANCE this means the gap is too large and means the stepper needs to make backwards adjustments.
int  SET_GAP_BACKWARDS_TOLERANCE(uint16_t x) {SET_GAP_BACKWARDS_TOLERANCE_ = x;return 1;}//Sets the negative tolerance for the gap. If TARGET_GAP - MEASURED_GAP > BACKWARD_TOLERANCE this means the gap is too small and means the stepper needs to make forwards adjustments.
int  GET_LENGTH_MEASURED(){measureLengthX(ADNS9500XData(), xCalibrationFactor);return 1;}//Prompts the teensy board to send the length of tape that has been measured over the ADNS-9500 in number of CM.
int  GET_WIDTH_MEASURED(){calculateGapWidth(sensorData);return 1;}          //Prompts the teensy board to send the distance of the width which has been measured over the S11108 CMOS Sensor in um.


void setup() {
  Serial.begin(115200);                                                     // Initialize serial communication
  while (!Serial && millis() < 5000);                                       // Wait for the serial port to open (for debugging)
  delay(500);                                                               // Delay for stability
  Serial.println(F("\nConfiguring PWM for Clock and ST Signals"));          // Print a message
  SerialRaspiCommand.begin(9600);
  initializeADNS9500();                                                     // Initialize USB Mouse & Calibration related pins and USB Host
  initializeStepper();                                                      // Initialize the stepper motor parameters
  configureAndStartTimers();                                                // Configure and start timers for Clock, ST, and SAVE pins
  configureInterrupts();                                                    // Configure TRIGG_PIN, TRIGG_SAVE_PIN, and EOS_PIN interrupts
}

void loop() {
getS11108Data();  
}

 
// function to controll stepper motor feedback loop
void STEPPER_CORRECTION(float measuredGap) {
  if (STEPPER_CORRECTION_STATE) {
    float error = SET_TARGET_GAP_WIDTH_ - measuredGap; // Calculate error using SET_TARGET_GAP_WIDTH
    float proportionalControl = KP * error; // Calculate the proportional control component
    integralError += KI * error; // Accumulate the integral error component
    if (integralError > MAX_INTEGRAL) integralError = MAX_INTEGRAL; // Limit the integral component to prevent wind-up
    else if (integralError < -MAX_INTEGRAL) integralError = -MAX_INTEGRAL;

    // Adjust the following lines to use the tolerance macros
    if (measuredGap - SET_TARGET_GAP_WIDTH_ > SET_GAP_FORWARD_TOLERANCE_)
      stepper.move(-(proportionalControl + integralError)); // Gap is too large, stepper needs to make backward adjustments
    else if (SET_TARGET_GAP_WIDTH_ - measuredGap > SET_GAP_BACKWARDS_TOLERANCE_)
      stepper.move(proportionalControl + integralError);    // Gap is too small, stepper needs to make forward adjustments
    else
      stepper.move(proportionalControl + integralError);

    while (stepper.isRunning()) stepper.run();              // Wait until the stepper motor has finished moving
  }
}

//Function to step the stepper in both directions
void STEPingStepper(int value) {
  if (!STEPPER_CORRECTION_STATE) {
  stepper.move(value);    
  }
}

// Function to calculate the gap width between plastic flat strips
// Function to calculate the gap width
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



    int highThreshold = maxVal - (maxVal - minVal) * thresholdPercentage / 100;
    int lowThreshold = minVal + (maxVal - minVal) * thresholdPercentage / 100;

    // Initialize variables for gap width calculation

    bool firstRisingEdgeDetected = false; // Flag to track the first rising edge
    int firstRisingEdgeIndex = 0; // Index of the first rising edge
     int secondRisingEdgeIndex = 0; // Index of the first rising edge
    int16_t startingPoint = 0;
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
    int pixelSize = 14; // 14 µm per pixel
    int gapWidthMicrometers = (lowPixelsLeft + lowPixelsRight) * pixelSize;

    return gapWidthMicrometers; // Return the gap width in µm
}

void CalibrationADNS9500(char axe) {

}

int32_t ADNS9500XData() {

}

int32_t ADNS9500YData() {

}

float measureLengthX(int32_t xMouseData, float xCalibrationFactor) {
  if(OPTICAL_LENGTH_MEASUREMENT_STATE)
  {return xMouseData * xCalibrationFactor;}
  else {return 0;}
}

float measureLengthY(int32_t yMouseData, float yCalibrationFactor) {
  if(OPTICAL_LENGTH_MEASUREMENT_STATE)
  {return yMouseData * yCalibrationFactor;}
  else {return 0;}
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

void initializeADNS9500() {
  Serial.println("initializing ADNS9500...");  // Print a message
  Serial.println("initializing ADNS9500 done!!!!!");  // Print a message
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
  analogWriteResolution(10);  // Set the analog write resolution to 10 bits (1024 levels)

  // Configure SAVE pin
  pinMode(SAVE_PIN, OUTPUT);
  analogWriteFrequency(SAVE_PIN, 175);  // Set the PWM frequency to 175Hz on Pin SAVE
  analogWriteResolution(10);  // Set the analog write resolution to 10 bits (1024 levels)
  Serial.println("Starting timers ST - CLOCK -SAVE");  // Print a message
  
  // Start Timers
  analogWrite(ST_PIN, 999);  // Set a duty cycle of 95.7% on Pin ST
  analogWrite(CLOCK_PIN, 512);  // Set a duty cycle of 50% on Pin Clock
  analogWrite(SAVE_PIN, 999);  // Set a duty cycle of 99.81% on Pin SAVE
}

void configureInterrupts() {
  Serial.println("Interrupt pin Configuration....");  // Print a message
  // Configure TRIGG_PIN Pin
  pinMode(TRIGG_PIN, INPUT_PULLUP);  // Configure TRIGG_PIN as INPUT with an internal pull-up resistor
  
  // Attach Save Pulse Interrupt
  pinMode(TRIGG_SAVE_PIN, INPUT);  // Configure TRIGG_SAVE_PIN as INPUT
  
  // Attach EOS Pulse Interrupt
  pinMode(EOS_PIN, INPUT);  // Configure EOS_PIN as INPUT
  }
// Function to parse the value from a command
int parseValueFromCommand(String command) {
  int parsedValue = 0; // Default value if parsing fails

  // List of available commands and corresponding functions
    struct CommandFunction {
    const char* commandName;
    int (*function)(uint16_t x);
  };

  // Define the list of commands and functions
  CommandFunction commandFunctions[] = {
    {"SET_MAX_STEPS", SET_MAX_STEPS},
    {"MANUAL_STEP_FORWARD", MANUAL_STEP_FORWARD},
    {"MANUAL_STEP_BACKWARDS", MANUAL_STEP_BACKWARDS},
    {"SET_TARGET_GAP_WIDTH", SET_TARGET_GAP_WIDTH},
    {"SET_GAP_FORWARD_TOLERANCE", SET_GAP_FORWARD_TOLERANCE},
    {"SET_GAP_BACKWARDS_TOLERANCE", SET_GAP_BACKWARDS_TOLERANCE},
    
  };

  // Iterate through the commands and check if the input command matches
  for (const auto& cmdFunc : commandFunctions) {
    if (command.startsWith(cmdFunc.commandName)) {
      // Check if the command contains '()' and extract the value
      int openParenthesisIndex = command.indexOf('(');
      int closeParenthesisIndex = command.indexOf(')');
      if (openParenthesisIndex != -1 && closeParenthesisIndex != -1) {
        // Extract the substring between '()' and convert it to an integer
        String valueString = command.substring(openParenthesisIndex + 1, closeParenthesisIndex);
        parsedValue = valueString.toInt();
      }
    }
  }

  return parsedValue;
}

void executeCommand(String command, int parsedValue) {
  //Serial.println(parsedValue);
  if (command.startsWith("ENABLE_STEPPER_CORRECTION")) {
    ENABLE_STEPPER_CORRECTION();
    Serial.println("Stepper Correction Enabled");
  } else if (command.startsWith("DISABLE_STEPPER_CORRECTION")) {
    DISABLE_STEPPER_CORRECTION();
    Serial.println("Stepper Correction Disabled");
  } else if (command.startsWith("ENABLE_CMOS_WIDTH_MEASUREMENT")) {
    ENABLE_CMOS_WIDTH_MEASUREMENT();
    Serial.println("CMOS Width Measurement Enabled");
  } else if (command.startsWith("DISABLE_CMOS_WIDTH_MEASUREMENT")) {
    DISABLE_CMOS_WIDTH_MEASUREMENT();
    Serial.println("CMOS Width Measurement Disabled");
  } 
  else if (command.startsWith("ENABLE_OPTICAL_LENGTH_MEASUREMENT")) {
    ENABLE_OPTICAL_LENGTH_MEASUREMENT();
    Serial.println("Optical Length Meeasureemnt Enabled");
  }
  else if (command.startsWith("DISABLE_OPTICAL_LENGTH_MEASUREMENT")) {
    ENABLE_OPTICAL_LENGTH_MEASUREMENT();
    Serial.println("Optical Length Meeasureemnt Disabled");
  }
  else if (command.startsWith("SET_MAX_STEPS")) {
    SET_MAX_STEPS(parsedValue);
    Serial.print("SET_MAX_STEPS(");
    Serial.print(parsedValue);
    Serial.println(")");
  }
  else if (command.startsWith("MANUAL_STEP_FORWARD")) {
    STEPingStepper(parsedValue);
    Serial.print("MANUAL_STEP_FORWARD(");
    Serial.print(parsedValue);
    Serial.println(")");
  }
  else if (command.startsWith("MANUAL_STEP_BACKWARDS")) {
    STEPingStepper(-parsedValue);
    Serial.print("MANUAL_STEP_BACKWARDS(");
    Serial.print(parsedValue);
    Serial.println(")");
  }
  else if (command.startsWith("SET_TARGET_GAP_WIDTH")) {
    SET_TARGET_GAP_WIDTH(parsedValue);
    Serial.print("SET_TARGET_GAP_WIDTH(");
    Serial.print(parsedValue);
    Serial.println(")");
  }
  else if (command.startsWith("SET_GAP_FORWARD_TOLERANCE")) {
    SET_GAP_FORWARD_TOLERANCE(parsedValue);
    Serial.print("SET_GAP_FORWARD_TOLERANCE(");
    Serial.print(parsedValue);
    Serial.println(")");
  }
  else if (command.startsWith("SET_GAP_BACKWARDS_TOLERANCE")) {
    SET_GAP_BACKWARDS_TOLERANCE(parsedValue);
    Serial.print("SET_GAP_BACKWARDS_TOLERANCE(");
    Serial.print(parsedValue);
    Serial.println(")");
  }
  else if (command.startsWith("GET_LENGTH_MEASURED")) {
    GET_LENGTH_MEASURED();
    SerialRaspiCommand.print(GET_LENGTH_MEASURED());
    Serial.print("LENGTH_MEASUREDH_MEASURED =");
    Serial.print(GET_LENGTH_MEASURED());
    Serial.println("Cm");
  }
  else if (command.startsWith("GET_WIDTH_MEASURED")) {
    GET_WIDTH_MEASURED();
    SerialRaspiCommand.print(GET_WIDTH_MEASURED());
    Serial.print("WIDTH_MEASURED =");
    Serial.print(GET_WIDTH_MEASURED());
    Serial.println("µm");
  }// Add more cases for other commands here
  else {
    Serial.println("Command not recognized");
  }
}

void RaspiCommandHandeler() {
  while (SerialRaspiCommand.available() > 0) {
    String command = SerialRaspiCommand.readStringUntil('\n');    // Read the command from Serial input
    int parsedValue = parseValueFromCommand(command);// Parse and execute the command
    executeCommand(command, parsedValue);
  }
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
applyLowPassFilter(sensorData, filteredData);
 gapWidth += calculateGapWidth(filteredData);
 sampling--;
if(sampling == 0){//showData(filteredData);
 Serial.print("Estimated Gap Width: ");
 Serial.print((float)(gapWidth/DefaultSampling));
 Serial.println(" µm");
 gapWidth = 0;
 sampling = DefaultSampling;
 delay(100);}
}

void applyLowPassFilter(uint16_t sensorData[], uint16_t filteredData[]) {
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

