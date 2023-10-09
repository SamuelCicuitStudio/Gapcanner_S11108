
#include <Arduino.h>
#include <AccelStepper.h>
#include "RegisterADNS9500.h"
#include "MegunoLink.h"
#include "Filter.h"

// Exponential filters for data smoothing
ExponentialFilter<long> ADCFilter(10, 0);
ExponentialFilter<long> ADCFilter1(3, 0);

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
#define TRIGG_SAVE_PIN  24                                // Data Storage Trigger interrupt pin linked to SAVE_PIN physically, used as a interrupt pin

#define calibrationBlinkCount    3                        // Calibration blink count
#define calibrationBlinkDuration 300                      // Blink delay in milliseconds

//ADNS-9500 PINS
const uint8_t Movement_Pin = 25;   //Connected to Motion Pin of ADNS-9500
const uint8_t CS_Pin = 10;         //Connected to NCS pin ADNS-9500
#define SCK  13                    //Connected to SCLK pin of the ADNS-9500
#define MOSI 11                    //Connected to MOSI pin of the ADNS-9500
#define MISO 12                    //Connected to MISO pin of the ADNS-9500

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

// S11108 System parameters
#define thresholdPercentage     30    // Percentage of the range for hysteresis
#define steadyError             50    // Output error
bool IsLonger = false;              // Parameter to determine if the strip width is longer than the sensor width

// Variables
uint16_t pixelCount = 0;            // Count of pixels
uint16_t PrevgapWidth = 0;          // Previous gap width

// Filtered data array
uint16_t discretData[NUM_PIXELS];    // Array to store filtered data

// Sensor Data Array
uint16_t sensorData[NUM_PIXELS];     // Array to store sensor data


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
void STEPPER_CORRECTION(float measuredGap);          // Function to control the stepper motor
void STEPingStepper(int value);                      // Function to send steps to stepper
void blinkLED(int count);                            // Function to blink LED
void initializeStepper();                            // Initialize the stepper motor parameters
int ADNS9500Data();                                  // Function to fetch data from ADNS9500 sensor

// S11108 related functions
uint16_t calculateGapWidth(uint16_t sensorData[]);    // Function to calculate the gap width
uint16_t showMeasuredGap();                           // Function to display the measured gap
uint16_t GetGapWidth(uint16_t SensorData[]);          // Function to get and process the gap width data
void configureAndStartTimers();                       // Function to configure and start timers
void PinConfiguration();                              // Function to configure the pins


 
//Custom Command raspi communication
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
int  GET_LENGTH_MEASURED(){ADNS9500Data();return 1;}//Prompts the teensy board to send the length of tape that has been measured over the ADNS-9500 in number of CM.
int  GET_WIDTH_MEASURED(){calculateGapWidth(sensorData);return 1;}          //Prompts the teensy board to send the distance of the width which has been measured over the S11108 CMOS Sensor in um.


void setup() {
Serial.begin(460800);                                                       // Initialize serial communication at a baud rate of 460800 and open the serial port (for debugging)
while (!Serial && millis() < 5000);                                         // Wait for the serial port to open
delay(500);                                                                 // Add a 500ms delay for stability
Serial.println(F("\nConfiguring PWM for Clock and ST Signals"));            // Print a message to indicate PWM configuration
SerialRaspiCommand.begin(9600);                                             // Initialize communication with a Raspberry Pi (assuming SerialRaspiCommand is used)
initializeADNS9500();                                                       // Initialize USB Mouse & Calibration related pins and USB Host
configureAndStartTimers();                                                  // Configure and start timers for Clock, ST, and SAVE pins
PinConfiguration();                                                         // Configure TRIGG_PIN, TRIGG_SAVE_PIN, and EOS_PIN
}

void loop() {
showMeasuredGap(); 
}

/***********************************************************RASPI COMMUNICATION FUNCTIONS************************************************************************/
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
// Function to execute command from Raspi
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
// Function RaspiCommandHandeler
void RaspiCommandHandeler() {
  while (SerialRaspiCommand.available() > 0) {
    String command = SerialRaspiCommand.readStringUntil('\n');    // Read the command from Serial input
    int parsedValue = parseValueFromCommand(command);// Parse and execute the command
    executeCommand(command, parsedValue);
  }
}
/*******************************************************************STEPPER FUNCTIONS****************************************************************************/
// function to INITIALIZE stepper 
void initializeStepper() {
  Serial.println("initializing Stepper...");  // Print a message
  stepper.setMaxSpeed(1000.0);  // Set the maximum speed in steps per second
  stepper.setAcceleration(500.0);  // Set the acceleration in steps per second^2
  stepper.setCurrentPosition(0);  // Set the initial position to zero
  Serial.println("initializing Stepper donne!!!!");  // Print a message
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
/*******************************************************************S11108 FUNCTIONS****************************************************************************/
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
/********************************************************************ADNS-9500CTIONS****************************************************************************/
void blinkLED(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(ledPin, HIGH);
    delay(calibrationBlinkDuration);
    digitalWrite(ledPin, LOW);
    delay(calibrationBlinkDuration);
  }
}
int ADNS9500Data(){
  if(OPTICAL_LENGTH_MEASUREMENT_STATE) return getDistance();
  return 0;
}
