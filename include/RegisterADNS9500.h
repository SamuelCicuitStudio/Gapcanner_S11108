
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SPI.h>
#include<firmwareADNS9500.h>



//Pin definition ADNS-9500
extern const uint8_t Movement_Pin;
extern const uint8_t CS_Pin;

// Registers
#define REG_Product_ID                           0x00
#define REG_Revision_ID                          0x01
#define REG_Motion                               0x02
#define REG_Delta_X_L                            0x03
#define REG_Delta_X_H                            0x04
#define REG_Delta_Y_L                            0x05
#define REG_Delta_Y_H                            0x06
#define REG_SQUAL                                0x07
#define REG_Pixel_Sum                            0x08
#define REG_Maximum_Pixel                        0x09
#define REG_Minimum_Pixel                        0x0a
#define REG_Shutter_Lower                        0x0b
#define REG_Shutter_Upper                        0x0c
#define REG_Frame_Period_Lower                   0x0d
#define REG_Frame_Period_Upper                   0x0e
#define REG_Configuration_I                      0x0f
#define REG_Configuration_II                     0x10
#define REG_Frame_Capture                        0x12
#define REG_SROM_Enable                          0x13
#define REG_Run_Downshift                        0x14
#define REG_Rest1_Rate                           0x15
#define REG_Rest1_Downshift                      0x16
#define REG_Rest2_Rate                           0x17
#define REG_Rest2_Downshift                      0x18
#define REG_Rest3_Rate                           0x19
#define REG_Frame_Period_Max_Bound_Lower         0x1a
#define REG_Frame_Period_Max_Bound_Upper         0x1b
#define REG_Frame_Period_Min_Bound_Lower         0x1c
#define REG_Frame_Period_Min_Bound_Upper         0x1d
#define REG_Shutter_Max_Bound_Lower              0x1e
#define REG_Shutter_Max_Bound_Upper              0x1f
#define REG_LASER_CTRL0                          0x20
#define REG_Observation                          0x24
#define REG_Data_Out_Lower                       0x25
#define REG_Data_Out_Upper                       0x26
#define REG_SROM_ID                              0x2a
#define REG_Lift_Detection_Thr                   0x2e
#define REG_Configuration_V                      0x2f
#define REG_Configuration_IV                     0x39
#define REG_Power_Up_Reset                       0x3a
#define REG_Shutdown                             0x3b
#define REG_Inverse_Product_ID                   0x3f
#define REG_Motion_Burst                         0x50
#define REG_SROM_Load_Burst                      0x62
#define REG_Pixel_Burst                          0x64


byte initComplete = 0;                          // Initialize initComplete variable to 0
byte testctr = 0;                               // Initialize testctr variable to 0
unsigned long currTime;                         // Declare currTime variable (not initialized here)
unsigned long timer;                            // Declare timer variable (not initialized here)
volatile int xydat[2];                          // Declare an array xydat with two elements
volatile bool movementflag = false;             // Declare movementflag variable as false
long int tdistance = 0;                         // Initialize tdistance variable to 0
extern const unsigned short firmware_length;    // Declare an external constant firmware_length
extern prog_uchar firmware_data[];              // Declare an external array firmware_data

//Prototypes
void initializeADNS9500();
void adns_com_begin();
void adns_com_end();
byte adns_read_reg(byte reg_addr);
void adns_write_reg(byte reg_addr, byte data);
void adns_upload_firmware();
void performStartup(void);
void UpdatePointer(void);
void dispRegisters(void);
int convTwosComp(int b);
int getDistance();


void initializeADNS9500() {
  pinMode(CS_Pin, OUTPUT);            // Set CS_Pin as an output pin
  
  pinMode(Movement_Pin,INPUT_PULLUP);
  attachInterrupt(Movement_Pin, UpdatePointer, FALLING);  // Attach an interrupt to Movement_Pin with UpdatePointer as the interrupt handler
  SPI.begin();                        // Initialize SPI communication
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));  // Configure SPI settings with a clock speed of 2 MHz, MSB first, and SPI mode 3
  performStartup();                   // Perform startup operations (presumably some sensor-specific initialization)
  dispRegisters();                    // Display registers (assuming it's a debugging or diagnostic function)
  delay(100);                         // Delay for 100 milliseconds
  initComplete = 9;                   // Set initComplete to 9
}

void adns_com_begin(){
  digitalWrite(CS_Pin, LOW);
}

void adns_com_end(){
  digitalWrite(CS_Pin, HIGH);
}

byte adns_read_reg(byte reg_addr){
  adns_com_begin();  // Begin a communication sequence
  SPI.transfer(reg_addr & 0x7f );  // Send address of the register with MSBit = 0 to indicate it's a read
  delayMicroseconds(100);  // tSRAD
  byte data = SPI.transfer(0);  // Read data
  delayMicroseconds(1);  // tSCLK-NCS for read operation is 120ns
  adns_com_end();  // End communication sequence
  delayMicroseconds(19);  // tSRW/tSRR (=20us) minus tSCLK-NCS
  return data;  // Return the read data
}

void adns_write_reg(byte reg_addr, byte data) {
  adns_com_begin();  // Begin a communication sequence
  // Send address of the register with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80);
  // Send data
  SPI.transfer(data);
  delayMicroseconds(20);  // tSCLK-NCS for write operation
  adns_com_end();  // End communication sequence
  delayMicroseconds(100);  // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but it looks like a safe lower bound
}

void adns_upload_firmware(){
  // send the firmware to the chip, cf p.18 of the datasheet
  Serial.println("Uploading firmware...");
  // set the configuration_IV register in 3k firmware mode
  adns_write_reg(REG_Configuration_IV, 0x02); // bit 1 = 1 for 3k mode, other bits are reserved 
  
  // write 0x1d in SROM_enable reg for initializing
  adns_write_reg(REG_SROM_Enable, 0x1d); 
  
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps... even if it should never be that low
  
  // write 0x18 to SROM_enable to start SROM download
  adns_write_reg(REG_SROM_Enable, 0x18); 
  
  // write the SROM file (=firmware data) 
  adns_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  
  // send all bytes of the firmware
  unsigned char c;
  for(int i = 0; i < firmware_length; i++){ 
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns_com_end();
  }

void performStartup(void){
  adns_com_end(); // ensure that the serial port is reset
  adns_com_begin(); // ensure that the serial port is reset
  adns_com_end(); // ensure that the serial port is reset
  adns_write_reg(REG_Power_Up_Reset, 0x5a); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  adns_read_reg(REG_Motion);
  adns_read_reg(REG_Delta_X_L);
  adns_read_reg(REG_Delta_X_H);
  adns_read_reg(REG_Delta_Y_L);
  adns_read_reg(REG_Delta_Y_H);
  // upload the firmware
  adns_upload_firmware();
  delay(10);
  //enable laser(bit 0 = 0b), in normal mode (bits 3,2,1 = 000b)
  // reading the actual value of the register is important because the real
  // default value is different from what is said in the datasheet, and if you
  // change the reserved bytes (like by writing 0x00...) it would not work.
  byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0);
  adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0 );
  
  delay(1);

  Serial.println("Optical Chip Initialized");
  }

void UpdatePointer(void){
  if(initComplete==9){

    digitalWrite(CS_Pin,LOW);
    xydat[0] = (int)adns_read_reg(REG_Delta_X_L);
    xydat[1] = (int)adns_read_reg(REG_Delta_Y_L);
    digitalWrite(CS_Pin,HIGH);     

    movementflag=true;
    }
  }

void dispRegisters(void){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x02  };
  char* oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","Motion"  };
  byte regres;

  digitalWrite(CS_Pin,LOW);
  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    regres = SPI.transfer(0);
    delay(1);
  }
  digitalWrite(CS_Pin,HIGH);
}

int convTwosComp(int b){
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
  }
   
  int getDistance() {
  while(movementflag){
    tdistance = tdistance + convTwosComp(xydat[0]);
    movementflag=0;
    delay(3);
    }
    return tdistance;
  }
  
