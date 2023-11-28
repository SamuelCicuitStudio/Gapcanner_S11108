#include <SPI.h>
#include <avr/pgmspace.h>

// Registers
#define REG_Product_ID               0x00
#define REG_Revision_ID              0x01
#define REG_Motion                   0x02
#define REG_Delta_X_L                0x03
#define REG_Delta_X_H                0x04
#define REG_Delta_Y_L                0x05
#define REG_Delta_Y_H                0x06
#define REG_SQUAL                    0x07
#define REG_Pixel_Sum                0x08
#define REG_Maximum_Pixel            0x09
#define REG_Minimum_Pixel            0x0a
#define REG_Shutter_Lower            0x0b
#define REG_Shutter_Upper            0x0c
#define REG_Frame_Period_Lower       0x0d
#define REG_Frame_Period_Upper       0x0e
#define REG_Configuration_I          0x0f
#define REG_Configuration_II         0x10
#define REG_Frame_Capture            0x12
#define REG_SROM_Enable              0x13
#define REG_Run_Downshift            0x14
#define REG_Rest1_Rate               0x15
#define REG_Rest1_Downshift          0x16
#define REG_Rest2_Rate               0x17
#define REG_Rest2_Downshift          0x18
#define REG_Rest3_Rate               0x19
#define REG_Frame_Period_Max_Bound_Lower     0x1a
#define REG_Frame_Period_Max_Bound_Upper     0x1b
#define REG_Frame_Period_Min_Bound_Lower     0x1c
#define REG_Frame_Period_Min_Bound_Upper     0x1d
#define REG_Shutter_Max_Bound_Lower          0x1e
#define REG_Shutter_Max_Bound_Upper          0x1f
#define REG_LASER_CTRL0              0x20
#define REG_Observation              0x24
#define REG_Data_Out_Lower           0x25
#define REG_Data_Out_Upper           0x26
#define REG_SROM_ID                  0x2a
#define REG_Lift_Detection_Thr       0x2e
#define REG_Configuration_V          0x2f
#define REG_Configuration_IV         0x39
#define REG_Power_Up_Reset           0x3a
#define REG_Shutdown                 0x3b
#define REG_Inverse_Product_ID       0x3f
#define REG_Motion_Burst             0x50
#define REG_SROM_Load_Burst          0x62
#define REG_Pixel_Burst              0x64

byte initComplete = 0;
byte testctr = 0;
unsigned long currTime;
unsigned long timer;
volatile int xydat[2];
volatile byte movementflag = 0;
int tdistance = 0;

#define MotionPin 2
#define SENSOR_NCS  8
#define ADNS9500_CPI 90 // set the QDNS CPI from 90 to 5040 max
extern const unsigned short firmware_length;
extern const unsigned char firmware_data[];


void adns_com_begin();
void adns_com_end();
byte adns_read_reg(byte reg_addr);
void adns_write_reg(byte reg_addr, byte data);
void adns_upload_firmware();
void performStartup();
void UpdatePointer(void);
void dispRegisters(void);
int convTwosComp(int b);
void setAdnsResolution(int resolutionCPI);
void printConfigIValue();
void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(SENSOR_NCS, OUTPUT);
  pinMode(MotionPin, INPUT_PULLUP); 
  attachInterrupt(MotionPin, UpdatePointer, FALLING);
  delay(500);
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(8);
  
  performStartup();
  dispRegisters();
  delay(500);
  initComplete = 9;
}

void loop() {
  if (movementflag) {
    int mouseData = convTwosComp(xydat[0]); // Assuming xydat[0] contains the mouse data

    // Use the defined resolution value to calculate distance in inches
    float distanceInches = static_cast<float>(mouseData) / ADNS9500_CPI;

    Serial.print("Distance in inches: ");
    Serial.println(distanceInches, 4); // Adjust the precision as needed

    tdistance += mouseData; // Assuming you still want to accumulate the data
    movementflag = 0;
    delay(1);
  }
}



void adns_com_begin() {
  digitalWrite(SENSOR_NCS, LOW);
}
void adns_com_end() {
  digitalWrite(SENSOR_NCS, HIGH);
}

byte adns_read_reg(byte reg_addr){
  adns_com_begin();
  
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7f );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  adns_com_end();
  delayMicroseconds(19); //  tSRW/tSRR (=20us) minus tSCLK-NCS

  return data;
}


void adns_write_reg(byte reg_addr, byte data){
  adns_com_begin();
  
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  
  delayMicroseconds(20); // tSCLK-NCS for write operation
  adns_com_end();
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened, but is looks like a safe lower bound 
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
   printConfigIValue();
   delay(10);
  setAdnsResolution(ADNS9500_CPI);//set the ADNS9500 CPI
  delay(10);
   printConfigIValue();
  delay(10);
  Serial.println("Optical Chip Initialized");
  }

void UpdatePointer(void){
  if(initComplete==9){

    digitalWrite(SENSOR_NCS,LOW);
    xydat[0] = (int)adns_read_reg(REG_Delta_X_L);
    xydat[1] = (int)adns_read_reg(REG_Delta_Y_L);
    digitalWrite(SENSOR_NCS,HIGH);     

    movementflag=1;
    }
  }

void dispRegisters(void){
  int oreg[7] = {
    0x00,0x3F,0x2A,0x02  };
  char* oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","Motion"  };
  byte regres;

  digitalWrite(SENSOR_NCS,LOW);

  int rctr=0;
  for(rctr=0; rctr<4; rctr++){
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr],HEX);
    regres = SPI.transfer(0);
    Serial.println(regres,BIN);  
    Serial.println(regres,HEX);  
    delay(1);
  }
  digitalWrite(SENSOR_NCS,HIGH);
}


int convTwosComp(int b){
  //Convert from 2's complement
  if(b & 0x80){
    b = -1 * ((b ^ 0xff) + 1);
    }
  return b;
  }

// Function to set the resolution in Configuration_I register
void setAdnsResolution(int resolutionCPI) {
  // Ensure that the resolution value is within the valid range
  resolutionCPI = constrain(resolutionCPI, 90, 5000);

  // Calculate the corresponding value for Configuration_I register
  byte resolutionValue = resolutionCPI / 90;

  // Read the current value of Configuration_I register
  byte currentConfigI = adns_read_reg(REG_Configuration_I);

  // Clear the RES bits (5:0) in the current value
  currentConfigI &= 0xC0;

  // Set the RES bits with the calculated resolution value
  currentConfigI |= (resolutionValue & 0x3F);

  // Write the updated value back to Configuration_I register
  adns_write_reg(REG_Configuration_I, currentConfigI);
}
void printConfigIValue() {
  // Read the value of Configuration_I register
  byte configIValue = adns_read_reg(REG_Configuration_I);

  // Print the binary representation of the Configuration_I register value
  Serial.print("Configuration_I Register Value: ");
  for (int i = 7; i >= 0; i--) {
    Serial.print((configIValue >> i) & 0x01);
  }
  Serial.println();
}
