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

#define MotionPin 2
#define SENSOR_MISO 12
#define SENSOR_MOSI 11
#define SENSOR_SCK  13
#define SENSOR_NCS  10

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


void setup() {
  Serial.begin(9600);
  delay(500);
  pinMode(SENSOR_NCS, OUTPUT);
  pinMode(MotionPin, INPUT_PULLUP);
  delay(500);
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(4);
  SPI.begin();
  
  performStartup();
  dispRegisters();
  delay(500);
  initComplete = 9;
}

void loop() {
  if (!digitalRead(MotionPin)) {
    Serial.println("Motion detected!");
  }
  if (movementflag) {
    tdistance = tdistance + convTwosComp(xydat[0]);
    Serial.println("Distance = " + String(tdistance));
    movementflag = 0;
    delay(3);
  }
}


void adns_com_begin() {
  digitalWrite(SENSOR_NCS, LOW);
}

void adns_com_end() {
  digitalWrite(SENSOR_NCS, HIGH);
}

byte adns_read_reg(byte reg_addr) {
  adns_com_begin();
  SPI.transfer(reg_addr & 0x7f);
  delayMicroseconds(100);
  byte data = SPI.transfer(0);
  delayMicroseconds(1);
  adns_com_end();
  delayMicroseconds(19);
  return data;
}

void adns_write_reg(byte reg_addr, byte data) {
  adns_com_begin();
  SPI.transfer(reg_addr | 0x80);
  SPI.transfer(data);
  delayMicroseconds(20);
  adns_com_end();
  delayMicroseconds(100);
}

void adns_upload_firmware() {
  Serial.println("Uploading firmware...");
  adns_write_reg(REG_Configuration_IV, 0x02);
  adns_write_reg(REG_SROM_Enable, 0x1d);
  delay(10);
  adns_write_reg(REG_SROM_Enable, 0x18);
  adns_com_begin();
  SPI.transfer(REG_SROM_Load_Burst | 0x80);
  delayMicroseconds(15);
  unsigned char c;
  for (int i = 0; i < firmware_length; i++) {
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  adns_com_end();
}

void performStartup() {
  adns_com_end();
  adns_com_begin();
  adns_com_end();
  adns_write_reg(REG_Power_Up_Reset, 0x5a);
  delay(50);
  adns_read_reg(REG_Motion);
  adns_read_reg(REG_Delta_X_L);
  adns_read_reg(REG_Delta_X_H);
  adns_read_reg(REG_Delta_Y_L);
  adns_read_reg(REG_Delta_Y_H);
  adns_upload_firmware();
  delay(10);
  byte laser_ctrl0 = adns_read_reg(REG_LASER_CTRL0);
  adns_write_reg(REG_LASER_CTRL0, laser_ctrl0 & 0xf0);
  delay(1);
  Serial.println("Optical Chip Initialized");
}

void UpdatePointer(void) {
  if (initComplete == 9) {
    digitalWrite(SENSOR_NCS, LOW);
    xydat[0] = (int)adns_read_reg(REG_Delta_X_L);
    xydat[1] = (int)adns_read_reg(REG_Delta_Y_L);
    digitalWrite(SENSOR_NCS, HIGH);
    movementflag = 1;
  }
}

void dispRegisters(void) {
  int oreg[7] = {
    0x00, 0x3F, 0x2A, 0x02
  };
  char* oregname[] = {
    "Product_ID", "Inverse_Product_ID", "SROM_Version", "Motion"
  };
  byte regres;
  digitalWrite(SENSOR_NCS, LOW);
  int rctr = 0;
  for (rctr = 0; rctr < 4; rctr++) {
    SPI.transfer(oreg[rctr]);
    delay(1);
    Serial.println("---");
    Serial.println(oregname[rctr]);
    Serial.println(oreg[rctr], HEX);
    regres = SPI.transfer(0);
    Serial.println(regres, BIN);
    Serial.println(regres, HEX);
    delay(1);
  }
  digitalWrite(SENSOR_NCS, HIGH);
}

int convTwosComp(int b) {
  if (b & 0x80) {
    b = -1 * ((b ^ 0xff) + 1);
  }
  return b;
}

int tdistance = 0;
