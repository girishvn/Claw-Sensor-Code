/*
 * Authors : Girish Narayanswamy (girish.narayanswamy@colorado.edu)
 * 
 * Desc    : A simple scrip to read values from the claw sensor strip,
 *           implementing ToF and IR proximity sensors.
 *           Based on the code produced by Sparkfun for their Infrared Proximity 
 *           and Time of Flight Sensors
 *           https://www.sparkfun.com/products/10901 (IR Prox)
 *           https://www.sparkfun.com/products/12784 (ToF)
 */

#include <Wire.h>
//#include <SparkFun_VL6180X.h>

//DEVICE ADDRESSES
#define VL6180X_ADDRESS 0x29 //ToF Address
#define VCNL4010_ADDRESS 0x13 //IR Prox Address
int i2c_ids_[] = {112};// MUX Address 112

//IR Prox Globals and User Parameters 
#define COMMAND_0 0x80  // starts measurements, relays data ready info
#define PRODUCT_ID 0x81  // product ID/revision ID, should read 0x21
#define IR_CURRENT 0x83  // sets IR current in steps of 10mA 0-200mA
#define AMBIENT_PARAMETER 0x84  // Configures ambient light measures
#define AMBIENT_RESULT_MSB 0x85  // high byte of ambient light measure
#define AMBIENT_RESULT_LSB 0x86  // low byte of ambient light measure
#define PROXIMITY_RESULT_MSB 0x87  // High byte of proximity measure
#define PROXIMITY_RESULT_LSB 0x88  // low byte of proximity measure
#define PROXIMITY_MOD 0x8F  // proximity modulator timing

int ir_current_ = 4; // range = [0, 20]. current = value * 10 mA //4 - 18 checks ++2
int ambient_light_measurement_rate_ = 7; // range = [0, 7]. 1, 2, 3, 4, 5, 6, 8, 10 samples per second
int ambient_light_auto_offset_ = 1; // on or off
int averaging_function_ = 7;  // range [0, 7] measurements per run are 2**value, with range [1, 2**7 = 128]
int proximity_freq_ = 1; // range = [0 , 3]. 390.625kHz, 781.250kHz, 1.5625MHz, 3.125MHz 

//ToF Prox Globals and User Parameters 
#define VL6180x_FAILURE_RESET  -1

#define VL6180X_IDENTIFICATION_MODEL_ID              0x0000
#define VL6180X_IDENTIFICATION_MODEL_REV_MAJOR       0x0001
#define VL6180X_IDENTIFICATION_MODEL_REV_MINOR       0x0002
#define VL6180X_IDENTIFICATION_MODULE_REV_MAJOR      0x0003
#define VL6180X_IDENTIFICATION_MODULE_REV_MINOR      0x0004
#define VL6180X_IDENTIFICATION_DATE                  0x0006 //16bit value
#define VL6180X_IDENTIFICATION_TIME                  0x0008 //16bit value

#define VL6180X_SYSTEM_MODE_GPIO0                    0x0010
#define VL6180X_SYSTEM_MODE_GPIO1                    0x0011
#define VL6180X_SYSTEM_HISTORY_CTRL                  0x0012
#define VL6180X_SYSTEM_INTERRUPT_CONFIG_GPIO         0x0014
#define VL6180X_SYSTEM_INTERRUPT_CLEAR               0x0015
#define VL6180X_SYSTEM_FRESH_OUT_OF_RESET            0x0016
#define VL6180X_SYSTEM_GROUPED_PARAMETER_HOLD        0x0017

#define VL6180X_SYSRANGE_START                       0x0018
#define VL6180X_SYSRANGE_THRESH_HIGH                 0x0019
#define VL6180X_SYSRANGE_THRESH_LOW                  0x001A
#define VL6180X_SYSRANGE_INTERMEASUREMENT_PERIOD     0x001B
#define VL6180X_SYSRANGE_MAX_CONVERGENCE_TIME        0x001C
#define VL6180X_SYSRANGE_CROSSTALK_COMPENSATION_RATE 0x001E
#define VL6180X_SYSRANGE_CROSSTALK_VALID_HEIGHT      0x0021
#define VL6180X_SYSRANGE_EARLY_CONVERGENCE_ESTIMATE  0x0022
#define VL6180X_SYSRANGE_PART_TO_PART_RANGE_OFFSET   0x0024
#define VL6180X_SYSRANGE_RANGE_IGNORE_VALID_HEIGHT   0x0025
#define VL6180X_SYSRANGE_RANGE_IGNORE_THRESHOLD      0x0026
#define VL6180X_SYSRANGE_MAX_AMBIENT_LEVEL_MULT      0x002C
#define VL6180X_SYSRANGE_RANGE_CHECK_ENABLES         0x002D
#define VL6180X_SYSRANGE_VHV_RECALIBRATE             0x002E
#define VL6180X_SYSRANGE_VHV_REPEAT_RATE             0x0031

#define VL6180X_SYSALS_START                         0x0038
#define VL6180X_SYSALS_THRESH_HIGH                   0x003A
#define VL6180X_SYSALS_THRESH_LOW                    0x003C
#define VL6180X_SYSALS_INTERMEASUREMENT_PERIOD       0x003E
#define VL6180X_SYSALS_ANALOGUE_GAIN                 0x003F
#define VL6180X_SYSALS_INTEGRATION_PERIOD            0x0040

#define VL6180X_RESULT_RANGE_STATUS                  0x004D
#define VL6180X_RESULT_ALS_STATUS                    0x004E
#define VL6180X_RESULT_INTERRUPT_STATUS_GPIO         0x004F
#define VL6180X_RESULT_ALS_VAL                       0x0050
#define VL6180X_RESULT_HISTORY_BUFFER                0x0052 
#define VL6180X_RESULT_RANGE_VAL                     0x0062
#define VL6180X_RESULT_RANGE_RAW                     0x0064
#define VL6180X_RESULT_RANGE_RETURN_RATE             0x0066
#define VL6180X_RESULT_RANGE_REFERENCE_RATE          0x0068
#define VL6180X_RESULT_RANGE_RETURN_SIGNAL_COUNT     0x006C
#define VL6180X_RESULT_RANGE_REFERENCE_SIGNAL_COUNT  0x0070
#define VL6180X_RESULT_RANGE_RETURN_AMB_COUNT        0x0074
#define VL6180X_RESULT_RANGE_REFERENCE_AMB_COUNT     0x0078
#define VL6180X_RESULT_RANGE_RETURN_CONV_TIME        0x007C
#define VL6180X_RESULT_RANGE_REFERENCE_CONV_TIME     0x0080

#define VL6180X_READOUT_AVERAGING_SAMPLE_PERIOD      0x010A
#define VL6180X_FIRMWARE_BOOTUP                      0x0119
#define VL6180X_FIRMWARE_RESULT_SCALER               0x0120
#define VL6180X_I2C_SLAVE_DEVICE_ADDRESS             0x0212
#define VL6180X_INTERLEAVED_MODE_ENABLE              0x02A3

enum vl6180x_als_gain { //Data sheet shows gain values as binary list

GAIN_20 = 0, // Actual ALS Gain of 20
GAIN_10,     // Actual ALS Gain of 10.32
GAIN_5,      // Actual ALS Gain of 5.21
GAIN_2_5,    // Actual ALS Gain of 2.60
GAIN_1_67,   // Actual ALS Gain of 1.72
GAIN_1_25,   // Actual ALS Gain of 1.28
GAIN_1 ,     // Actual ALS Gain of 1.01
GAIN_40,     // Actual ALS Gain of 40

};

struct VL6180xIdentification
{
  uint8_t idModel;
  uint8_t idModelRevMajor;
  uint8_t idModelRevMinor;
  uint8_t idModuleRevMajor;
  uint8_t idModuleRevMinor;
  uint16_t idDate;
  uint16_t idTime;
};

//OTHER GLOBALS
#define NUM_SENSORS 4 //2 ToF and 2 IR-Prox

/***** GLOBAL VARIABLES *****/
int num_devices_;
unsigned int ambient_value_;
unsigned int proximity_value_;
volatile byte serialByte = 's';


void setup() {

  Serial.begin(115200); //Start Serial at 115200bps
  Wire.begin(); //Start I2C library
  delay(100); // delay .1s

  // get number of i2c devices specified by user
  num_devices_ = sizeof(i2c_ids_) / sizeof(int);
  Serial.print("Attached i2c devices: ");
  Serial.println(num_devices_);

  // initialize attached devices
  for (int i = 0; i < num_devices_; i++)
  {
    Serial.print("Initializing Sensor Strip: ");
    Serial.println(i2c_ids_[i]);

    initIRSensors(i2c_ids_[i]); //init IR Prox
    //initTOFSensors(i2c_ids_[i]); //init ToF
  }
  
  delay(1000); // delay 1s

} 

void loop() {

  for (int i = 0; i < num_devices_; i++)
  {
    readIRSensorStripValues(i2c_ids_[i]);
    //readTOFSensorStripValues(i2c_ids_[i]);
    delay(0);
  }
  
  delay(50);
}

/***********************************************************************************************

IR SENSOR FUNCTIONS 

***********************************************************************************************/

void initIRSensors(int id)
{
  Wire.beginTransmission(id);
  Wire.write(0);
  Serial.println("WIRE IN");
  int errcode = Wire.endTransmission();
  Serial.println(errcode); 

  // initialize each of the 2 sensors 
  for (int i = 0; i < NUM_SENSORS/2; i++)
  {

    // specify IR sensor
    Wire.beginTransmission(id);
    Wire.write(1 << i);
    Wire.endTransmission();
    
    //Feel free to play with any of these values, but check the datasheet first!
    IRwriteByte(AMBIENT_PARAMETER, 0x7F,VCNL4010_ADDRESS);
    IRwriteByte(IR_CURRENT, ir_current_,VCNL4010_ADDRESS);
    IRwriteByte(PROXIMITY_MOD, 1,VCNL4010_ADDRESS); // 1 recommended by Vishay
    
    delay(50); 

    byte temp = IRreadByte(PRODUCT_ID,VCNL4010_ADDRESS);
    byte proximityregister = IRreadByte(IR_CURRENT,VCNL4010_ADDRESS);
    //Serial.println(proximityregister);
    
    if (temp != 0x21){  // Product ID Should be 0x21 for the 4010 sensor
      Serial.print("IR sensor failed to initialize: id = ");
      Serial.print(i);
      Serial.print(". ");
      Serial.println(temp, HEX);
    }
    else
    {
      Serial.print("IR sensor online: id = ");
      Serial.println(i);
    }
  }
  
  Wire.beginTransmission(id);
  Wire.write(0);
  Wire.endTransmission();

}

void IRwriteByte(byte address, byte data, uint8_t deviceAddr)
{
  Wire.beginTransmission(deviceAddr);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

byte IRreadByte(byte address,uint8_t deviceAddr){
  Wire.beginTransmission(VCNL4010_ADDRESS);
  Wire.write(address);
  
  Wire.endTransmission();
  Wire.requestFrom(VCNL4010_ADDRESS, 1);
  unsigned long timeBefore = millis();
  while(!Wire.available()){
    if((millis()-timeBefore)>20){
      return 0;
    }
  }
  byte data = Wire.read();
  return data;
}

void readIRSensorStripValues(int id)
{
  char buf[8];
  // read all 8 sensors on the strip
  for (int i = 0; i < NUM_SENSORS/2; i++)
  {
    Wire.beginTransmission(id);
    Wire.write(1 << i);
    Wire.endTransmission();
    
    //ambient_value_ = readIRAmbient();
    proximity_value_ = readIRProximity();

    sprintf(buf, "%6u", proximity_value_);
    delay(5);
    Serial.print(buf);
  }
  Serial.println();
  Wire.beginTransmission(id);
  Wire.write(0);
  Wire.endTransmission();
}

unsigned int readIRProximity(){   
  byte temp = IRreadByte(0x80,VCNL4010_ADDRESS);
  IRwriteByte(0x80, temp | 0x08,VCNL4010_ADDRESS);  // command the sensor to perform a proximity measure
  unsigned long startTime = millis();
  while(!(IRreadByte(0x80,VCNL4010_ADDRESS) & 0x20)){  // Wait for the proximity data ready bit to be set
   if((millis()-startTime)>20){
     return 0;
   }
  }
  unsigned int data = IRreadByte(0x87,VCNL4010_ADDRESS) << 8;
  data |= IRreadByte(0x88,VCNL4010_ADDRESS);

  return data;
}

unsigned int readIRAmbient(){   
  byte temp = IRreadByte(0x80,VCNL4010_ADDRESS);
  IRwriteByte(0x80, temp | 0x10,VCNL4010_ADDRESS);  // command the sensor to perform ambient measure

  unsigned long startTime = millis();
  while(!(IRreadByte(0x80,VCNL4010_ADDRESS) & 0x40,VCNL4010_ADDRESS)){  // Wait for the proximity data ready bit to be set
   if((millis()-startTime)>20){
     return 0;
   }
  }
  unsigned int data = IRreadByte(0x85,VCNL4010_ADDRESS) << 8;
  data |= IRreadByte(0x86,VCNL4010_ADDRESS);

  return data;
}

/***********************************************************************************************

ToF SENSOR FUNCTIONS 

***********************************************************************************************/

uint8_t VL6180x_getRegister(uint16_t registerAddr)
{
  //uint8_t data;

  //Wire.beginTransmission(VL6180X_ADDRESS); // Address set on class instantiation
  //Wire.write((registerAddr >> 8) & 0xFF); //MSB of register address
  //Wire.write(registerAddr & 0xFF); //LSB of register address
  //Wire.endTransmission(false); //Send address and register address bytes
  //Wire.requestFrom(VL6180X_ADDRESS , 1);
  //data = Wire.read(); //Read Data from selected register

  //return data;

  Wire.beginTransmission(VL6180X_ADDRESS);
  Wire.write(registerAddr);
  
  Wire.endTransmission();
  Wire.requestFrom(VCNL4010_ADDRESS, 1);
  unsigned long timeBefore = millis();
  while(!Wire.available()){
    if((millis()-timeBefore)>20){
      return 0;
    }
  }
  uint8_t data = Wire.read();
  return data;
}

void VL6180x_setRegister(uint16_t registerAddr, uint8_t data)
{
  Wire.beginTransmission(VL6180X_ADDRESS); // Address set on class instantiation
  Wire.write((registerAddr >> 8) & 0xFF); //MSB of register address
  Wire.write(registerAddr & 0xFF); //LSB of register address
  Wire.write(data); // Data/setting to be sent to device.
  Wire.endTransmission(); //Send address and register address bytes
}

uint8_t VL6180xInit(uint8_t id) {
  
  uint8_t data; //for temp data storage
  data = VL6180x_getRegister(VL6180X_SYSTEM_FRESH_OUT_OF_RESET);

  if(data != 1) return VL6180x_FAILURE_RESET;

  //Required by datasheet
  //http://www.st.com/st-web-ui/static/active/en/resource/technical/document/application_note/DM00122600.pdf
  VL6180x_setRegister(0x0207, 0x01);
  VL6180x_setRegister(0x0208, 0x01);
  VL6180x_setRegister(0x0096, 0x00);
  VL6180x_setRegister(0x0097, 0xfd);
  VL6180x_setRegister(0x00e3, 0x00);
  VL6180x_setRegister(0x00e4, 0x04);
  VL6180x_setRegister(0x00e5, 0x02);
  VL6180x_setRegister(0x00e6, 0x01);
  VL6180x_setRegister(0x00e7, 0x03);
  VL6180x_setRegister(0x00f5, 0x02);
  VL6180x_setRegister(0x00d9, 0x05);
  VL6180x_setRegister(0x00db, 0xce);
  VL6180x_setRegister(0x00dc, 0x03);
  VL6180x_setRegister(0x00dd, 0xf8);
  VL6180x_setRegister(0x009f, 0x00);
  VL6180x_setRegister(0x00a3, 0x3c);
  VL6180x_setRegister(0x00b7, 0x00);
  VL6180x_setRegister(0x00bb, 0x3c);
  VL6180x_setRegister(0x00b2, 0x09);
  VL6180x_setRegister(0x00ca, 0x09);  
  VL6180x_setRegister(0x0198, 0x01);
  VL6180x_setRegister(0x01b0, 0x17);
  VL6180x_setRegister(0x01ad, 0x00);
  VL6180x_setRegister(0x00ff, 0x05);
  VL6180x_setRegister(0x0100, 0x05);
  VL6180x_setRegister(0x0199, 0x05);
  VL6180x_setRegister(0x01a6, 0x1b);
  VL6180x_setRegister(0x01ac, 0x3e);
  VL6180x_setRegister(0x01a7, 0x1f);
  VL6180x_setRegister(0x0030, 0x00);

  return 0;

}

void initTOFSensors(int id)
{
  Wire.beginTransmission(id);
  Wire.write(0);
  Serial.println("WIRE IN");
  int errcode = Wire.endTransmission();
  Serial.println(errcode); 

  // initialize each of the 4 sensors 
  for (int i = 2; i < NUM_SENSORS; i++)
  {

    // specify ToF sensor
    Wire.beginTransmission(id);
    Wire.write(1 << i);
    Wire.endTransmission();
    
    byte temp = VL6180xInit(VL6180X_ADDRESS); //init each tof sensor
    
    delay(50); 
    
    if (temp != 0x00){  // Product ID Should be 0x00 for the ToF sensor
      Serial.print("ToF sensor failed to initialize: id = ");
      Serial.print(i);
      Serial.print(": ");
      Serial.println(temp, HEX);
    }
    else
    {
      Serial.print("ToF sensor online: id = ");
      Serial.println(i);
    }
  }
  
  Wire.beginTransmission(id);
  Wire.write(0);
  Wire.endTransmission();

}

