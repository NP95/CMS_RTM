#define VERBOSE1  1 // for debug

// Code for I2C interface.
#include <Wire.h>

// The red LED is controlled by pin 13.
#define RED_LED 13

//Retimer Address
#define Retimer_WriteAddress_0 0x18
#define Retimer_WriteAddress_1 0x1B
#define Retimer_ReadAddress_0 0x18
#define Retimer_ReadAddress_1 0x1B

//Register Addresses
#define Broadcast_Enable 0xFF
#define Channel_Registers 0x00
#define Preset_Signal_Detect 0x14
#define Override 0x09
#define VCO_Cap_Count 0x08
#define Divider_Select 0x18
#define Charge_Pump_Control 0x1B
#define Loop_filter_DAC 0x1F
#define PRBS_enable  0x1E
#define Clock_enable 0x30
#define Output_Mux_PRBS_Gen 0x1E
#define PRBS_Load 0x0D


//Data
#define Broadcast_Enable_Data 0x0C
#define Reset_Channel_Registers_Data 0x04
#define Preset_Signal_Detect_Data 0x80
#define Divider_Select_Override_Data 0x04
#define VCO_Cap_Select_Override_Data 0x80
#define VCO_Cap_Count_Data 0x06
#define Divider_Select_Data 0x00
#define Charge_Pump_Down_Override_Data 0x08
#define Charge_Pump_Down_Enable_Data   0x00
#define Loop_filter_DAC_Override_Data 0x40
#define Loop_filter_DAC_Override_Value_Data 0x12
#define PRBS_enable_Data  0x10
#define Clock_enable_Data 0x0A
#define Loopthr_select_override_Data 0x20
#define Output_Mux_PRBS_Gen_Data 0x80
#define PRBS_Load_Data 0x20

//Masks

#define Broadcast_Enable_Mask 0xFF
#define Reset_Channel_Registers_Mask 0x04
#define Preset_Signal_Detect_Mask 0x80
#define Divider_Select_Override_Mask 0x04
#define VCO_Cap_Select_Override_Mask 0x80
#define VCO_Cap_Count_Mask 0x1F
#define Divider_Select_Mask 0x70
#define Charge_Pump_Down_Override_Mask 0x08
#define Charge_Pump_Down_Enable_Mask   0x03
#define Loop_filter_DAC_Override_Mask 0x40
#define Loop_filter_DAC_Override_Value_Mask 0x1F
#define PRBS_enable_Mask  0x10
#define Clock_enable_Mask 0x0F
#define Loopthr_select_override_Mask 0x20
#define Output_Mux_PRBS_Gen_Mask 0xE0
#define PRBS_Load_Mask 0x20

void setRegister(byte i2c_addr, byte addr, byte data, byte mask)
{ 
  byte readval, writeval;

  Wire.beginTransmission(i2c_addr);
  Wire.write(addr);
  stat = Wire.endTransmission(); // stop transmitting

  if (stat != 0) {
    Serial.print("I2C status = ");
    Serial.println(stat); // https://www.arduino.cc/en/Reference/WireEndTransmission
    exit(0);
  }

  Wire.requestFrom(i2c_addr,1); // request 1 byte from slave device 24
  readval = Wire.read();    // receive a byte

  writeval = (data & mask) | (readval & (~mask));

  Wire.beginTransmission(i2c_addr);
  Wire.write(addr); // send byte
  Wire.write(writeval);  // send byte
  stat = Wire.endTransmission(); // stop transmitting

  if (stat != 0) {
    Serial.print("I2C status = ");
    Serial.println(stat); // https://www.arduino.cc/en/Reference/WireEndTransmission
    exit(0);
  }

  // double check

  Wire.beginTransmission(i2c_addr);
  Wire.write(addr); 
  stat = Wire.endTransmission(); // stop transmitting

  if (stat != 0) {
    Serial.print("I2C status = ");
    Serial.println(stat); // https://www.arduino.cc/en/Reference/WireEndTransmission
    exit(0);
  }

  Wire.requestFrom(i2c_addr,1); // request 1 byte from slave device 24
  readval = Wire.read();    // receive a byte

  if ( (readval&mask) != (data&mask) ){
    Serial.println("incorrect write!");
    exit(0);
  }

}

// void readRegister(byte retimer_address,byte address)
// {
// }

void PRBS31_Retimer_0()
{
setRegister(Retimer_WriteAddress_0,Broadcast_Enable,Broadcast_Enable_Data,Broadcast_Enable_Mask);
setRegister(Retimer_WriteAddress_0,Channel_Registers,Reset_Channel_Registers_Data,Reset_Channel_Registers_Mask);
setRegister(Retimer_WriteAddress_0,Preset_Signal_Detect,Preset_Signal_Detect_Data,Preset_Signal_Detect_Mask);
setRegister(Retimer_WriteAddress_0,Override,Divider_Select_Override_Data,Divider_Select_Override_Mask);
setRegister(Retimer_WriteAddress_0,Override,VCO_Cap_Select_Override_Data,VCO_Cap_Select_Override_Mask);
setRegister(Retimer_WriteAddress_0,VCO_Cap_Count,VCO_Cap_Count_Data,VCO_Cap_Count_Mask);
setRegister(Retimer_WriteAddress_0,Divider_Select,Divider_Select_Data,Divider_Select_Mask);
setRegister(Retimer_WriteAddress_0,Override,Charge_Pump_Down_Override_Data,Charge_Pump_Down_Override_Mask);
setRegister(Retimer_WriteAddress_0,Charge_Pump_Control,Charge_Pump_Down_Enable_Data,Charge_Pump_Down_Enable_Mask);
setRegister(Retimer_WriteAddress_0,Override,Loop_filter_DAC_Override_Data,Loop_filter_DAC_Override_Mask);
setRegister(Retimer_WriteAddress_0,Loop_filter_DAC,Loop_filter_DAC_Override_Value_Data,Loop_filter_DAC_Override_Value_Mask);
setRegister(Retimer_WriteAddress_0,PRBS_enable,PRBS_enable_Data,PRBS_enable_Mask);
setRegister(Retimer_WriteAddress_0,Clock_enable,Clock_enable_Data,Clock_enable_Mask);
setRegister(Retimer_WriteAddress_0,Override,Loopthr_select_override_Data,Loopthr_select_override_Mask);
setRegister(Retimer_WriteAddress_0,Output_Mux_PRBS_Gen,Output_Mux_PRBS_Gen_Data,Output_Mux_PRBS_Gen_Mask);
setRegister(Retimer_WriteAddress_0,PRBS_Load,PRBS_Load_Data,PRBS_Load_Mask);

}

void PRBS31_Retimer_1()
{
setRegister(Retimer_WriteAddress_1,Broadcast_Enable,Broadcast_Enable_Data,Broadcast_Enable_Mask);
setRegister(Retimer_WriteAddress_1,Channel_Registers,Reset_Channel_Registers_Data,Reset_Channel_Registers_Mask);
setRegister(Retimer_WriteAddress_1,Preset_Signal_Detect,Preset_Signal_Detect_Data,Preset_Signal_Detect_Mask);
setRegister(Retimer_WriteAddress_1,Override,Divider_Select_Override_Data,Divider_Select_Override_Mask);
setRegister(Retimer_WriteAddress_1,Override,VCO_Cap_Select_Override_Data,VCO_Cap_Select_Override_Mask);
setRegister(Retimer_WriteAddress_1,VCO_Cap_Count,VCO_Cap_Count_Data,VCO_Cap_Count_Mask);
setRegister(Retimer_WriteAddress_1,Divider_Select,Divider_Select_Data,Divider_Select_Mask);
setRegister(Retimer_WriteAddress_1,Override,Charge_Pump_Down_Override_Data,Charge_Pump_Down_Override_Mask);
setRegister(Retimer_WriteAddress_1,Charge_Pump_Control,Charge_Pump_Down_Enable_Data,Charge_Pump_Down_Enable_Mask);
setRegister(Retimer_WriteAddress_1,Override,Loop_filter_DAC_Override_Data,Loop_filter_DAC_Override_Mask);
setRegister(Retimer_WriteAddress_1,Loop_filter_DAC,Loop_filter_DAC_Override_Value_Data,Loop_filter_DAC_Override_Value_Mask);
setRegister(Retimer_WriteAddress_1,PRBS_enable,PRBS_enable_Data,PRBS_enable_Mask);
setRegister(Retimer_WriteAddress_1,Clock_enable,Clock_enable_Data,Clock_enable_Mask);
setRegister(Retimer_WriteAddress_1,Override,Loopthr_select_override_Data,Loopthr_select_override_Mask);
setRegister(Retimer_WriteAddress_1,Output_Mux_PRBS_Gen,Output_Mux_PRBS_Gen_Data,Output_Mux_PRBS_Gen_Mask);
setRegister(Retimer_WriteAddress_1,PRBS_Load,PRBS_Load_Data,PRBS_Load_Mask);

}

// void ReadSequence_1()
// {
//   readRegister(Retimer_ReadAddress_0,Broadcast_Enable);
//   readRegister(Retimer_ReadAddress_0,Channel_Registers);
//   readRegister(Retimer_ReadAddress_0,Preset_Signal_Detect);
//   readRegister(Retimer_ReadAddress_0,Override);
//   readRegister(Retimer_ReadAddress_0,Override);
//   readRegister(Retimer_ReadAddress_0,VCO_Cap_Count);
//   readRegister(Retimer_ReadAddress_0,Divider_Select);
//   readRegister(Retimer_ReadAddress_0,Override);
//   readRegister(Retimer_ReadAddress_0,Charge_Pump_Control);
//   readRegister(Retimer_ReadAddress_0,Override);
//   readRegister(Retimer_ReadAddress_0,Loop_filter_DAC);
//   readRegister(Retimer_ReadAddress_0,PRBS_enable);
//   readRegister(Retimer_ReadAddress_0,Clock_enable);
//   readRegister(Retimer_ReadAddress_0,Override);
//   readRegister(Retimer_ReadAddress_0,Output_Mux_PRBS_Gen);
//   readRegister(Retimer_ReadAddress_0,PRBS_Load);
// }

// void ReadSequence_2()
// {
//   readRegister(Retimer_ReadAddress_1,Broadcast_Enable);
//   readRegister(Retimer_ReadAddress_1,Channel_Registers);
//   readRegister(Retimer_ReadAddress_1,Preset_Signal_Detect);
//   readRegister(Retimer_ReadAddress_1,Override);
//   readRegister(Retimer_ReadAddress_1,Override);
//   readRegister(Retimer_ReadAddress_1,VCO_Cap_Count);
//   readRegister(Retimer_ReadAddress_1,Divider_Select);
//   readRegister(Retimer_ReadAddress_1,Override);
//   readRegister(Retimer_ReadAddress_1,Charge_Pump_Control);
//   readRegister(Retimer_ReadAddress_1,Override);
//   readRegister(Retimer_ReadAddress_1,Loop_filter_DAC);
//   readRegister(Retimer_ReadAddress_1,PRBS_enable);
//   readRegister(Retimer_ReadAddress_1,Clock_enable);
//   readRegister(Retimer_ReadAddress_1,Override);
//   readRegister(Retimer_ReadAddress_1,Output_Mux_PRBS_Gen);
//   readRegister(Retimer_ReadAddress_1,PRBS_Load);
// }


int pca9545a_write(byte i2c_addr, byte dat);

int stat;

// The setup function runs once when you press reset or power the board.
void setup() {

  // Initialize the RED_LED pin as an output.
  pinMode(RED_LED, OUTPUT);

  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);

  // Join i2c bus.
  Wire.begin();

  // Debug thru the IDE console.
  if (VERBOSE1 == 1) {
    while (!Serial);
    Serial.begin(9600); // start serial for output
    Serial.println("setup done.");
    Serial.println();
  }

}

// the loop function runs over and over again forever
void loop() {
  pca9545a_write(0x73,0x6);  // enable channel 3

  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);              // wait for 1/2 second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(500);              // wait for 1/2 second
  byte error, address; //variable for error and I2C address
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000); // wait 5 seconds for the next I2C scan

  PRBS31_Retimer_0();
  PRBS31_Retimer_1();


}

int pca9545a_write(byte i2c_addr, byte dat)
{
  Wire.beginTransmission(i2c_addr); // transmit to I2C device
  Wire.write(dat); // send byte
  stat = Wire.endTransmission(); // stop transmitting

  if (VERBOSE1 == 1) {
    Serial.print("I2C status = ");
    Serial.println(stat); // https://www.arduino.cc/en/Reference/WireEndTransmission
  }
  //if (stat == 0) {
  //Serial.print("I2C addr = ");
  //Serial.println(i2c_addr);
  //}

  Wire.requestFrom(i2c_addr,1); // request 1 byte from i2c device
  int c = Wire.read();    // receive a byte

  if (VERBOSE1 == 1) {
    Serial.print("c = ");
    Serial.println(c);
  }

  return(0);
}
