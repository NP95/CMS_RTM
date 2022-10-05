#include <Wire.h>
#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  // Required for Serial on Zero based boards
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif
//Retimer Address
//Swithc channels

#define channel1 0
#define channel2 1
#define channel3 2
#define channel4 3



#define Retimer_WriteAddress_0 0x18
#define Retimer_WriteAddress_1 0x1B
#define Retimer_ReadAddress_0 0x18
#define Retimer_ReadAddress_1 0x1B





#define bytesToRead 2

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

void setRegister(byte retimer_address,byte address,byte data,byte mask)
{  

  byte read1;
  byte read2;
  Wire.beginTransmission(retimer_address); // select device with "beginTransmission()"
  Wire.write(address); // select starting register with "write()"
  Wire.endTransmission(); // end write operation, as we just wanted to select the starting register
  Wire.requestFrom(retimer_address, 1); // select number of bytes to get from the device (2 bytes in this case)
  read1= Wire.read(); // read next byte from the following register
  Serial.print(address);
  Serial.print(":");
  Serial.println(read1);



      Wire.beginTransmission(retimer_address);   
      Wire.write(address);
      Wire.write( (data) );
      delay(100);
      Wire.write( (data) );  
      Wire.endTransmission( true ); 


  Wire.beginTransmission(retimer_address); // select device with "beginTransmission()"
  Wire.write(address); // select starting register with "write()"
  Wire.endTransmission(); // end write operation, as we just wanted to select the starting register
  Wire.requestFrom(retimer_address, 1); // select number of bytes to get from the device (2 bytes in this case)
  read1= Wire.read(); // read next byte from the following register
  Serial.print(address);
  Serial.print(":");
  Serial.println(read2);
    
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

void ReadSequence_1()
{
  readRegister(Retimer_ReadAddress_0,Broadcast_Enable);
  readRegister(Retimer_ReadAddress_0,Channel_Registers);
  readRegister(Retimer_ReadAddress_0,Preset_Signal_Detect);
  readRegister(Retimer_ReadAddress_0,Override);
  readRegister(Retimer_ReadAddress_0,Override);
  readRegister(Retimer_ReadAddress_0,VCO_Cap_Count);
  readRegister(Retimer_ReadAddress_0,Divider_Select);
  readRegister(Retimer_ReadAddress_0,Override);
  readRegister(Retimer_ReadAddress_0,Charge_Pump_Control);
  readRegister(Retimer_ReadAddress_0,Override);
  readRegister(Retimer_ReadAddress_0,Loop_filter_DAC);
  readRegister(Retimer_ReadAddress_0,PRBS_enable);
  readRegister(Retimer_ReadAddress_0,Clock_enable);
  readRegister(Retimer_ReadAddress_0,Override);
  readRegister(Retimer_ReadAddress_0,Output_Mux_PRBS_Gen);
  readRegister(Retimer_ReadAddress_0,PRBS_Load);
}

void ReadSequence_2()
{
  readRegister(Retimer_ReadAddress_1,Broadcast_Enable);
  readRegister(Retimer_ReadAddress_1,Channel_Registers);
  readRegister(Retimer_ReadAddress_1,Preset_Signal_Detect);
  readRegister(Retimer_ReadAddress_1,Override);
  readRegister(Retimer_ReadAddress_1,Override);
  readRegister(Retimer_ReadAddress_1,VCO_Cap_Count);
  readRegister(Retimer_ReadAddress_1,Divider_Select);
  readRegister(Retimer_ReadAddress_1,Override);
  readRegister(Retimer_ReadAddress_1,Charge_Pump_Control);
  readRegister(Retimer_ReadAddress_1,Override);
  readRegister(Retimer_ReadAddress_1,Loop_filter_DAC);
  readRegister(Retimer_ReadAddress_1,PRBS_enable);
  readRegister(Retimer_ReadAddress_1,Clock_enable);
  readRegister(Retimer_ReadAddress_1,Override);
  readRegister(Retimer_ReadAddress_1,Output_Mux_PRBS_Gen);
  readRegister(Retimer_ReadAddress_1,PRBS_Load);
}

// void ReadRegister(byte address, byte data)
// {

// }

void readSwitchRegisters()
{

}

void setup( void )
{
  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(13,OUTPUT);
  
  //byte    input[bytesToRead]; 
    //Select Channel 3 of the SMBus switch
  digitalWrite(7, HIGH); // sets the digital pin D7 on
  delay(1000);            // waits for a second
  digitalWrite(9, HIGH);  // sets the digital pin D9 on
  delay(1000);            // waits for a second
  Serial.begin( 9600 );
  delay( 1000 );
  //Serial.println( "Test writing Retimer to generate PRBS31" );
  Wire.begin(); // Initialize I2C library
  Wire.beginTransmission(0x73);
  Wire.write(8);
  Wire.endTransmission();

}

void readSwitch(byte address)
{
  byte read1;
  byte read2;
  Wire.requestFrom(address, 1); // select number of bytes to get from the device (2 bytes in this case)
  read1= Wire.read(); // read next byte from the following register
  read2=Wire.read();
 // Serial.print(switch_address);
  Serial.print("First byte: ");
  Serial.println(read1,HEX);

}

void loop( void )
{

  //Serial.println( "Test writing Retimer to generate PRBS31" );

  digitalWrite(13, HIGH);
  delay(1000);
  
  digitalWrite(13, LOW);
  delay(1000);
  byte error, address; //variable for error and I2C address
  int nDevices;
  PRBS31_Retimer_1();

//   Serial.println("Scanning...");

//   nDevices = 0;
//   for (address = 1; address < 127; address++ )
//   {
//     // The i2c_scanner uses the return value of
//     // the Write.endTransmisstion to see if
//     // a device did acknowledge to the address.
//     Wire.beginTransmission(address);
//     error = Wire.endTransmission();

//     if (error == 0)
//     {
//       Serial.print("I2C device found at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.print(address, HEX);
//       Serial.println("  !");
//       nDevices++;
//     }
//     else if (error == 4)
//     {
//       Serial.print("Unknown error at address 0x");
//       if (address < 16)
//         Serial.print("0");
//       Serial.println(address, HEX);
//     }
//   }
//   if (nDevices == 0)
//     Serial.println("No I2C devices found\n");
//   else
//     Serial.println("done\n");

//   delay(5000); // wait 5 seconds for the next I2C scan


// readSwitch(address);


// writeSwitch(address,channel1);
// readSwitch(address);

// writeSwitch(address,channel2);
// readSwitch(address);

// writeSwitch(address,channel3);
// readSwitch(address);

// writeSwitch(address,channel4);
// readSwitch(address);

  
}

void writeSwitch(byte address,byte channel)
{
  Wire.beginTransmission(address);
  Wire.write(1<<channel);
  Wire.endTransmission();

}
