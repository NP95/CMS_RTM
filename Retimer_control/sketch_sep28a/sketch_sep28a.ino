#include <Wire.h>
//Retimer Address
#define Retimer_WriteAddress 0x36
#define Retimer_Read_Address 0x37

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
#define PRBS_enable_data  0x10
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


void setRegister(byte address,byte data,byte mask)
{
      Wire.beginTransmission(0x36);   
      Wire.write(address)
      Wire.write( (data&mask );
      delay(100)
      Wire.write( (data&mask );      
      Wire.endTransmission( true );     
}



void setup( void )
{
  pinMode(D7, OUTPUT);
  pinMode(D9, OUTPUT);
    //Select Channel 3 of the SMBus switch
  digitalWrite(D7, HIGH); // sets the digital pin D7 on
  delay(1000);            // waits for a second
  digitalWrite(D9, HIGH);  // sets the digital pin D9 on
  delay(1000);            // waits for a second

  Serial.begin( 9600 );
  delay( 1000 );
  Serial.println( "Test writing Retimer to generate PRBS31" );
  Wire.begin(); // Initialize I2C library
}

void loop( void )
{

setRegister(Broadcast_Enable,Broadcast_Enable_Data,Broadcast_Enable_Mask);
setRegister(Channel_Registers,Reset_Channel_Registers_Data,Reset_Channel_Registers_Mask);
setRegister(Preset_Signal_Detect,,);
setRegister(Override,Divider_Select_Override_Data,Divider_Select_Override_Mask);
setRegister(Override,VCO_Cap_Select_Override_Data,VCO_Cap_Select_Override_Mask);
setRegister(VCO_Cap_Count,VCO_Cap_Count_Data,VCO_Cap_Count_Mask);
setRegister(Divider_Select,Divider_Select_Data,Divider_Select_Mask);
setRegister(Override,Charge_Pump_Down_Override_Data,Charge_Pump_Down_Override_Mask);
setRegister(Charge_Pump_Control,Charge_Pump_Down_Enable_Data,Charge_Pump_Down_Enable_Mask);
setRegister(Override,Loop_filter_DAC_Override_Data,Loop_filter_DAC_Override_Mask);
setRegister(Loop_filter_DAC,Loop_filter_DAC_Override_Value_Data,Loop_filter_DAC_Override_Value_Mask);
setRegister(PRBS_enable,PRBS_enable_Data,PRBS_enable_Mask);
setRegister(Clock_enable,Clock_enable_Data,Clock_enable_Mask);
setRegister(Override,Loopthr_select_override_Data,Loopthr_select_override_Mask);
setRegister(Output_Mux_PRBS_Gen,Output_Mux_PRBS_Gen_Data,Output_Mux_PRBS_Gen_Mask);
setRegister(PRBS_Load,PRBS_Load_Data,PRBS_Load_Mask);

}


