#include <Arduino.h>

#include <Wire.h>
// BME280 7-bit address  0x77 or 0x76
//I2C address bit 0:        GND:0, VDDIO:1
#define BME280Address 0x76

//DIG_T1: unsigned short
//DIG_T2/3: signed short
#define DIG_T1_LO_REGISTER_ADDRESS 0x88
#define DIG_T1_HI_REGISTER_ADDRESS 0x89
#define DIG_T2_LO_REGISTER_ADDRESS 0x8A
#define DIG_T2_HI_REGISTER_ADDRESS 0x8B
#define DIG_T3_LO_REGISTER_ADDRESS 0x8C
#define DIG_T3_HI_REGISTER_ADDRESS 0x8D


#define DIG_H1_REGISTER_ADDRESS 0xA1
#define DIG_H2_LO_REGISTER_ADDRESS 0xE1
#define DIG_H2_HI_REGISTER_ADDRESS 0xE2

#define DIG_H3_REGISTER_ADDRESS 0xE3
#define DIG_H4_HI_REGISTER_ADDRESS 0xE4
#define DIG_H4_H5_REGISTER_ADDRESS 0xE5
#define DIG_H5_HI_REGISTER_ADDRESS 0xE6

int analogPin[6]; 
float voltageAtAdcPin;  
int i;

/*
DIG_T1: unsigned short
DIG_T2/3: signed short
*/

unsigned short dig_T1;
signed short dig_T2,dig_T3;


// humidity compensation data
unsigned char dig_H1,dig_H3;
signed short dig_H2,dig_H4,dig_H5;

void ConfigAnalogPins()
{
  analogPin[0] = A0;
  analogPin[1] = A1;
  analogPin[2] = A2;
  analogPin[3] = A3;
  analogPin[4] = A4;
  analogPin[5] = A5;
};

void InitBme280I2c()
{
  //Wire.begin(address)
  // address: the 7-bit slave address (optional); if not specified, join the bus as a master.
  Wire.begin(BME280Address);
}

// 0xFD: hum_msb[7:0] contains msb part of hum[15:8] of the raw humidity measurement output data
// 0xFE: hum_lsb[7:0] contains lsb part of hum[7:0] of the raw humidity measurement output data
int ReadBME280HumidityData()
{
  int16_t reading;
  Wire.beginTransmission(BME280Address); // transmit to device    
  Wire.write(byte(0xFD));      // sets register pointer to the command register (0x00)
  Wire.endTransmission();      // stop transmitting


  Wire.beginTransmission(BME280Address); // transmit to device    
  Wire.requestFrom(BME280Address, 2);    // request 2 bytes from slave device 
  Wire.beginTransmission(BME280Address); // transmit to device    


  if (2 <= Wire.available()) { // if two bytes were received

    reading = Wire.read();  // receive high byte (overwrites previous reading)

    reading = reading << 8;    // shift high byte to be high 8 bits

    reading |= Wire.read(); // receive low byte as lower 8 bits

    //Serial.println(reading);   // print the reading
  }
  return reading;
}

int16_t ReadCompRegister(byte registerAddress)
{
  int16_t data=0;
  Wire.beginTransmission(BME280Address); // transmit to device    
  Wire.write(byte(registerAddress));      // sets register pointer to the command register (0x00)
  Wire.endTransmission();      // stop transmitting

  // dig_H3 data
  Wire.beginTransmission(BME280Address); // transmit to device    
  Wire.requestFrom(BME280Address, 1);    // request 1 bytes from slave device 
  Wire.beginTransmission(BME280Address); // transmit to device    
  data=Wire.read();
  return data;  // receive high byte (overwrites previous reading)

  
}

void ReadAllHumidityCompRegister()

{
  int16_t dataHi,dataLo;
  dig_H1=ReadCompRegister(DIG_H1_REGISTER_ADDRESS);
  // register address 0xE1/0xE2, data dig_H2[7:0]/[15:8]
  dataLo=ReadCompRegister(DIG_H2_LO_REGISTER_ADDRESS);
  dataHi=ReadCompRegister(DIG_H2_HI_REGISTER_ADDRESS);
  dig_H2=(dataHi)<<8 & (dataLo);

  dig_H3=ReadCompRegister(DIG_H3_REGISTER_ADDRESS);

  dataHi=ReadCompRegister(DIG_H4_HI_REGISTER_ADDRESS);
  dataLo=ReadCompRegister(DIG_H4_H5_REGISTER_ADDRESS);
  dig_H4=(dataHi<<4)|(dataLo & 0x000F);// dig_H4[11:4]/[3:0]

  dataHi=ReadCompRegister(DIG_H5_HI_REGISTER_ADDRESS);// dig_H5[11:4]
  dig_H5=(dataHi<<4)|((dataLo & 0x00F0)>>4); // // dig_H5[11:4]/[3:0]   0xE6,0xE5[7:4]

}


void ReadAllTempCompRegister()
{
  int16_t dataHi,dataLo;
  dataLo=ReadCompRegister(DIG_T1_LO_REGISTER_ADDRESS);
  dataHi=ReadCompRegister(DIG_T1_HI_REGISTER_ADDRESS);
  dig_T1=(dataHi<<8)|dataLo;

  // register address 0xE1/0xE2, data dig_H2[7:0]/[15:8]
  dataLo=ReadCompRegister(DIG_T2_LO_REGISTER_ADDRESS);
  dataHi=ReadCompRegister(DIG_T2_HI_REGISTER_ADDRESS);
  dig_T3=(dataHi<<8)|dataLo;

  dataLo=ReadCompRegister(DIG_T3_LO_REGISTER_ADDRESS);
  dataHi=ReadCompRegister(DIG_T3_HI_REGISTER_ADDRESS);
  dig_T3=(dataHi<<8)|dataLo;
}


void setup() 
{
  Serial.begin(115200);           //  setup serial
  ConfigAnalogPins();
}


void loop() {
  // send the Gas Sensor ADC value to the SerialPort
  for(i=0;i<6;i++)
  {
    voltageAtAdcPin = (float) analogRead(analogPin[i])/1024*5;  // read the input pin

    Serial.print(String(i+1));
    Serial.print(voltageAtAdcPin,4);          // debug value

    Serial.print("\r\n");
    delay(100);
  }

// use serial port console to see what is printed 
  Serial.print("T1");
  Serial.print(dig_T1,BIN);
  Serial.print("\r\n");
  delay(100);

  Serial.print("H2");
  Serial.print(dig_T2,BIN);
  Serial.print("\r\n");
  delay(100);

  Serial.print("H3");
  Serial.print(dig_T3,BIN);
  Serial.print("\r\n");
  delay(100);
  // Send the Humidity Data and its compensation the the SerialPort
  // Compensation algorithm, Page 26: 
  // https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf
  // unsigned char dig_H1,dig_H3;
  // signed short dig_H2,dig_H4,dig_H5;




  Serial.print("H1");
  Serial.print(dig_H1,BIN);
  Serial.print("\r\n");
  delay(100);

  Serial.print("H2");
  Serial.print(dig_H2,BIN);
  Serial.print("\r\n");
  delay(100);

  Serial.print("H3");
  Serial.print(dig_H3,BIN);
  Serial.print("\r\n");
  delay(100);

  Serial.print("H4");
  Serial.print(dig_H4,BIN);
  Serial.print("\r\n");
  delay(100);

  Serial.print("H5");
  Serial.print(dig_H5,BIN);
  Serial.print("\r\n");
  delay(100);

}

