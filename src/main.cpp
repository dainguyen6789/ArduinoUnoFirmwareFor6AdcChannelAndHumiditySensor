#include <Arduino.h>

#include <Wire.h>
// BME280 7-bit address  0x77 or 0x76
//I2C address bit 0:        GND:0, VDDIO:1
#define BME280Address 0x76;

int analogPin[6]; // potentiometer wiper (middle terminal) connected to analog pin 3
                    // outside leads to ground and +5V
float voltageAtAdcPin;  // variable to store the value read
int i;

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
    if (2 <= Wire.available()) { // if two bytes were received

    reading = Wire.read();  // receive high byte (overwrites previous reading)

    reading = reading << 8;    // shift high byte to be high 8 bits

    reading |= Wire.read(); // receive low byte as lower 8 bits

    Serial.println(reading);   // print the reading

  }
  Wire.beginTransmission(BME280Address); // transmit to device    
  Wire.write(byte(0xFD));      // sets register pointer to the command register (0x00)
  Wire.endTransmission();      // stop transmitting


  Wire.beginTransmission(BME280Address); // transmit to device    
  Wire.requestFrom(BME280Address, 2);    // request 2 bytes from slave device #112
  Wire.beginTransmission(BME280Address); // transmit to device    


  if (2 <= Wire.available()) { // if two bytes were received

    reading = Wire.read();  // receive high byte (overwrites previous reading)

    reading = reading << 8;    // shift high byte to be high 8 bits

    reading |= Wire.read(); // receive low byte as lower 8 bits

    //Serial.println(reading);   // print the reading
  }
  return reading;
}

void setup() {
  Serial.begin(115200);           //  setup serial

  ConfigAnalogPins();
}


void loop() {

  for(i=0;i<6;i++)
  {
    voltageAtAdcPin = (float) analogRead(analogPin[i])/1024*5;  // read the input pin

    Serial.print(String(i+1));
    Serial.print(val,4);          // debug value

    Serial.print("\r\n");
    delay(100);
  }

}

