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


void setup() {
  Serial.begin(115200);           //  setup serial

  ConfigAnalogPins();
}


void loop() {

  for(i=0;i<6;i++)
  {
    voltageAtAdcPin = (float) analogRead(analogPin[i])/1024*5;  // read the input pin

    Serial.print(String(i+1));
    Serial.print(voltageAtAdcPin,4);          // debug value

    Serial.print("\r\n");
    delay(100);
  }

}

