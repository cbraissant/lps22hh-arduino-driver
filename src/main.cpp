#include <Arduino.h>
#include <SPI.h>
#include <LPS22.h>

const int CS_pin = 10; // Chip Select pin for the LPS22HH

byte pressure_xl, pressure_l, pressure_h; // Variables to store pressure data
byte temp_l, temp_h; // Variables to store temperature data

int readRegister(int register_address) {
  digitalWrite(CS_pin, LOW);
  SPI.transfer(register_address | 0x80);
  int data = SPI.transfer(0x00);
  digitalWrite(CS_pin, HIGH);
  return data;
}

void writeRegister(int register_address, int register_value) {
  digitalWrite(CS_pin, LOW);
  SPI.transfer(register_address);
  SPI.transfer(register_value);
  digitalWrite(CS_pin, HIGH);
}



/**
 * @brief
 * 
 * Set the output data rate
 * 
 * @param output_data_rate value of the Output data rate bit configurations 
*/
void setDataRate(lps_odr output_data_rate){
  // read the actual configuration of the register
  int controlRegister = readRegister(LPS_CTRL_REG1);
  Serial.print("Actual control register: ");
  Serial.println(controlRegister);

  // change the data rate
  controlRegister |= output_data_rate << 4;
  Serial.print("Futur control register: ");
  Serial.println(controlRegister);
  
  // write the new configuration to the register
  writeRegister(LPS_CTRL_REG1, controlRegister);

  // read the configuration to confirm
  controlRegister = readRegister(LPS_CTRL_REG1);
  Serial.print("New control register: ");
  Serial.println(controlRegister);

  // Disable SPI communication
  digitalWrite(CS_pin, LOW); 
}




void setup() {
  Serial.begin(115200);
  Serial.println("Programm started");

  SPI.begin();

  pinMode(CS_pin, OUTPUT);
  digitalWrite(CS_pin, HIGH);

  // Initialize the SPI settings
  SPI.beginTransaction(SPISettings(LPS_SPI_CLOCK_FREQUENCY, MSBFIRST, SPI_MODE3));

  // Read the WHO_AM_I register
  byte who_am_i = readRegister(LPS_WHO_AM_I);
  Serial.print("Who Am I? ");
  Serial.println(who_am_i);

  // Change the Output data rate
  setDataRate(LPS_ODR_10_HZ);

}

void loop() {
  // Read the temperature
  temp_l = readRegister(LPS_TEMP_OUT_L);
  temp_h = readRegister(LPS_TEMP_OUT_H);

  int32_t temperature = (temp_h << 8) | temp_l;

  Serial.print("Temp: ");
  Serial.println(temperature*LPS_TEMPERATURE_RESOLUTION);

  // Read the 3 bytes of the pressure
  pressure_xl = readRegister(LPS_PRESSURE_OUT_XL); // Read the pressure low byte
  pressure_l = readRegister(LPS_PRESSURE_OUT_L); // Read the pressure medium byte
  pressure_h = readRegister(LPS_PRESSURE_OUT_H); // Read the pressure high byte

  // Concatenate the pressures
  int32_t pressure;
  pressure = (int32_t)(pressure_h);
  pressure <<= 8;
  pressure |= (int32_t)(pressure_l);
  pressure <<= 8;
  pressure |= (int32_t)(pressure_xl);


  Serial.print("Pressure: ");
  Serial.println(pressure*LPS_PRESSURE_RESOLUTION);

  // Repeat the loop
  delay(1000);
}