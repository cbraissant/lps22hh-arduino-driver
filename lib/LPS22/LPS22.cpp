#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "LPS22.h"
/**
 *    ____                                      
 *   / ___|    ___   _ __    ___    ___    _ __ 
 *   \___ \   / _ \ | '_ \  / __|  / _ \  | '__|
 *    ___) | |  __/ | | | | \__ \ | (_) | | |   
 *   |____/   \___| |_| |_| |___/  \___/  |_|
 * 
*/


/**
 * @brief Construct a new LPS22
*/
LPS22::LPS22(){
}


/**
 * @brief Initialise the output pin
 * @param cs_pin Chip select pin
*/
void LPS22::setCsPin(int8_t cs_pin){
    this->cs_pin = cs_pin;
    pinMode(cs_pin, OUTPUT);
    disableSPI();
}


/**
 * @brief Software reset. Registers are reset to their default
 * value if set to "1". Automatically return to "0" when the
 * reset is completed.
*/
void LPS22::swreset(void) {
    writeSingleBit(LPS_CTRL_REG2, 2, 1);
    while( readSingleBit(LPS_CTRL_REG2, 2) ==1  ){
        delay(1);
    };
}


/**
 * @brief Set the output data rate of the device
 * @param ODR the required Output Data Rate
*/
void LPS22::setDataRate(lps_odr ODR) {
  Serial.println("Debug - setDataRate - Before: ");
  Serial.print("ODR: ");
  Serial.println(ODR);
  Serial.print("Control Register: ");
  Serial.println(readSingleRegister(LPS_CTRL_REG1));
  writeMultiBits(LPS_CTRL_REG1, 4, ODR, 3);

  Serial.print("Debug - setDataRate - After: ");
  Serial.println(readSingleRegister(LPS_CTRL_REG1));
}


/**
 * @brief Read the internal ID of the chip
 * @return ID of the sensor
*/
uint8_t LPS22::whoAmI(void){
  return readSingleRegister(LPS_WHO_AM_I);
}


/**
 * @brief Check if a pressure data is available
 * @return True if new data, False otherwise
*/
bool LPS22::hasNewPressure(void){
  return readSingleBit(LPS_STATUS,0);
}


/**
 * @brief Read the status register
 * @return Value of the register
*/
uint8_t LPS22::getStatus(void){
  return readSingleRegister(LPS_STATUS);
}


/**
 * @brief Retrieve the absolute pressure
 * @return Raw value of the absolute pressure 
*/
int32_t LPS22::getPressureValue(void){
  uint8_t buffer[3];
  readMultiRegister(buffer, LPS_PRESSURE_OUT_XL, 3);

  // To optain the pressure, concatenate the values of the buffer
  int32_t pressure = 0;
  pressure = (int32_t)buffer[2];
  pressure <<= 8;
  pressure |= (int32_t)(buffer[1]);
  pressure <<= 8;
  pressure |= (int32_t)(buffer[0]); 
  return pressure;
}


/**
 * @brief Retrieve the absolute pressure
 * @return Absolute pressure in hPa 
*/
float LPS22::getPressure(void){
  return getPressureValue()*LPS_PRESSURE_RESOLUTION;
}


/**
 * @brief Retrieve the temperature
 * @return Raw value of the temperature 
*/
int16_t LPS22::getTemperatureValue(void){
  uint8_t buffer[2];
  readMultiRegister(buffer, LPS_TEMP_OUT_L, 2);

  // To optain the temperature, concatenate the values of the buffer
  int16_t temperature = 0;
  temperature |= (int16_t)(buffer[1]);
  temperature <<= 8;
  temperature |= (int16_t)(buffer[0]); 
  return temperature;
}


/**
 * @brief Retrieve the temperature
 * @return Temperature in Celsius 
*/
float LPS22::getTemperature(void){
  return getTemperatureValue()*LPS_TEMPERATURE_RESOLUTION;
}



/**
 *    ____    ____    ___ 
 *   / ___|  |  _ \  |_ _|
 *   \___ \  | |_) |  | | 
 *    ___) | |  __/   | | 
 *   |____/  |_|     |___|
 * 
*/     


/**
 * @brief Enable communication on the SPI bus
*/
void LPS22::enableSPI(void){
  digitalWrite(cs_pin, LOW);
}


/**
 * @brief Disable communication on the SPI bus 
*/
void LPS22::disableSPI(void){
  digitalWrite(cs_pin, HIGH);
}


/**
 *    ____                   _         _                 
 *   |  _ \    ___    __ _  (_)  ___  | |_    ___   _ __ 
 *   | |_) |  / _ \  / _` | | | / __| | __|  / _ \ | '__|
 *   |  _ <  |  __/ | (_| | | | \__ \ | |_  |  __/ | |   
 *   |_| \_\  \___|  \__, | |_| |___/  \__|  \___| |_|   
 *                   |___/
*/


/**
 * @brief Read the value of a register
 * @param reg Address of the register to read
 * @return Value of the register
*/
uint8_t LPS22::readSingleRegister(uint8_t reg){
  enableSPI();
  SPI.transfer(reg | 0x80);
  uint8_t data = SPI.transfer(0x00);
  disableSPI();
  return data;
}


/**
 * @brief Read the value of multiple registers
 * @param reg  Address of the first register to read
 * @param numRegs Number of register to read
 * @return Value of the registers
*/
void LPS22::readMultiRegister(uint8_t *buffer, uint8_t reg, uint8_t numRegs){
  enableSPI();
  SPI.transfer(reg | 0x80);
  for (uint8_t i=0; i<numRegs; i++){
    buffer[i] = SPI.transfer(0x00);
  }
  disableSPI();
}


/**
 * @brief Write a value to a register
 * @param reg Address of the register to write
 * @param value value to write to the register
*/
void LPS22::writeSingleRegister(uint8_t reg, uint8_t value){
  enableSPI();
  SPI.transfer(reg);
  SPI.transfer(value);
  disableSPI();
}

/**
 *    ____    _   _         
 *   | __ )  (_) | |_   ___ 
 *   |  _ \  | | | __| / __|
 *   | |_) | | | | |_  \__ \
 *   |____/  |_|  \__| |___/
 * 
*/


/**
 * @brief Read the value of a single bit of a register
 * @param reg Address of the register
 * @param position Position of the bit to read (0 = LSB)
 * @return Value of the read register
*/
bool LPS22::readSingleBit(uint8_t reg, uint8_t position){
  uint8_t data = readSingleRegister(reg);
  return (data & (1<<position)) != 0;
}


/**
 * @brief Read the value of multiple bits of a register
 * @param reg Address of the register
 * @param position Position of the bit to read (0 = LSB)
 * @param numBits Number of bits to read
*/
uint8_t LPS22::readMultiBits(uint8_t reg, uint8_t position, uint8_t numBits){
  uint8_t data = readSingleRegister(reg);
  uint8_t mask = (1<<numBits) - 1;
  data >>= position;
  data &= mask;
  return (data);
  // return ( (data >> position) & ((1<<numBits) - 1)))
}


/**
 * @brief Write the value of a single bit to a register
 * @param reg Address of the register
 * @param position Position of the bit to write (0 = LSB)
 * @param value Value of the bit to write
*/
void LPS22::writeSingleBit(uint8_t reg, uint8_t position, bool value){
  uint8_t data = readSingleRegister(reg);
  uint8_t mask = 1 << position;
  if (data) {
    data |= mask;
  } else {
    data &= ~mask;
  };
  // data = data ? (data | mask) : (data & ~mask)
  writeSingleRegister(reg, data);
}


/**
 * @brief Write the value of mulitple bits to a register
 * @param reg Address of the register
 * @param position Position of the bits to write (0 = LSB)
 * @param value Value of the bits to write
 * @param numBits Number of bits to write
*/
void LPS22::writeMultiBits(uint8_t reg, uint8_t position, uint8_t value, uint8_t numBits){
  uint8_t data = readSingleRegister(reg);
  uint8_t mask = (1 << numBits ) - 1;
  mask <<= position;
  data &= ~mask;
  data |= value << position;
  writeSingleRegister(reg, data);
}
