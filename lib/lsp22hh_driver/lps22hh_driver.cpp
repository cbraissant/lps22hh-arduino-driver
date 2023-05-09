/*!
 * @file LPS22.h
 *
 * Driver for the ST LPS22 pressure sensor
 */

#include <Arduino.h>
#include <stdint.h>
#include <SPI.h>
#include "lps22hh_driver.h"


/**
 * @brief Construct a new LPS22
*/
LPS22HH_driver::LPS22HH_driver(){
}


/**
 * @brief Initialize the LPS22
*/
void LPS22HH_driver::init(SPIClass *theSpi, int8_t csPin){
  setSpi(theSpi);
  setCsPin(csPin);
}


/**
 * @brief Attribute the chip select pin
 * @param csPin Chip select pin
*/
void LPS22HH_driver::setSpi(SPIClass *theSpi){
    this->_spi = theSpi;
}


/**
 * @brief Attribute the chip select pin
 * @param csPin Chip select pin
*/
void LPS22HH_driver::setCsPin(int8_t csPin){
    this->_csPin = csPin;
    pinMode(_csPin, OUTPUT);
    endTransaction();
}


/**
 * @brief Software reset. Registers are reset to their default
 * value if set to "1". Automatically return to "0" when the
 * reset is completed.
*/
void LPS22HH_driver::reset(void) {
    writeSingleBit(LPS22HH_CTRL_REG2, 2, 1);
}


/**
 * @brief Set the output data rate (ODR) of the device
 * @param dataRate the required Output Data Rate
*/
void LPS22HH_driver::setDataRate(lps22hh_odr dataRate) {
  writeMultiBits(LPS22HH_CTRL_REG1, 4, 3, dataRate);
}


lps22hh_odr LPS22HH_driver::getDataRate(void){
  return static_cast<lps22hh_odr>(readMultiBits(LPS22HH_CTRL_REG1, 4, 3));
}


/**
 * @brief Set the FIFO mode of the device
 * @param fifoMode the required FIFO mode
*/
void LPS22HH_driver::setFifoMode(lps22hh_fifo fifoMode) {
  writeMultiBits(LPS22HH_FIFO_CTRL, 0, 2, fifoMode);
}


/**
 * @brief Set the Block Data Update (BDU) mode
 * @param bduMode the desired Block Data Update mode
*/
void LPS22HH_driver::setBlockDataUpdate(lps22hh_bdu bduMode){
   writeSingleBit(LPS22HH_CTRL_REG1, 1, bduMode);
}


/**
 * @brief Enable low-pass filter on pressure data when Continuous mode is used
 * @param filterEnabled True to enable the filter
*/
void LPS22HH_driver::setLowPassFilter(bool filterEnabled){
  writeSingleBit(LPS22HH_CTRL_REG1, 3, filterEnabled);
}


/**
 * @brief Configure the bandwidth of the low pass filter
 * @param filterBandwidth The desired bandwidth
*/
void LPS22HH_driver::setFilterBandwidth(lps22hh_lpfp filterBandwidth){
  writeSingleBit(LPS22HH_CTRL_REG1, 2, filterBandwidth);
}


/**
 * @brief Configure the bandwidth of the low pass filter
 * @param filterBandwidth The desired bandwidth
*/
void LPS22HH_driver::setLowNoise(bool lowNoiseEnabled){
  // Note: For proper behavior of the pressure sensor,
  // the LOW_NOISE_EN bit must be changed only when the
  // device is in power-down (AN5209 - 3.4)
  lps22hh_odr ODR = getDataRate();
  powerDown();
  writeSingleBit(LPS22HH_CTRL_REG2, 1, lowNoiseEnabled);
  setDataRate(ODR);
}



/**
 * @brief Read the internal ID of the chip
 * @return ID of the sensor
*/
uint8_t LPS22HH_driver::getDeviceId(void){
  return readSingleRegister(LPS22HH_WHO_AM_I);
}


/**
 * @brief Trigger a new measurement when in standby
*/
void LPS22HH_driver::triggerNewMeasurement(void){
  writeSingleBit(LPS22HH_CTRL_REG2, 0, 1);
}


/**
 * @brief Set the data rate to ONE_SHOT which put the device in standby mode
*/
void LPS22HH_driver::powerDown(void){
  setDataRate(LPS22HH_ODR_ONE_SHOT);
}


/**
 * @brief Check if a pressure data is available
 * @return True if new data, False otherwise
*/
bool LPS22HH_driver::hasNewPressure(void){
  return readSingleBit(LPS22HH_STATUS,0);
}


/**
 * @brief Read the status register
 * @return Value of the register
*/
uint8_t LPS22HH_driver::getStatus(void){
  return readSingleRegister(LPS22HH_STATUS);
}


/**
 * @brief Retrieve the absolute pressure
 * @return Raw value of the absolute pressure 
*/
int32_t LPS22HH_driver::getRawPressure(void){
  uint8_t buffer[3];
  readMultiRegister(buffer, LPS22HH_PRESSURE_OUT_XL, 3);

  // To optain the pressure, concatenate the values of the buffer
  int32_t pressure = 0;
  pressure = buffer[2];
  pressure <<= 8;
  pressure |= buffer[1];
  pressure <<= 8;
  pressure |= buffer[0]; 
  return pressure;
}


/**
 * @brief Retrieve the absolute pressure
 * @return Absolute pressure in hPa 
*/
float LPS22HH_driver::getPressure(void){
  return getRawPressure()*LPS22HH_PRESSURE_RESOLUTION;
}


/**
 * @brief Retrieve the temperature
 * @return Raw value of the temperature 
*/
int16_t LPS22HH_driver::getRawTemperature(void){
  uint8_t buffer[2];
  readMultiRegister(buffer, LPS22HH_TEMP_OUT_L, 2);

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
float LPS22HH_driver::getTemperature(void){
  return getRawTemperature()*LPS22HH_TEMPERATURE_RESOLUTION;
}
  

/**
 * @brief Configure the pressure offset
 * @param offset Pressure offset
 * @remark The offset is then multiplied by 256 and substracted
 * from the p_compensated(t)   
*/
void LPS22HH_driver::setPressureOffset(uint16_t offset){
  writeMultiRegister(LPS22HH_RPDS_L, 2, offset);
}



/**
 * @brief Enable communication on the SPI bus
*/
void LPS22HH_driver::beginTransaction(void){
  digitalWrite(_csPin, LOW);
}


/**
 * @brief Disable communication on the SPI bus 
*/
void LPS22HH_driver::endTransaction(void){
  digitalWrite(_csPin, HIGH);
}


/**
 * @brief Read the value of a register
 * @param reg Address of the register to read
 * @return Value of the register
*/
uint8_t LPS22HH_driver::readSingleRegister(uint8_t reg){
  beginTransaction();
  _spi->transfer(reg | 0x80);
  uint8_t data = _spi->transfer(0x00);
  endTransaction();
  return data;
}


/**
 * @brief Read the value of multiple registers
 * @param reg  Address of the first register to read
 * @param numRegs Number of register to read
 * @return Value of the registers
*/
void LPS22HH_driver::readMultiRegister(uint8_t *buffer, uint8_t reg, uint8_t numRegs){
  beginTransaction();
  _spi->transfer(reg | 0x80);
  for (uint8_t i=0; i<numRegs; i++){
    buffer[i] = _spi->transfer(0x00);
  }
  endTransaction();
}


/**
 * @brief Write a value to a register
 * @param reg Address of the register to write
 * @param value value to write to the register
*/
void LPS22HH_driver::writeSingleRegister(uint8_t reg, uint8_t value){
  beginTransaction();
  _spi->transfer(reg);
  _spi->transfer(value);
  endTransaction();
}


/**
 * @brief Write a value to multiple registers
 * @param reg Address of the first register to write
 * @param numRegs Number of register to write
 * @param value value to write to the register
*/
void LPS22HH_driver::writeMultiRegister(uint8_t reg, uint8_t numRegs, uint32_t value){
  beginTransaction();
  _spi->transfer(reg);
  for (uint8_t i=0; i<numRegs; i++){
    _spi->transfer(value);
    value = value >> 8;
  }
  endTransaction();
}


/**
 * @brief Read the value of a single bit of a register
 * @param reg Address of the register
 * @param position Position of the bit to read (0 = LSB)
 * @return Value of the read register
*/
bool LPS22HH_driver::readSingleBit(uint8_t reg, uint8_t position){
  uint8_t data = readSingleRegister(reg);
  return (data & (1<<position)) != 0;
}


/**
 * @brief Read the value of multiple bits of a register
 * @param reg Address of the register
 * @param position Position of the bit to read (0 = LSB)
 * @param numBits Number of bits to read
*/
uint8_t LPS22HH_driver::readMultiBits(uint8_t reg, uint8_t position, uint8_t numBits){
  uint8_t data = readSingleRegister(reg);
  uint8_t mask = (1<<numBits) - 1;
  data >>= position;
  data &= mask;
  return (data);
}


/**
 * @brief Write the value of a single bit to a register
 * @param reg Address of the register
 * @param position Position of the bit to write (0 = LSB)
 * @param value Value of the bit to write
*/
void LPS22HH_driver::writeSingleBit(uint8_t reg, uint8_t position, bool value){
  uint8_t data = readSingleRegister(reg);
  uint8_t mask = 1 << position;
  if (value) {
    data |= mask;
  } else {
    data &= ~mask;
  };
  writeSingleRegister(reg, data);
}


/**
 * @brief Write the value of mulitple bits to a register
 * @param reg Address of the register
 * @param position Position of the bits to write (0 = LSB)
 * @param value Value of the bits to write
 * @param numBits Number of bits to write
*/
void LPS22HH_driver::writeMultiBits(uint8_t reg, uint8_t position, uint8_t numBits, uint8_t value){
  uint8_t data = readSingleRegister(reg);
  uint8_t mask = (1 << numBits ) - 1;
  mask <<= position;
  data &= ~mask;
  data |= value << position;
  writeSingleRegister(reg, data);
}
