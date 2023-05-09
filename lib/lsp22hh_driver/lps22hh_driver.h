/*!
 * @file LPS22.h
 *
 * Driver for the ST LPS22 pressure sensor
 */

#ifndef _LPS22HH_DRIVER_H
#define _LPS22HH_DRIVER_H

#include <Arduino.h>
#include <Wire.h>


/** Registers definition
 */
#define LPS22HH_INTERRUPT_CFG 0x0B  ///< Interrupt mode register
#define LPS22HH_THS_P_L 0x0C        ///< Pressure threshold registers (LSB)
#define LPS22HH_THS_P_H 0x0D        ///< Pressure threshold registers (MSB)
#define LPS22HH_IF_CTRL 0x0E        ///< Interface control register
#define LPS22HH_WHO_AM_I 0x0F       ///< Who am I

#define LPS22HH_CTRL_REG1 0x10      ///< Control register (LSB)
#define LPS22HH_CTRL_REG2 0x11      ///< Control register
#define LPS22HH_CTRL_REG3 0x12      ///< Control register (MSB)

#define LPS22HH_FIFO_CTRL 0x13      ///< FIFO configuration register
#define LPS22HH_FIFO_WTM 0x14       ///< FIFO Threshold setting register
#define LPS22HH_REF_P_L 0x15        ///< Reference pressure register (LSB)
#define LPS22HH_REF_P_H 0x16        ///< Reference pressure register (MSB)

#define LPS22HH_RPDS_L 0x18         ///< Pressure offset register (LSB)
#define LPS22HH_RPDS_H 0x19         ///< Pressure offset register (MSB)

#define LPS22HH_INT_SOURCE 0x24     ///< Interrupt register
#define LPS22HH_FIFO_STATUS1 0x25   ///< FIFO status register
#define LPS22HH_FIFO_STATUS2 0x26   ///< FIFO status register

#define LPS22HH_STATUS 0x27         ///< Status register

#define LPS22HH_PRESSURE_OUT_XL 0x28    ///< Pressure output register (LSB)
#define LPS22HH_PRESSURE_OUT_L 0x29     ///< Pressure output register
#define LPS22HH_PRESSURE_OUT_H 0x2A     ///< Pressure output register (MSB)
#define LPS22HH_TEMP_OUT_L 0x2B         ///< Temperature output register (LSB)
#define LPS22HH_TEMP_OUT_H 0x2C         ///< Temperature output register (LSB)

#define LPS22HH_FIFO_DATA_OUT_PRESS_XL 0x78 ///< FIFO pressure output register
#define LPS22HH_FIFO_DATA_OUT_PRESS_L 0x79  ///< FIFO pressure output register
#define LPS22HH_FIFO_DATA_OUT_PRESS_H 0x7A  ///< FIFO pressure output register
#define LPS22HH_FIFO_DATA_OUT_TEMP_L 0x7B   ///< FIFO temperature output registers
#define LPS22HH_FIFO_DATA_OUT_TEMP_H 0x7C   ///< FIFO temperature output registers

/** Characteristics
*/
#define LPS22HH_MIN_PRESSURE 260    // hPa
#define LPS22HH_MAX_PRESSURE 1260   // hPa
#define LPS22HH_PRESSURE_SENSITIVITY 4096       // 4096 LSB = 1 hPa
#define LPS22HH_PRESSURE_RESOLUTION 0.00024414  // 1 LSB = 1/4096 = 0.0002441406 hPa
#define LPS22HH_TEMPERATURE_SENSITIVITY 100     // 100 LSB = °C
#define LPS22HH_TEMPERATURE_RESOLUTION  0.01    // 1 LSB = 1/100 = 0.01 °C
#define LPS22HH_SPI_CLOCK_FREQUENCY 2000000      // 100 kHz


/**
 * @brief
 * Allowed values to configure the Output data rate bits
*/
typedef enum{
    LPS22HH_ODR_ONE_SHOT,
    LPS22HH_ODR_1_HZ,
    LPS22HH_ODR_10_HZ,
    LPS22HH_ODR_25_HZ,
    LPS22HH_ODR_50_HZ,
    LPS22HH_ODR_75_HZ,
    LPS22HH_ODR_100_HZ,
    LPS22HH_ODR_200_HZ
} lps22hh_odr;


/**
 * @brief
 * Allowed values to configure the FIFO mode selection
*/
typedef enum{
    LPS22HH_FIFO_BYPASS,
    LPS22HH_FIFO_MODE,
    LPS22HH_FIFO_STREAM_MODE,
    LPS22HH_FIFO_STREAM_TO_FIFO,
    LPS22HH_FIFO_BYPASS_TO_STREAM,
    LPS22HH_FIFO_RESERVED,
    LPS22HH_FIFO_DYNAMIC_STREAM,
    LPS22HH_FIFO_BYPASS_TO_FIFO
} lps22hh_fifo;


/**
 * @brief
 * Allowed values to configure the Block Data Update
*/
typedef enum{
    LPS22HH_BDU_CONTINUOUS_UPDATE,
    LPS22HH_BDU_SYNCHRONOUS_UPDATE
} lps22hh_bdu;


/**
 * @brief
 * Allowed values to configure the low pass filter
*/
typedef enum{
    LPS22HH_LPFP_ODR_BY_9,
    LPS22HH_LPFP_ODR_BY_20
} lps22hh_lpfp;
 

/**
 * @brief
 * 
 * Class to store state and function to interact
 * with the LPS22 Pressure & Temperature Sensor.
*/
class LPS22HH_driver {
public:
    LPS22HH_driver();
    void init(SPIClass *theSpi, int8_t csPin);

    float getPressure();           ///< absolute pressure in hPa
    float getTemperature();        ///< temperature in Celsius
    int32_t getRawPressure();    ///< raw absolute pressure
    int16_t getRawTemperature(); ///< raw temperature

    void setSpi(SPIClass *theSpi);  ///< configure SPI
    void setCsPin(int8_t csPin); ///< configure CS Pin
    void setDataRate(lps22hh_odr dataRate); ///< configure ODR
    lps22hh_odr getDataRate(void); ///< retrieve the ODR
    void setFifoMode(lps22hh_fifo fifoMode); ///< configure FIFO
    void setBlockDataUpdate(lps22hh_bdu bduMode); ///< configure BDU

    void setLowPassFilter(bool filterEnabled); ///< configure LP filter
    void setFilterBandwidth(lps22hh_lpfp filterBandwidth); ///< configure LPFP

    void setLowNoise(bool lowNoiseEnabled); ///< configure the Low Noise

    uint8_t getDeviceId(void);       ///< Get the ID of the sensor
    void triggerNewMeasurement(void);  ///< Trigger a single measurement
    void powerDown(void);     ///< Put the device in standby
    void reset(void);         ///< Software reset
    bool hasNewPressure(void);  ///< New measurement done
    uint8_t getStatus(void);    ///< Get status register


private:
    int8_t _csPin;
    SPIClass *_spi;

    void beginTransaction(void);
    void endTransaction(void);

    uint8_t readSingleRegister(uint8_t reg);
    void readMultiRegister(uint8_t *buffer, uint8_t reg, uint8_t numRegs);

    void writeSingleRegister(uint8_t reg, uint8_t value);
    void writeMultiRegister(uint8_t reg, uint8_t numRegs, uint32_t value);

    bool readSingleBit(uint8_t reg, uint8_t position);
    uint8_t readMultiBits(uint8_t reg, uint8_t position, uint8_t numBits);

    void writeSingleBit(uint8_t reg, uint8_t position, bool value);
    void writeMultiBits(uint8_t reg, uint8_t position, uint8_t numBits, uint8_t value);
};

#endif