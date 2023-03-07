/*!
 * @file LPS22.h
 *
 * Driver for the ST LPS22 pressure sensor
 */

#ifndef _LPS22
#define _LPS22

#include <Arduino.h>
#include <Wire.h>


/** Registers definition
 */
#define LPS_INTERRUPT_CFG 0x0B  ///< Interrupt mode register
#define LPS_THS_P_L 0x0C        ///< Pressure threshold registers (LSB)
#define LPS_THS_P_H 0x0D        ///< Pressure threshold registers (MSB)
#define LPS_IF_CTRL 0x0E        ///< Interface control register
#define LPS_WHO_AM_I 0x0F       ///< Who am I

#define LPS_CTRL_REG1 0x10      ///< Control register (LSB)
#define LPS_CTRL_REG2 0x11      ///< Control register
#define LPS_CTRL_REG3 0x12      ///< Control register (MSB)

/* WARNING
 * These registers have a different address
 * depending on the chip used (HB, HH, DF, ...)
 */
#define LPS_FIFO_CTRL 0x13      ///< FIFO configuration register
#define LPS_FIFO_WTM 0x14       ///< FIFO Threshold setting register
#define LPS_REF_P_L 0x15        ///< Reference pressure register (LSB)
#define LPS_REF_P_H 0x16        ///< Reference pressure register (MSB)

#define LPS_RPDS_L 0x18         ///< Pressure offset register (LSB)
#define LPS_RPDS_H 0x19         ///< Pressure offset register (MSB)

#define LPS_INT_SOURCE 0x24     ///< Interrupt register
#define LPS_FIFO_STATUS1 0x25   ///< FIFO status register
#define LPS_FIFO_STATUS2 0x26   ///< FIFO status register
/* END WARNING */

#define LPS_STATUS 0x27         ///< Status register

#define LPS_PRESSURE_OUT_XL 0x28    ///< Pressure output register (LSB)
#define LPS_PRESSURE_OUT_L 0x29     ///< Pressure output register
#define LPS_PRESSURE_OUT_H 0x2A     ///< Pressure output register (MSB)
#define LPS_TEMP_OUT_L 0x2B         ///< Temperature output register (LSB)
#define LPS_TEMP_OUT_H 0x2C         ///< Temperature output register (LSB)

#define LPS_FIFO_DATA_OUT_PRESS_XL 0x78 ///< FIFO pressure output register
#define LPS_FIFO_DATA_OUT_PRESS_L 0x79  ///< FIFO pressure output register
#define LPS_FIFO_DATA_OUT_PRESS_H 0x7A  ///< FIFO pressure output register
#define LPS_FIFO_DATA_OUT_TEMP_L 0x7B   ///< FIFO temperature output registers
#define LPS_FIFO_DATA_OUT_TEMP_H 0x7C   ///< FIFO temperature output registers

/** Characteristics
*/
#define LPS_MIN_PRESSURE 260    // hPa
#define LPS_MAX_PRESSURE 1260   // hPa
#define LPS_PRESSURE_SENSITIVITY 4096       // 4096 LSB = 1 hPa
#define LPS_PRESSURE_RESOLUTION 0.00024414  // 1 LSB = 1/4096 = 0.0002441406 hPa
#define LPS_TEMPERATURE_SENSITIVITY 100     // 100 LSB = °C
#define LPS_TEMPERATURE_RESOLUTION  0.01    // 1 LSB = 1/100 = 0.01 °C
#define LPS_SPI_CLOCK_FREQUENCY 100000      // 100 kHz

/**
 * @brief
 * 
 * Allowed values to configure the Output data rate bits
*/
typedef enum{
    LPS_ODR_ONE_SHOT,
    LPS_ODR_1_HZ,
    LPS_ODR_10_HZ,
    LPS_ODR_25_HZ,
    LPS_ODR_50_HZ,
    LPS_ODR_75_HZ,
    LPS_ODR_100_HZ,
    LPS_ODR_200_HZ
} lps_odr;

/**
 * @brief
 * 
 * Allowed values to configure the FIFO mode selection
*/
typedef enum{
    LPS_FIFO_BYPASS,
    LPS_FIFO_MODE,
    LPS_FIFO_STREAM_MODE,
    LPS_FIFO_STREAM_TO_FIFO,
    LPS_FIFO_BYPASS_TO_STREAM,
    LPS_FIFO_RESERVED,
    LPS_FIFO_DYNAMIC_STREAM,
    LPS_FIFO_BYPASS_TO_FIFO
} lps_fifo;


/**
 * @brief
 * 
 * Class to store state and function to interact
 * with the LPS22 Pressure & Temperature Sensor.
*/
class LPS22 {
public:
    LPS22();

    float getPressure();           ///< absolute pressure in hPa
    float getTemperature();        ///< temperature in Celsius
    int32_t getPressureValue();    ///< raw absolute pressure
    int16_t getTemperatureValue(); ///< raw temperature

    void setCsPin(int8_t cs_pin); ///< configure CS Pin
    void setDataRate(lps_odr data_rate); ///< configure ODR
    void setFifoMode(lps_fifo fifo_mode); ///< configure FIFO
    
    uint8_t whoAmI(void);       ///< Get the ID of the sensor
    void triggerOneShot(void);  ///< Trigger a single measurement
    void swreset(void);         ///< Software reset
    bool hasNewPressure(void);  ///< New measurement done
    uint8_t getStatus(void);    ///< Get status register
    
    uint8_t readSingleRegister(uint8_t reg);
    void readMultiRegister(uint8_t *buffer, uint8_t reg, uint8_t numRegs);

    void writeSingleRegister(uint8_t reg, uint8_t value);
    bool writeMultiRegister(uint8_t reg, uint32_t value, uint8_t numRegs);

    bool readSingleBit(uint8_t reg, uint8_t position);
    uint8_t readMultiBits(uint8_t reg, uint8_t position, uint8_t numBits);

    void writeSingleBit(uint8_t reg, uint8_t position, bool value);
    void writeMultiBits(uint8_t reg, uint8_t position, uint8_t value, uint8_t numBits);

    bool _isWorking = false;

private:
    void enableSPI(void);
    void disableSPI(void);
    int8_t _cs_pin;
};

#endif