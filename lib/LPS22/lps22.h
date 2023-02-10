/*!
 * @file LPS22.h
 *
 * Driver for the ST LPS22 pressure sensor
 */

#ifndef _LPS22_H
#define _LPS22_H

/** Registers definition
 */
#define LPS_INTERRUPT_CFG 0x0B  ///< Interrupt mode register
#define LPS_THS_P_L 0x0C ///< Pressure threshold registers (LSB)
#define LPS_THS_P_H 0x0D ///< Pressure threshold registers (MSB)
#define LPS_IF_CTRL 0x0E ///< Interface control register
#define LPS_WHO_AM_I 0x0F ///< Who am I

#define LPS_CTRL_REG1 0x10 ///< Control register (LSB)
#define LPS_CTRL_REG2 0x11 ///< Control register
#define LPS_CTRL_REG3 0x12 ///< Control register (MSB)

#define LPS_FIFO_CTRL 0x13 ///< FIFO configuration register
#define LPS_FIFO_WTM 0x14 ///< FIFO Threshold setting register
#define LPS_REF_P_L 0x15 ///< Reference pressure register (LSB)
#define LPS_REF_P_H 0x16 ///< Reference pressure register (MSB)
#define LPS_RPDS_L 0x18 ///< Pressure offset register (LSB)
#define LPS_RPDS_H 0x19 ///< Pressure offset register (MSB)

#define LPS_INT_SOURCE 0x24 ///< Interrupt register
#define LPS_FIFO_STATUS1 0x25 ///< FIFO status register
#define LPS_FIFO_STATUS2 0x26 ///< FIFO status register
#define LPS_STATUS 0x27 ///< Status register

#define LPS_PRESSURE_OUT_XL 0x28  ///< Pressure output register (LSB)
#define LPS_PRESSURE_OUT_L 0x29   ///< Pressure output register
#define LPS_PRESSURE_OUT_H 0x2A   ///< Pressure output register (MSB)
#define LPS_TEMP_OUT_L 0x2B   ///< Temperature output register (LSB)
#define LPS_TEMP_OUT_H 0x2C   ///< Temperature output register (LSB)

#define LPS_FIFO_DATA_OUT_PRESS_XL 0x78 ///< FIFO pressure output register
#define LPS_FIFO_DATA_OUT_PRESS_L 0x79  ///< FIFO pressure output register
#define LPS_FIFO_DATA_OUT_PRESS_H 0x7A  ///< FIFO pressure output register
#define FIFO_DATA_OUT_TEMP_L 0x7B // FIFO temperature output registers
#define FIFO_DATA_OUT_TEMP_H 0x7C // FIFO temperature output registers
