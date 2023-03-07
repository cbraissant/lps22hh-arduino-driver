#include <Arduino.h>
#include <SPI.h>
#include <LPS22.h>
#include <unity.h>

#define CS_PIN 10
LPS22 sensor;


void setUp(void)
{
}


void tearDown(void)
{
  // clean stuff up here
}


void test_set_cs_pin(void)
{
  sensor.setCsPin(CS_PIN);
  TEST_ASSERT_EQUAL(HIGH, digitalRead(CS_PIN));
}


// *********************************************************
// Read register
// *********************************************************
                                                         
void test_read_single_register(void)
{
  TEST_ASSERT_EQUAL(0xB3, sensor.readSingleRegister(LPS_WHO_AM_I));
}


void test_read_single_bit(void)
{
  TEST_ASSERT_EQUAL(1, sensor.readSingleBit(LPS_WHO_AM_I,7));
  TEST_ASSERT_EQUAL(0, sensor.readSingleBit(LPS_WHO_AM_I,6));
  TEST_ASSERT_EQUAL(1, sensor.readSingleBit(LPS_WHO_AM_I,5));
  TEST_ASSERT_EQUAL(1, sensor.readSingleBit(LPS_WHO_AM_I,4));
  TEST_ASSERT_EQUAL(0, sensor.readSingleBit(LPS_WHO_AM_I,3));
  TEST_ASSERT_EQUAL(0, sensor.readSingleBit(LPS_WHO_AM_I,2));
  TEST_ASSERT_EQUAL(1, sensor.readSingleBit(LPS_WHO_AM_I,1));
  TEST_ASSERT_EQUAL(1, sensor.readSingleBit(LPS_WHO_AM_I,0));
}


void test_read_multibits(void)
{
  // The value of the Who Am I register is 1011 0001
  TEST_ASSERT_EQUAL(0B011, sensor.readMultiBits(LPS_WHO_AM_I,4,3));
  TEST_ASSERT_EQUAL(0B101100, sensor.readMultiBits(LPS_WHO_AM_I,2,6));
}


void test_read_default_registers(void)
{
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_INTERRUPT_CFG));
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_THS_P_L));
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_THS_P_H));
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_IF_CTRL));
  TEST_ASSERT_EQUAL(0xB3, sensor.readSingleRegister(LPS_WHO_AM_I));
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_CTRL_REG1));
  TEST_ASSERT_EQUAL(0x10, sensor.readSingleRegister(LPS_CTRL_REG2));
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_CTRL_REG3));
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_FIFO_CTRL));
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_FIFO_WTM));
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_REF_P_L));
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_REF_P_H));
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_RPDS_L));
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_RPDS_H));
}


void test_read_multi_registers(void)
{
  uint8_t buffer[4];
  sensor.readMultiRegister(buffer, LPS_WHO_AM_I, 4);
  // based on the assumption that the registers have their default value
  TEST_ASSERT_EQUAL(0xB3, buffer[0]); // WHO_AM_I
  TEST_ASSERT_EQUAL(0x00, buffer[1]); // CTRL_REG1
  TEST_ASSERT_EQUAL(0x10, buffer[2]); // CTRL_REG2
  TEST_ASSERT_EQUAL(0x00, buffer[3]); // CTRL_REG3
}

// *********************************************************
// Write register
// *********************************************************

void test_write_single_register(void)
{
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_THS_P_H));
  sensor.writeSingleRegister(LPS_THS_P_H, 0xD7);
  TEST_ASSERT_EQUAL(0xD7, sensor.readSingleRegister(LPS_THS_P_H));
}

void test_write_single_bit(void)
{
  sensor.writeSingleRegister(LPS_THS_P_H, 0x00);
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_THS_P_H));
  sensor.writeSingleBit(LPS_THS_P_H, 7, 1);
  TEST_ASSERT_EQUAL(0x80, sensor.readSingleRegister(LPS_THS_P_H));
  sensor.writeSingleBit(LPS_THS_P_H, 3, 1);
  TEST_ASSERT_EQUAL(0x88, sensor.readSingleRegister(LPS_THS_P_H));
  sensor.writeSingleBit(LPS_THS_P_H, 5, 1);
  TEST_ASSERT_EQUAL(0xA8, sensor.readSingleRegister(LPS_THS_P_H));
}

void test_write_multibits(void)
{
  // reset the testing conditions
  sensor.writeSingleRegister(LPS_THS_P_H, 0x00);
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_THS_P_H));

  sensor.writeMultiBits(LPS_THS_P_H, 2, 0B1101, 4);
  TEST_ASSERT_EQUAL(0B00110100, sensor.readSingleRegister(LPS_THS_P_H));

  sensor.writeMultiBits(LPS_THS_P_H, 1, 0B10001, 5);
  TEST_ASSERT_EQUAL(0B00100010, sensor.readSingleRegister(LPS_THS_P_H));
}


void test_write_multi_registers(void)
{
  TEST_ASSERT_EQUAL(0x00, sensor.readSingleRegister(LPS_REF_P_H));
}

// *********************************************************
// Utility functions
// *********************************************************

void test_reset_software(void)
{
  sensor.swreset();
  RUN_TEST(test_read_default_registers);
}


void test_set_data_rate(void)
{
  sensor.setDataRate(LPS_ODR_10_HZ);
  TEST_ASSERT_EQUAL(2, sensor.readMultiBits(LPS_CTRL_REG1, 4, 3));
}


// *********************************************************
// Setup
// *********************************************************

void setup()
{
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  delay(2000);
  Serial.begin(9600);
  SPI.begin();
  SPI.beginTransaction(SPISettings(LPS_SPI_CLOCK_FREQUENCY, MSBFIRST, SPI_MODE0));
 
  UNITY_BEGIN(); // IMPORTANT LINE!

  RUN_TEST(test_set_cs_pin);

  // read register, bit-by-bit and multibits of Who Am I
  RUN_TEST(test_read_single_register);
  RUN_TEST(test_read_single_bit);
  RUN_TEST(test_read_multibits);

  // write random value on a register
  RUN_TEST(test_write_single_register);
  RUN_TEST(test_write_single_bit);
  RUN_TEST(test_write_multibits);

  // it also reset all the values of the previous tests
  // and check that the registers have their default values
  RUN_TEST(test_reset_software);

  //
  RUN_TEST(test_read_multi_registers);

  // set the ODR
  RUN_TEST(test_set_data_rate); 
}

void loop()
{
  UNITY_END(); // stop unit testing
}