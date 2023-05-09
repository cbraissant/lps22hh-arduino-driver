#include <SPI.h>
#include <lps22hh_driver.h>

#define ARDUINO_CS_PIN 10
#define SPI_CLOCK_FREQUENCY 2000000

LPS22HH_driver lps22;
SPIClass theSpi; 

void setup() {
  Serial.begin(9600);

  theSpi.begin();
  theSpi.beginTransaction(SPISettings(SPI_CLOCK_FREQUENCY, MSBFIRST, SPI_MODE0));

  lps22.init(&theSpi, ARDUINO_CS_PIN);

  lps22.reset();

  // The offset pressure (RPDS) is non-volatile and need to be manually reset.
  lps22.setPressureOffset(0);

  // The ODR is set to "one-shot" by default,
  // and need to be configured to get some readings
  lps22.setDataRate(LPS22HH_ODR_200_HZ);

  // Set the Block Data Update (BDU) to 1:
  // output registers not updated until MSB and LSB have been read
  lps22.setBlockDataUpdate(LPS22HH_BDU_SYNCHRONOUS_UPDATE);  
};


void loop() {
  if (lps22.hasNewPressure()){
    Serial.println(lps22.getPressure());
  }
}
