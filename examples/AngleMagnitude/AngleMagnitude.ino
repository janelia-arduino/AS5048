#include <SPI.h>
#include <Streaming.h>
#include <AS5048.h>


const long BAUD = 115200;
const int LOOP_DELAY = 100;
const int CHIP_SELECT_PIN = 10;

// Instantiate AS5048
AS5048 as5048;
AS5048::Error error;
uint16_t magnitude;
uint16_t angle;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUD);

  as5048.setup(CHIP_SELECT_PIN);
}

void loop()
{
  angle = as5048.getAngle();
  if (!as5048.transmissionError())
  {
    Serial << "angle = " << angle << "\n";
    uint8_t percent = (100 * angle) / (as5048.getAngleMax() - as5048.getAngleMin());
    Serial << "percent = " << percent << "\n";
  }
  else
  {
    Serial << "transmission error!\n";
  }

  magnitude = as5048.getMagnitude();
  if (!as5048.transmissionError())
  {
    Serial << "magnitude = " << magnitude << "\n";
  }
  else
  {
    Serial << "transmission error!\n";
  }

  delay(LOOP_DELAY);
}
