#include <SPI.h>
#include <Streaming.h>
#include <AS5048.h>


const long BAUD = 115200;
const int LOOP_DELAY = 100;
const int CHIP_SELECT_PIN = 10;
const int SAMPLES_IN_AVERAGE = 64;

// Instantiate AS5048
AS5048 as5048;
AS5048::Error error;
int32_t position = 0;
double rotations = 0.0;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUD);

  as5048.setup(CHIP_SELECT_PIN);

  as5048.setPosition(position);
}

void loop()
{
  position = as5048.getPosition(SAMPLES_IN_AVERAGE);
  if (!as5048.transmissionError())
  {
    Serial << "position = " << position << "\n";
    rotations = (double)position / (double)AS5048::POSITIONS_PER_REVOLUTION;
    Serial << "rotations = " << rotations << "\n";
  }
  else
  {
    Serial << "transmission error!\n";
  }

  delay(LOOP_DELAY);
}
