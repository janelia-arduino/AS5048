#include <SPI.h>
#include <Streaming.h>
#include <AS5048.h>


const long BAUD = 115200;
const int LOOP_DELAY = 2000;
const int CHIP_SELECT_PIN = 15;
const int CLOCK_FREQUENCY_MHZ = 32;

// Instantiate AS5048
AS5048 as5048;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUD);

  as5048.setup(CHIP_SELECT_PIN,CLOCK_FREQUENCY_MHZ);

  as5048.initialize();

}

void loop()
{
  bool communicating = as5048.communicating();
  Serial << "communicating: " << communicating << "\n\n";
  delay(LOOP_DELAY);
}
