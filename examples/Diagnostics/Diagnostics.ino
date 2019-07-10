#include <SPI.h>
#include <Streaming.h>
#include <AS5048.h>


const long BAUD = 115200;
const int LOOP_DELAY = 2000;
const int CHIP_SELECT_PIN = 10;

// Instantiate AS5048
AS5048 as5048;

void setup()
{
  // Setup serial communications
  Serial.begin(BAUD);

  as5048.setup(CHIP_SELECT_PIN);
}

void loop()
{
  uint16_t diagnostics = as5048.getDiagnostics();
  Serial << "diagnostics: " << _BIN(diagnostics) << "\n\n";
  delay(LOOP_DELAY);
}
