#include <SPI.h>
#include <Streaming.h>
#include <AS5048.h>


const long BAUD = 115200;
const int LOOP_DELAY = 500;
const int CHIP_SELECT_PIN = 10;

// Instantiate AS5048
AS5048 as5048;
AS5048::Error error;

void printError(AS5048::Error error)
{
  if (error.framing_error)
  {
    Serial << "framing_error!\n\n";
  }
  if (error.command_invalid)
  {
    Serial << "command_invalid!\n\n";
  }
  if (error.parity_error)
  {
    Serial << "parity_error!\n\n";
  }
}

void printDiagnostics(AS5048::Diagnostics diagnostics)
{
  Serial << "diagnostics:\n";
  Serial << "automatic_gain_control = " << diagnostics.automatic_gain_control << "\n";
  Serial << "offset_compensation_finished = " << diagnostics.offset_compensation_finished << "\n";
  Serial << "cordic_overflow = " << diagnostics.cordic_overflow << "\n";
  Serial << "weak_magnetic_field = " << diagnostics.weak_magnetic_field << "\n";
  Serial << "strong_magnetic_field = " << diagnostics.strong_magnetic_field << "\n";
  Serial << "\n";
}

void setup()
{
  // Setup serial communications
  Serial.begin(BAUD);

  as5048.setup(CHIP_SELECT_PIN);
}

void loop()
{
  AS5048::Diagnostics diagnostics = as5048.getDiagnostics();
  if (!as5048.transmissionError())
  {
    printDiagnostics(diagnostics);
  }
  else
  {
    Serial << "transmision error!\n";
    AS5048::Error error = as5048.getError();
    printError(error);
  }
  delay(LOOP_DELAY);
}
