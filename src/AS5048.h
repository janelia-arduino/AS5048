// ----------------------------------------------------------------------------
// AS5048.h
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// ----------------------------------------------------------------------------
#ifndef AS5048_H
#define AS5048_H
#include <Arduino.h>
#include <SPI.h>

#include <Streaming.h>

class AS5048
{
public:
  void setup(size_t chip_select_pin);

  uint16_t getDiagnostics();

private:
  // SPISettings
  const static uint32_t SPI_CLOCK = 1000000;
  const static uint8_t SPI_BIT_ORDER = MSBFIRST;
  const static uint8_t SPI_MODE = SPI_MODE1;

  // Datagrams
  const static uint8_t DATAGRAM_SIZE = 2;

  const static uint8_t ADDRESS_DATA_BIT_COUNT = 14;

  // MOSI Datagrams
  union MosiDatagram
  {
    struct Fields
    {
      uint16_t address_data : ADDRESS_DATA_BIT_COUNT;
      uint16_t rw : 1;
      uint16_t par : 1;
    } fields;
    uint16_t uint16;
  };
  const static uint8_t RW_READ = 1;
  const static uint8_t RW_WRITE = 0;

  // MISO Datagrams
  union MisoDatagram
  {
    struct Fields
    {
      uint16_t data : ADDRESS_DATA_BIT_COUNT;
      uint16_t ef : 1;
      uint16_t par : 1;
    } fields;
    uint16_t uint16;
  };
  size_t chip_select_pin_;

  // Control and Error Registers
  const static uint16_t ADDRESS_NOP = 0x0000;
  const static uint16_t ADDRESS_CLEAR_ERROR_FLAG = 0x0001;
  const static uint16_t ADDRESS_PROGRAMMING_CONTROL = 0x0003;

  union ClearErrorFlag
  {
    struct Fields
    {
      uint16_t framing_error : 1;
      uint16_t command_invalid : 1;
      uint16_t parity_error : 1;
      uint16_t space : 13;
    } fields;
    uint16_t uint16;
  };
  union ProgrammingControl
  {
    struct Fields
    {
      uint16_t programming_enabled : 1;
      uint16_t space0 : 2;
      uint16_t burn : 1;
      uint16_t space1 : 2;
      uint16_t verify : 1;
      uint16_t space3 : 9;
    } fields;
    uint16_t uint16;
  };

  // Programmable Customer Settings
  const static uint16_t ADDRESS_ZERO_HIGH = 0x0016;
  const static uint16_t ADDRESS_ZERO_LOW = 0x0017;

  // Readout Registers
  const static uint16_t ADDRESS_DIAG_AGC = 0x3FFD;
  const static uint16_t ADDRESS_MAGNITUDE = 0x3FFE;
  const static uint16_t ADDRESS_ANGLE = 0x3FFF;

  union DiagnosticsGainControl
  {
    struct Fields
    {
      uint16_t agc_value : 8;
      uint16_t ocf : 1;
      uint16_t cof : 1;
      uint16_t comp_low : 1;
      uint16_t comp_high : 1;
      uint16_t space : 4;
    } fields;
    uint16_t uint16;
  };

  void enableChipSelect();
  void disableChipSelect();
  void spiBeginTransaction();
  void spiEndTransaction();

  uint16_t readRegister(uint16_t address);
  void writeRegister(uint16_t address,
    uint16_t data);
  MisoDatagram writeRead(MosiDatagram mosi_datagram);
  uint8_t calculateEvenParityBit(uint16_t value);

};

#endif
