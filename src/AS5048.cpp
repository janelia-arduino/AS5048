// ----------------------------------------------------------------------------
// AS5048.cpp
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// ----------------------------------------------------------------------------
#include "AS5048.h"


void AS5048::setup(size_t chip_select_pin)
{
  chip_select_pin_ = chip_select_pin;
  transmission_error_ = false;

  pinMode(chip_select_pin_,OUTPUT);
  disableChipSelect();

  SPI.begin();
}

uint16_t AS5048::getAngle()
{
  return readRegister(ADDRESS_ANGLE);
}

uint16_t AS5048::getMagnitude()
{
  return readRegister(ADDRESS_MAGNITUDE);
}

bool AS5048::transmissionError()
{
  return transmission_error_;
}

AS5048::Error AS5048::getError()
{
  return error_;
}

AS5048::Diagnostics AS5048::getDiagnostics()
{
  DiagnosticsGainControl diag_gain_control;
  diag_gain_control.uint16 = readRegister(ADDRESS_DIAG_AGC);
  if (transmissionError())
  {
    clearError();
  }
  return diag_gain_control.fields.diagnostics;
}

// private
void AS5048::enableChipSelect()
{
  digitalWrite(chip_select_pin_,LOW);
}

void AS5048::disableChipSelect()
{
  digitalWrite(chip_select_pin_,HIGH);
}
void AS5048::spiBeginTransaction()
{
  enableChipSelect();
  delayMicroseconds(1);
  SPI.beginTransaction(SPISettings(SPI_CLOCK,SPI_BIT_ORDER,SPI_MODE));
}

void AS5048::spiEndTransaction()
{
  SPI.endTransaction();
  delayMicroseconds(1);
  disableChipSelect();
}

uint16_t AS5048::readRegister(uint16_t address)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.uint16 = 0;
  mosi_datagram.fields.address_data = address;
  mosi_datagram.fields.rw = RW_READ;
  mosi_datagram.fields.par = calculateEvenParityBit(mosi_datagram.uint16);
  // for a single read command two transmission sequences are necessary
  writeRead(mosi_datagram);
  MisoDatagram miso_datagram = writeRead(mosi_datagram);
  return miso_datagram.fields.data;
}

void AS5048::writeRegister(uint16_t address,
  uint16_t data)
{
  MosiDatagram mosi_datagram;

  // write address
  mosi_datagram.uint16 = 0;
  mosi_datagram.fields.address_data = address;
  mosi_datagram.fields.rw = RW_WRITE;
  mosi_datagram.fields.par = calculateEvenParityBit(mosi_datagram.uint16);
  writeRead(mosi_datagram);

  // write data
  mosi_datagram.uint16 = 0;
  mosi_datagram.fields.address_data = data;
  mosi_datagram.fields.rw = RW_WRITE;
  mosi_datagram.fields.par = calculateEvenParityBit(mosi_datagram.uint16);
  writeRead(mosi_datagram);
}

AS5048::MisoDatagram AS5048::writeRead(MosiDatagram mosi_datagram)
{
  MisoDatagram miso_datagram;
  miso_datagram.uint16 = 0x0;
  spiBeginTransaction();
  for (int i=(DATAGRAM_SIZE - 1); i>=0; --i)
  {
    uint8_t byte_write = (mosi_datagram.uint16 >> (8*i)) & 0xff;
    uint8_t byte_read = SPI.transfer(byte_write);
    miso_datagram.uint16 |= ((uint16_t)byte_read) << (8*i);
  }
  spiEndTransaction();
  noInterrupts();
  if (miso_datagram.fields.ef)
  {
    transmission_error_ = true;
  }
  else
  {
    transmission_error_ = false;
    error_.framing_error = 0;
    error_.command_invalid = 0;
    error_.parity_error = 0;
  }
  interrupts();
  return miso_datagram;
}

uint8_t AS5048::calculateEvenParityBit(uint16_t value)
{
  uint8_t operand_compare = bitRead(value,0);
  uint8_t i = 1;
  do
  {
    operand_compare ^= bitRead(value,i);
  }
  while ((i++) < ADDRESS_DATA_BIT_COUNT);
  return operand_compare & 0x1;
}

void AS5048::clearError()
{
  ErrorFlag error_flag;
  error_flag.uint16 = readRegister(ADDRESS_CLEAR_ERROR_FLAG);
  error_ = error_flag.fields.error;
}
