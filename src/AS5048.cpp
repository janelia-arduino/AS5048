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

  pinMode(chip_select_pin_,OUTPUT);
  disableChipSelect();

  SPI.begin();
}

uint16_t AS5048::getDiagnostics()
{
  // DiagnosticsGainControl diag_gain_control;
  // diag_gain_control.uint16 = readRegister(ADDRESS_DIAG_AGC);
  // return diag_gain_control.fields.ocf;
  uint16_t diagnostics = readRegister(ADDRESS_DIAG_AGC);
  return diagnostics;
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
  mosi_datagram.fields.address_data = address;
  mosi_datagram.fields.rw = RW_READ;
  mosi_datagram.fields.par = calculateEvenParityBit(address);
  writeRead(mosi_datagram);
  MisoDatagram miso_datagram = writeRead(mosi_datagram);
  return miso_datagram.fields.data;
}

void AS5048::writeRegister(uint16_t address,
  uint16_t data)
{
  // MosiDatagram mosi_datagram;
  // mosi_datagram.fields.address = address;
  // mosi_datagram.fields.rw = RW_WRITE;
  // mosi_datagram.fields.data = data;
  // writeRead(mosi_datagram);
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
