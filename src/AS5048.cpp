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

  angle_previous_ = getAngle();
  position_ = 0;
  invert_position_direction_ = false;
}

uint16_t AS5048::getAngle(uint8_t samples_per_average)
{
  return readRegisterAndAverage(ADDRESS_ANGLE,samples_per_average);
}

int32_t AS5048::getPosition(uint8_t samples_per_average)
{
  int32_t angle = getAngle(samples_per_average);
  int32_t angle_change;
  int32_t angle_change_a = angle - angle_previous_;
  int32_t angle_change_b;
  if (angle < angle_previous_)
  {
    angle_change_b = ANGLE_MAX + angle - angle_previous_;
  }
  else
  {
    angle_change_b = angle - ANGLE_MAX - angle_previous_;
  }
  if (abs(angle_change_a) < abs(angle_change_b))
  {
    angle_change = angle_change_a;
  }
  else
  {
    angle_change = angle_change_b;
  }

  if (invert_position_direction_)
  {
    angle_change *= -1;
  }

  position_ += angle_change;
  angle_previous_ = angle;
  return position_;
}

void AS5048::setPosition(int32_t position)
{
  noInterrupts();
  position_ = position;
  interrupts();
}

void AS5048::setPositionDirectionInverted()
{
  invert_position_direction_ = true;
}

void AS5048::setPositionDirectionNormal()
{
  invert_position_direction_ = false;
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

uint16_t AS5048::readRegisterAndAverage(uint16_t address,
  uint8_t samples_per_average)
{
  samples_per_average = constrain(samples_per_average,
    SAMPLES_PER_AVERAGE_MIN,
    SAMPLES_PER_AVERAGE_MAX);
  MosiDatagram mosi_datagram;
  mosi_datagram.uint16 = 0;
  mosi_datagram.fields.address_data = address;
  mosi_datagram.fields.rw = RW_READ;
  mosi_datagram.fields.par = calculateEvenParityBit(mosi_datagram.uint16);
  // for a single read command two transmission sequences are necessary
  writeRead(mosi_datagram);
  MisoDatagram miso_datagram;
  uint32_t value = 0;
  for (uint8_t i=0; i<samples_per_average; ++i)
  {
    miso_datagram = writeRead(mosi_datagram);
    value += miso_datagram.fields.data;
  }
  return value / samples_per_average;
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
