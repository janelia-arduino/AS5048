// ----------------------------------------------------------------------------
// AS5048.cpp
//
// Authors:
// Peter Polidoro peterpolidoro@gmail.com
// ----------------------------------------------------------------------------
#include "AS5048.h"


void AS5048::setup(size_t chip_select_pin,
  uint8_t clock_frequency_mhz)
{
  chip_select_pin_ = chip_select_pin;

  pinMode(chip_select_pin_,OUTPUT);
  digitalWrite(chip_select_pin_,HIGH);

  specifyClockFrequencyInMHz(clock_frequency_mhz);

  for (uint8_t motor=0; motor<MOTOR_COUNT; ++motor)
  {
    pulse_div_[motor] = 0;
    ramp_div_[motor] = 0;
  }

  SPI.begin();

  setStepDiv(STEP_DIV_MAX);

  stopAll();
}

bool AS5048::communicating()
{
  return (getVersion() == VERSION);
}


// private
uint32_t AS5048::readRegister(uint8_t smda,
  uint8_t address)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.fields.rrs = RRS_REGISTER;
  mosi_datagram.fields.address = address;
  mosi_datagram.fields.smda = smda;
  mosi_datagram.fields.rw = RW_READ;
  mosi_datagram.fields.data = 0;
  MisoDatagram miso_datagram = writeRead(mosi_datagram);
  return miso_datagram.fields.data;
}

void AS5048::writeRegister(uint8_t smda,
  uint8_t address,
  uint32_t data)
{
  MosiDatagram mosi_datagram;
  mosi_datagram.fields.rrs = RRS_REGISTER;
  mosi_datagram.fields.address = address;
  mosi_datagram.fields.smda = smda;
  mosi_datagram.fields.rw = RW_WRITE;
  mosi_datagram.fields.data = data;
  writeRead(mosi_datagram);
}

AS5048::MisoDatagram AS5048::writeRead(MosiDatagram mosi_datagram)
{
  MisoDatagram miso_datagram;
  miso_datagram.uint32 = 0x0;
  spiBeginTransaction();
  for (int i=(DATAGRAM_SIZE - 1); i>=0; --i)
  {
    uint8_t byte_write = (mosi_datagram.uint32 >> (8*i)) & 0xff;
    uint8_t byte_read = SPI.transfer(byte_write);
    miso_datagram.uint32 |= ((uint32_t)byte_read) << (8*i);
  }
  spiEndTransaction();
  noInterrupts();
  status_ = miso_datagram.fields.status;
  interrupts();
  return miso_datagram;
}

int32_t AS5048::unsignedToSigned(uint32_t input_value,
  uint8_t num_bits)
{
  uint32_t mask = 1 << (num_bits - 1);
  return -(input_value & mask) + (input_value & ~mask);
}

void AS5048::specifyClockFrequencyInMHz(uint8_t clock_frequency)
{
  if (clock_frequency <= CLOCK_FREQUENCY_MAX)
  {
    clock_frequency_ = clock_frequency;
  }
  else
  {
    clock_frequency_ = CLOCK_FREQUENCY_MAX;
  }
}

void AS5048::setOptimalStepDivHz(uint32_t velocity_max_hz)
{
  int step_div = getStepDiv();

  double step_time = stepDivToStepTime(step_div);

  uint32_t velocity_max_upper_limit = (double)MHZ_PER_HZ/(step_time*2);

  while ((velocity_max_upper_limit < velocity_max_hz) && (step_div >= 1))
  {
    --step_div;
    step_time = stepDivToStepTime(step_div);
    velocity_max_upper_limit = (double)MHZ_PER_HZ/(step_time*2);
  }

  setStepDiv(step_div);
}

uint8_t AS5048::getStepDiv()
{
  GlobalParameters global_parameters;
  global_parameters.uint32 = readRegister(SMDA_COMMON,ADDRESS_GLOBAL_PARAMETERS);
  return global_parameters.fields.clk2_div & STEP_DIV_MASK;
}

void AS5048::setStepDiv(uint8_t step_div)
{
  GlobalParameters global_parameters;
  global_parameters.uint32 = readRegister(SMDA_COMMON,ADDRESS_GLOBAL_PARAMETERS);
  global_parameters.fields.clk2_div = step_div & STEP_DIV_MASK;
  writeRegister(SMDA_COMMON,ADDRESS_GLOBAL_PARAMETERS,global_parameters.uint32);
}

double AS5048::stepDivToStepTime(uint8_t step_div)
{
  double step_time = (double)(16*(1 + step_div))/(double)clock_frequency_;
  return step_time;
}

int32_t AS5048::convertVelocityToHz(uint8_t pulse_div,
  int16_t velocity)
{
  // (clock_frequency_*MHZ_PER_HZ*velocity)/((1 << pulse_div)*VELOCITY_CONSTANT);
  double x = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)VELOCITY_CONSTANT;
  double y = (x*(double)velocity)/((double)(1 << pulse_div));
  return y;
}

int16_t AS5048::convertVelocityFromHz(uint8_t pulse_div,
  int32_t velocity)
{
  // (velocity*(1 << pulse_div)*VELOCITY_CONSTANT)/(clock_frequency_*MHZ_PER_HZ);
  double x = ((double)velocity*(double)(1 << pulse_div))/((double)clock_frequency_*(double)MHZ_PER_HZ);
  double y = x*(double)VELOCITY_CONSTANT;
  return y;
}

uint8_t AS5048::findOptimalPulseDivHz(uint32_t velocity_max_hz)
{
  uint8_t pulse_div = PULSE_DIV_MAX + 1;
  uint32_t velocity_max_upper_limit = 0;
  while ((velocity_max_upper_limit < velocity_max_hz) && (pulse_div >= 1))
  {
    --pulse_div;
    velocity_max_upper_limit = getVelocityMaxUpperLimitInHz(pulse_div);
  }
  return pulse_div;
}

void AS5048::setOptimalPulseDivHz(size_t motor,
  uint32_t velocity_max_hz)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  ClkConfig clk_config;
  clk_config.uint32 = readRegister(motor,ADDRESS_CLOCK_CONFIGURATION);
  clk_config.fields.clk_config.pulse_div = pulse_div;
  writeRegister(motor,ADDRESS_CLOCK_CONFIGURATION,clk_config.uint32);
  pulse_div_[motor] = pulse_div;
}

AS5048::Mode AS5048::getMode(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return RAMP_MODE;
  }

  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  switch (ref_conf_mode.fields.mode)
  {
    case RAMP_MODE:
      return RAMP_MODE;
      break;
    case SOFT_MODE:
      return SOFT_MODE;
      break;
    case VELOCITY_MODE:
      return VELOCITY_MODE;
      break;
    case HOLD_MODE:
      return HOLD_MODE;
      break;
  }
  return RAMP_MODE;
}

void AS5048::setMode(size_t motor,
  Mode mode)
{
  if (motor >= MOTOR_COUNT)
  {
    return;
  }

  RefConfMode ref_conf_mode;
  ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  ref_conf_mode.fields.mode = (uint8_t)mode;
  writeRegister(motor,ADDRESS_REF_CONF_MODE,ref_conf_mode.uint32);
}

AS5048::ReferenceConfiguration AS5048::getReferenceConfiguration(size_t motor)
{
  RefConfMode ref_conf_mode;
  if (motor < MOTOR_COUNT)
  {
    ref_conf_mode.uint32 = readRegister(motor,ADDRESS_REF_CONF_MODE);
  }
  return ref_conf_mode.fields.ref_conf;
}

AS5048::InterfaceConfiguration AS5048::getInterfaceConfiguration()
{
  IfConf if_conf;
  if_conf.uint32 = readRegister(SMDA_COMMON,ADDRESS_IF_CONFIGURATION_429);
  return if_conf.fields.if_conf;
}

AS5048::SwitchState AS5048::getSwitchState()
{
  SwState switch_state;
  switch_state.uint32 = readRegister(SMDA_COMMON,ADDRESS_SWITCHES);
  return switch_state.fields.switch_state;
}

AS5048::ClockConfiguration AS5048::getClockConfiguration(size_t motor)
{
  ClkConfig clk_config;
  if (motor < MOTOR_COUNT)
  {
    clk_config.uint32 = readRegister(motor,ADDRESS_CLOCK_CONFIGURATION);
  }
  return clk_config.fields.clk_config;
}

double AS5048::getProportionalityFactor(size_t motor)
{
  if (motor >= MOTOR_COUNT)
  {
    return 0.0;
  }
  PropFactor prop_factor;
  prop_factor.uint32 = readRegister(motor,ADDRESS_PROP_FACTOR);
  int pm = prop_factor.fields.pmul;
  int pd = prop_factor.fields.pdiv;
  return ((double)(pm)) / ((double)(1 << (pd + 3)));
}

double AS5048::getStepTimeInMicroS()
{
  uint8_t step_div = getStepDiv();
  return stepDivToStepTime(step_div);
}

uint16_t AS5048::getVelocityMin(size_t motor)
{
  return readRegister(motor,ADDRESS_V_MIN);
}

void AS5048::setVelocityMinInHz(size_t motor,
  uint32_t velocity_min_hz)
{
  uint32_t velocity_min = convertVelocityFromHz(pulse_div_[motor],velocity_min_hz);
  if (velocity_min < VELOCITY_MIN_MIN)
  {
    velocity_min = VELOCITY_MIN_MIN;
  }
  writeRegister(motor,ADDRESS_V_MIN,velocity_min);
}

uint16_t AS5048::getVelocityMax(size_t motor)
{
  return readRegister(motor,ADDRESS_V_MAX);
}

void AS5048::setVelocityMaxInHz(size_t motor,
  uint32_t velocity_max_hz)
{
  uint32_t velocity_max = convertVelocityFromHz(pulse_div_[motor],velocity_max_hz);
  uint32_t velocity_max_upper_limit = getVelocityMaxUpperLimitInHz();
  if (velocity_max > velocity_max_upper_limit)
  {
    velocity_max = velocity_max_upper_limit;
  }
  writeRegister(motor,ADDRESS_V_MAX,velocity_max);
}

int16_t AS5048::getTargetVelocity(size_t motor)
{
  uint32_t velocity_unsigned = readRegister(motor,ADDRESS_V_TARGET);
  return unsignedToSigned(velocity_unsigned,V_BIT_COUNT);
}

void AS5048::setTargetVelocity(size_t motor,
  int16_t velocity)
{
  writeRegister(motor,ADDRESS_V_TARGET,velocity);
}

int16_t AS5048::getActualVelocity(size_t motor)
{
  uint32_t velocity_unsigned = readRegister(motor,ADDRESS_V_ACTUAL);
  return unsignedToSigned(velocity_unsigned,V_BIT_COUNT);
}

uint32_t AS5048::convertAccelerationToHzPerS(uint8_t pulse_div,
  uint8_t ramp_div,
  uint32_t acceleration)
{
  // (clock_frequency_*MHZ_PER_HZ*clock_frequency_*MHZ_PER_HZ*acceleration)/((1 << pulse_div)*(1 << ramp_div)*ACCELERATION_CONSTANT);
  double a = ((double)clock_frequency_*(double)MHZ_PER_HZ)/(double)ACCELERATION_CONSTANT;
  double b = a*(double)clock_frequency_*(double)MHZ_PER_HZ;
  double c = b/((double)(1 << pulse_div));
  double d = c/((double)(1 << ramp_div));
  uint32_t e = round(d*(double)acceleration);
  return e;
}

uint32_t AS5048::convertAccelerationFromHzPerS(uint8_t pulse_div,
  uint8_t ramp_div,
  uint32_t acceleration)
{
  // (acceleration*(1 << pulse_div)*(1 << ramp_div)*ACCELERATION_CONSTANT)/(clock_frequency_*MHZ_PER_HZ*clock_frequency_*MHZ_PER_HZ);
  double a = ((double)acceleration*(double)(1 << pulse_div))/((double)clock_frequency_*(double)MHZ_PER_HZ);
  double b = a*(double)ACCELERATION_CONSTANT;
  double c = b/((double)clock_frequency_*(double)MHZ_PER_HZ);
  uint32_t d = round(c*(1 << ramp_div));
  return d;
}

uint8_t AS5048::findOptimalRampDivHz(uint32_t velocity_max_hz,
  uint32_t acceleration_max_hz_per_s)
{
  uint8_t pulse_div = findOptimalPulseDivHz(velocity_max_hz);
  uint8_t ramp_div = RAMP_DIV_MAX;
  uint32_t acceleration_max_upper_limit = getAccelerationMaxUpperLimitInHzPerS(pulse_div,ramp_div);;
  uint32_t acceleration_max_lower_limit = getAccelerationMaxLowerLimitInHzPerS(pulse_div,ramp_div,velocity_max_hz);

  while ((acceleration_max_upper_limit < acceleration_max_hz_per_s) &&
    (acceleration_max_lower_limit < acceleration_max_hz_per_s) &&
    (ramp_div >= 1) &&
    (ramp_div >= pulse_div))
  {
    --ramp_div;
    acceleration_max_upper_limit = getAccelerationMaxUpperLimitInHzPerS(pulse_div,ramp_div);
    acceleration_max_lower_limit = getAccelerationMaxLowerLimitInHzPerS(pulse_div,ramp_div,velocity_max_hz);
  }
  return ramp_div;
}

void AS5048::setOptimalRampDivHz(size_t motor,
  uint32_t velocity_max_hz,
  uint32_t acceleration_max_hz_per_s)
{
  uint8_t ramp_div = findOptimalRampDivHz(velocity_max_hz,acceleration_max_hz_per_s);
  ClkConfig clk_config;
  clk_config.uint32 = readRegister(motor,ADDRESS_CLOCK_CONFIGURATION);
  clk_config.fields.clk_config.ramp_div = ramp_div;
  writeRegister(motor,ADDRESS_CLOCK_CONFIGURATION,clk_config.uint32);
  ramp_div_[motor] = ramp_div;
}

uint32_t AS5048::getVelocityMaxUpperLimitInHz(uint8_t pulse_div)
{
  return convertVelocityToHz(pulse_div,VELOCITY_REGISTER_MAX);
}

uint32_t AS5048::getAccelerationMaxUpperLimitInHzPerS(uint8_t pulse_div,
  uint8_t ramp_div)
{
  uint32_t a_max_upper_limit;
  if (((int8_t)ramp_div - (int8_t)pulse_div + 1) >= 0)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MAX;
  }
  else if (((int8_t)ramp_div - (int8_t)pulse_div + 12) < 1)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MIN;
  }
  else
  {
    a_max_upper_limit = (1 << ((int8_t)ramp_div - (int8_t)pulse_div + 12)) - 1;
  }
  if (a_max_upper_limit > ACCELERATION_REGISTER_MAX)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MAX;
  }
  if (a_max_upper_limit < ACCELERATION_REGISTER_MIN)
  {
    a_max_upper_limit = ACCELERATION_REGISTER_MIN;
  }
  return convertAccelerationToHzPerS(pulse_div,ramp_div,a_max_upper_limit);
}

uint32_t AS5048::getAccelerationMaxLowerLimitInHzPerS(uint8_t pulse_div,
  uint8_t ramp_div,
  uint32_t velocity_max)
{
  uint32_t a_max_lower_limit;
  if (((int8_t)ramp_div - (int8_t)pulse_div - 1) <= 0)
  {
    a_max_lower_limit = ACCELERATION_REGISTER_MIN;
  }
  else
  {
    a_max_lower_limit = (1 << ((int8_t)ramp_div - (int8_t)pulse_div - 1));
    if (convertVelocityFromHz(pulse_div,velocity_max) <= (int16_t)VELOCITY_REGISTER_THRESHOLD)
    {
      a_max_lower_limit /= 2;
    }
  }
  if (a_max_lower_limit > ACCELERATION_REGISTER_MAX)
  {
    a_max_lower_limit = ACCELERATION_REGISTER_MAX;
  }
  if (a_max_lower_limit < ACCELERATION_REGISTER_MIN)
  {
    a_max_lower_limit = ACCELERATION_REGISTER_MIN;
  }
  return convertAccelerationToHzPerS(pulse_div,ramp_div,a_max_lower_limit);
}

uint32_t AS5048::getAccelerationMax(size_t motor)
{
  return readRegister(motor,ADDRESS_A_MAX);
}

uint32_t AS5048::setAccelerationMaxInHzPerS(size_t motor,
  uint32_t velocity_max_hz,
  uint32_t acceleration_max_hz_per_s)
{
  uint32_t acceleration_max_upper_limit = getAccelerationMaxUpperLimitInHzPerS(pulse_div_[motor],ramp_div_[motor]);
  uint32_t acceleration_max_lower_limit = getAccelerationMaxLowerLimitInHzPerS(pulse_div_[motor],ramp_div_[motor],velocity_max_hz);
  if (acceleration_max_hz_per_s > acceleration_max_upper_limit)
  {
    acceleration_max_hz_per_s = acceleration_max_upper_limit;
  }
  if (acceleration_max_hz_per_s < acceleration_max_lower_limit)
  {
    acceleration_max_hz_per_s = acceleration_max_lower_limit;
  }
  uint32_t acceleration_max = convertAccelerationFromHzPerS(pulse_div_[motor],ramp_div_[motor],acceleration_max_hz_per_s);
  if (acceleration_max > ACCELERATION_REGISTER_MAX)
  {
    acceleration_max = ACCELERATION_REGISTER_MAX;
  }
  if (acceleration_max < ACCELERATION_REGISTER_MIN)
  {
    acceleration_max = ACCELERATION_REGISTER_MIN;
  }
  writeRegister(motor,ADDRESS_A_MAX,acceleration_max);
  return acceleration_max;
}

int16_t AS5048::getActualAcceleration(size_t motor)
{
  uint32_t acceleration_unsigned = readRegister(motor,ADDRESS_A_ACTUAL);
  return unsignedToSigned(acceleration_unsigned,A_BIT_COUNT);
}

void AS5048::setOptimalPropFactor(size_t motor,
  uint32_t acceleration_max)
{
  // int pdiv, pmul, pm, pd ;
  // double p_ideal, p_best, p, p_reduced;

  // pm=-1; pd=-1; // -1 indicates : no valid pair found

  // p_ideal = a_max / (pow(2, ramp_div-pulse_div)*128.0);
  // p = a_max / ( 128.0 * pow(2, ramp_div-pulse_div) );
  // p_reduced = p * ( 1.0 – p_reduction );
  // for (pdiv=0; pdiv<=13; pdiv++)
  // {
  //   pmul = (int)(p_reduced * 8.0 * pow(2, pdiv)) – 128;
  //   if ( (0 <= pmul) && (pmul <= 127) )
  //   {
  //     pm = pmul + 128;
  //     pd = pdiv;
  //   }
  //   *p_mul = pm;
  //   *p_div = pd;
  //   p_best = ((double)(pm)) / ((double)pow(2,pd+3));
  // }
  // *PIdeal = p_ideal;
  // *PBest = p_best;
  // *PRedu = p_reduced;

  int pdiv, pmul, pm, pd ;
  double p_ideal, p_reduced;

  pm=-1; pd=-1; // -1 indicates : no valid pair found
  p_ideal = acceleration_max/(128.0*(1 << (ramp_div_[motor] - pulse_div_[motor])));
  p_reduced = p_ideal*0.99;
  for (pdiv=0; pdiv<=13; ++pdiv)
  {
    pmul = (int)(p_reduced*8.0*(1 << pdiv)) - 128;
    if ((0 <= pmul) && (pmul <= 127))
    {
      pm = pmul + 128;
      pd = pdiv;
    }
  }
  if ((pm == -1) || (pd == -1))
  {
    return;
  }
  PropFactor prop_factor;
  prop_factor.uint32 = readRegister(motor,ADDRESS_PROP_FACTOR);
  prop_factor.fields.pmul = pm;
  prop_factor.fields.pdiv = pd;
  writeRegister(motor,ADDRESS_PROP_FACTOR,prop_factor.uint32);
}

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
