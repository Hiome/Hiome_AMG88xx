#include "Hiome_AMG88xx.h"

/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
    @param  addr Optional I2C address the sensor can be found on. Default is 0x69
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/
bool Hiome_AMG88xx::begin(uint8_t addr)
{
	_i2caddr = addr;
	
	I2c.begin();
	
  //enter normal mode
	setNormalMode();
	
	//software reset
	_rst.RST = AMG88xx_INITIAL_RESET;
	write8(AMG88xx_RST, _rst.get());
	
	//disable interrupts by default
	disableInterrupt();
	
	//set to 10 FPS
	_fpsc.FPS = AMG88xx_FPS_10;
	write8(AMG88xx_FPSC, _fpsc.get());

	return true;
}

/**************************************************************************/
/*! 
    @brief  Read current power mode
    @returns current power mode state
      // 0x00 = Normal Mode
      // 0x01 = Sleep Mode
      // 0x20 = Stand-by mode (60 sec intermittence)
      // 0x21 = Stand-by mode (10 sec intermittence))
*/
/**************************************************************************/
uint8_t Hiome_AMG88xx::getPowerMode()
{
  return _pctl.get();
}

/**************************************************************************/
/*! 
    @brief  Set power mode to normal.
*/
/**************************************************************************/
void Hiome_AMG88xx::setNormalMode()
{
  _pctl.PCTL = AMG88xx_NORMAL_MODE;
  write8(AMG88xx_PCTL, _pctl.get());
}

/**************************************************************************/
/*! 
    @brief  Set power mode to sleep, temperature register is not updated.
*/
/**************************************************************************/
void Hiome_AMG88xx::setSleepMode()
{
  _pctl.PCTL = AMG88xx_SLEEP_MODE;
  write8(AMG88xx_PCTL, _pctl.get());
}

/**************************************************************************/
/*! 
    @brief  Set power mode to standby, temperature register is updated every 60 seconds.
*/
/**************************************************************************/
void Hiome_AMG88xx::set60sStandbyMode()
{
  _pctl.PCTL = AMG88xx_STAND_BY_60;
  write8(AMG88xx_PCTL, _pctl.get());
}

/**************************************************************************/
/*! 
    @brief  Set power mode to standby, temperature register is updated every 10 seconds.
*/
/**************************************************************************/
void Hiome_AMG88xx::set10sStandbyMode()
{
  _pctl.PCTL = AMG88xx_STAND_BY_10;
  write8(AMG88xx_PCTL, _pctl.get());
}

/**************************************************************************/
/*! 
    @brief  Set the moving average mode.
    @param  mode if True is passed, output will be twice the moving average
*/
/**************************************************************************/
void Hiome_AMG88xx::setMovingAverageMode(bool mode)
{
	_ave.MAMOD = mode;
	write8(AMG88xx_AVE, _ave.get());
}

/**************************************************************************/
/*! 
    @brief  Set the interrupt levels. The hysteresis value defaults to .95 * high
    @param  high the value above which an interrupt will be triggered
    @param  low the value below which an interrupt will be triggered
*/
/**************************************************************************/
void Hiome_AMG88xx::setInterruptLevels(float high, float low)
{
	setInterruptLevels(high, low, high * .95);
}

/**************************************************************************/
/*! 
    @brief  Set the interrupt levels
    @param  high the value above which an interrupt will be triggered
    @param  low the value below which an interrupt will be triggered
    @param hysteresis the hysteresis value for interrupt detection
*/
/**************************************************************************/
void Hiome_AMG88xx::setInterruptLevels(float high, float low, float hysteresis)
{
	int highConv = high / AMG88xx_PIXEL_TEMP_CONVERSION;
	highConv = constrain(highConv, -4095, 4095);
	_inthl.INT_LVL_H = highConv & 0xFF;
	_inthh.INT_LVL_H = (highConv & 0xF) >> 4;
	this->write8(AMG88xx_INTHL, _inthl.get());
	this->write8(AMG88xx_INTHH, _inthh.get());
	
	int lowConv = low / AMG88xx_PIXEL_TEMP_CONVERSION;
	lowConv = constrain(lowConv, -4095, 4095);
	_intll.INT_LVL_L = lowConv & 0xFF;
	_intlh.INT_LVL_L = (lowConv & 0xF) >> 4;
	this->write8(AMG88xx_INTLL, _intll.get());
	this->write8(AMG88xx_INTLH, _intlh.get());
	
	int hysConv = hysteresis / AMG88xx_PIXEL_TEMP_CONVERSION;
	hysConv = constrain(hysConv, -4095, 4095);
	_ihysl.INT_HYS = hysConv & 0xFF;
	_ihysh.INT_HYS = (hysConv & 0xF) >> 4;
	this->write8(AMG88xx_IHYSL, _ihysl.get());
	this->write8(AMG88xx_IHYSH, _ihysh.get());
}

/**************************************************************************/
/*! 
    @brief  enable the interrupt pin on the device.
*/
/**************************************************************************/
void Hiome_AMG88xx::enableInterrupt()
{
	_intc.INTEN = 1;
	this->write8(AMG88xx_INTC, _intc.get());
}

/**************************************************************************/
/*! 
    @brief  disable the interrupt pin on the device
*/
/**************************************************************************/
void Hiome_AMG88xx::disableInterrupt()
{
	_intc.INTEN = 0;
	this->write8(AMG88xx_INTC, _intc.get());
}

/**************************************************************************/
/*! 
    @brief  Set the interrupt to either absolute value or difference mode
    @param  mode passing AMG88xx_DIFFERENCE sets the device to difference mode, AMG88xx_ABSOLUTE_VALUE sets to absolute value mode.
*/
/**************************************************************************/
void Hiome_AMG88xx::setInterruptMode(uint8_t mode)
{
	_intc.INTMOD = mode;
	this->write8(AMG88xx_INTC, _intc.get());
}

/**************************************************************************/
/*! 
    @brief  Read the state of the triggered interrupts on the device. The full interrupt register is 8 bytes in length.
    @param  buf the pointer to where the returned data will be stored
    @param  size Optional number of bytes to read. Default is 8 bytes.
    @returns up to 8 bytes of data in buf
*/
/**************************************************************************/
void Hiome_AMG88xx::getInterrupt(uint8_t *buf, uint8_t size)
{
	uint8_t bytesToRead = min(size, (uint8_t)8);
	
  uint8_t num = bytesToRead << 1;
  uint8_t rxBuffer[num];
  I2c.read(_i2caddr, AMG88xx_INT_OFFSET, num, rxBuffer);

  char tmp;
  int raw;

  for (uint8_t i=0; i<num; i++){
    tmp = rxBuffer[i];

    // if we are processing the upper byte
    if (i % 2) {
      // convert the upper byte of the 12 bit signed value into proper 16 bit
      // signed format, if value is negative
      if(tmp & 0b00001000) tmp |= 0b11111000;

      // add the upper byte into 16 bit register by shifting it 8 bits up
      raw |= tmp << 8;

      buf[i/2] = raw;
    } else {
      // lower byte is being processed
      raw = 0xFF & tmp;
    }
  }
}

/**************************************************************************/
/*! 
    @brief  Clear any triggered interrupts
*/
/**************************************************************************/
void Hiome_AMG88xx::clearInterrupt()
{
	_rst.RST = AMG88xx_FLAG_RESET;
	write8(AMG88xx_RST, _rst.get());
}

/**************************************************************************/
/*! 
    @brief  read the onboard thermistor
    @returns a the floating point temperature in degrees Celsius
*/
/**************************************************************************/
float Hiome_AMG88xx::readThermistor()
{
  uint8_t rxBuffer[2];
  I2c.read(_i2caddr, AMG88xx_TTHL, 2, rxBuffer);

  char tmp = rxBuffer[1];
  // convert the upper byte of the 12 bit signed value into proper 16 bit
  // signed format, if value is negative
  if(tmp & 0b00001000) tmp |= 0b11111000;

  // add the upper byte into 16 bit register by shifting it 8 bits up
  int raw = (tmp << 8) | rxBuffer[0];

  return raw * AMG88xx_THERMISTOR_CONVERSION;
}

/**************************************************************************/
/*! 
    @brief  Read Infrared sensor values in ºC
    @param  buf the array to place the pixels in
    @param  minVal don't let pixel value fall below this
    @param  maxVal don't let pixel value rise above this
    @return true/false whether buf changed, 64 floats of pixel data in buf
*/
/**************************************************************************/
bool Hiome_AMG88xx::readPixels(float *buf, float minVal, float maxVal)
{
  uint8_t rxBuffer[128];
  uint8_t res = I2c.read(_i2caddr, AMG88xx_PIXEL_OFFSET, 128, rxBuffer);
  if (res) return false;

  char tmp;
  int raw = 0;
  bool changed = false;

  for (uint8_t i=0; i<128; i++){
    tmp = rxBuffer[i];

    // if we are processing the upper byte
    if (i % 2) {
      // convert the upper byte of the 12 bit signed value into proper 16 bit
      // signed format, if value is negative
      if(tmp & 0b00001000) tmp |= 0b11111000;

      // add the upper byte into 16 bit register by shifting it 8 bits up
      raw |= tmp << 8;

      // convert to Celsius
      float newVal = raw * AMG88xx_PIXEL_TEMP_CONVERSION;

      // enforce min and max boundaries
      if (newVal < minVal) newVal = minVal;
      else if (newVal > maxVal) newVal = maxVal;

      // check if pixel changed
      if (!changed && abs(newVal - buf[i/2]) > 0.001) changed = true;

      // save new value
      buf[i/2] = newVal;
    } else {
      // lower byte is being processed
      raw = 0xFF & tmp;
    }
  }

  return changed;
}

/**************************************************************************/
/*! 
    @brief  write one byte of data to the specified register
    @param  reg the register to write to
    @param  value the value to write
*/
/**************************************************************************/
void Hiome_AMG88xx::write8(byte reg, byte value)
{
  I2c.write(_i2caddr, reg, &value, 1);
}
