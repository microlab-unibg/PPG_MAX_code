#include <Arduino.h>
#include <Wire.h>

#include "MAX86916_cm.h"



MAX86916::MAX86916() {  
  // Constructor
}


//begin-> i2c configuration
boolean MAX86916::begin() {
  Wire.begin();
  Wire.setClock(I2C_SPEED_FAST);
  // Check that a MAX86916 is connected. I read the value of the register 0xff 
  //I check that this corrisponds to the expected value
  if (read_Part_ID() != MAX86916_EXPECTED_PART_ID) {
    // Error -- Part ID read from MAX86916 does not match expected part ID.
    return false;
  }
  read_Revision_ID();
  return true;
}

//Setup the sensor
// Default settings:
// Sample Average = 4
// Mode = 3(all led)
// Sample rate = 400
// Pulse Width = 420
// ADC Range = 16384 (31.25pA per LSB)
void MAX86916::setup(uint8_t range, uint8_t powerLevel, uint8_t sampleAverage, byte ledMode, uint8_t sampleRate, uint8_t pulseWidth, uint8_t adcRange) {
  softReset(); //Reset all configuration, threshold, and data registers

  //FIFO Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  setFIFOAverage(sampleAverage);
  

  //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  enableFIFORollover(); //Allow FIFO to wrap/roll over
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Mode Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (ledMode == 3) setLEDMode(MAX86916_MODE_FLEXLED); //Watch all three LED channels
  else if (ledMode == 2) setLEDMode(MAX86916_MODE_REDIRONLY); //Red and IR
  else setLEDMode(MAX86916_MODE_IRONLY); //IR only
  if (ledMode < 3) activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer
  else if(ledMode == 3) activeLEDs = 4;
  else activeLEDs = ledMode;
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Particle Sensing Configuration
  setADCRange(adcRange);

  setSampleRate(sampleRate);

  //The longer the pulse width the longer range of detection you'll have
  //At 70us and 0.4mA it's about 2 inches
  //At 420us and 0.4mA it's about 6 inches
  setPulseWidth(pulseWidth);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //LED Pulse Amplitude Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Default is range=0 powerLevel=0x1F which gets us 6.4mA
  //range      = 0
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  setAllLEDRange(range);
  setPA_IR(powerLevel);
  setPA_RED(powerLevel);
  setPA_GREEN(powerLevel);
  setPA_BLUE(powerLevel);
  setPA_PROX(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Flex-LED Mode Configuration, Enable the reading of the three LEDs
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  enableSlot(1, SLOT_IR_LED);
  if (ledMode > 1) enableSlot(2, SLOT_RED_LED);
  if (ledMode > 2){
    enableSlot(3, SLOT_GREEN_LED);
    enableSlot(4, SLOT_BLUE_LED);
  }
  //enableSlot(1, SLOT_IR_PILOT);
  //enableSlot(2, SLOT_RED_PILOT);
  //enableSlot(3, SLOT_GREEN_PILOT);
  //enableSlot(4, SLOT_BLUE_PILOT);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  clearFIFO(); //Reset the FIFO before we begin checking the sensor
}


//
// Configuration
//

//Begin Interrupt configuration
uint8_t MAX86916::get_INT_1() {
  return (readRegister(MAX86916_I2C_ADDRESS, MAX86916_INT_STAT_1));
}

void MAX86916::enable_A_FULL() {
  mask(MAX86916_INT_EN_1, MAX86916_INT_A_FULL_MASK, MAX86916_INT_A_FULL_ENABLE);
}
void MAX86916::disable_A_FULL() {
  mask(MAX86916_INT_EN_1, MAX86916_INT_A_FULL_MASK, MAX86916_INT_A_FULL_DISABLE);
}

void MAX86916::enable_DATA_RDY() {
  mask(MAX86916_INT_EN_1, MAX86916_INT_DATA_RDY_MASK, MAX86916_INT_DATA_RDY_ENABLE);
}
void MAX86916::disable_DATA_RDY() {
  mask(MAX86916_INT_EN_1, MAX86916_INT_DATA_RDY_MASK, MAX86916_INT_DATA_RDY_DISABLE);
}

void MAX86916::enable_ALC_OVF() {
  mask(MAX86916_INT_EN_1, MAX86916_INT_ALC_OVF_MASK, MAX86916_INT_ALC_OVF_ENABLE);
}
void MAX86916::disable_ALC_OVF() {
  mask(MAX86916_INT_EN_1, MAX86916_INT_ALC_OVF_MASK, MAX86916_INT_ALC_OVF_DISABLE);
}

void MAX86916::enable_PROX_INT() {
  mask(MAX86916_INT_EN_1, MAX86916_INT_PROX_INT_MASK, MAX86916_INT_PROX_INT_ENABLE);
}
void MAX86916::disable_PROX_INT() {
  mask(MAX86916_INT_EN_1, MAX86916_INT_PROX_INT_MASK, MAX86916_INT_PROX_INT_DISABLE);
}
//End Interrupt configuration

void MAX86916::softReset() {
  mask(MAX86916_MODECONFIG, MAX86916_RESET_MASK, MAX86916_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  unsigned long startTime = millis();
  while (millis() - startTime < 100)
  {
    uint8_t response = readRegister(MAX86916_I2C_ADDRESS, MAX86916_MODECONFIG);
    if ((response & MAX86916_RESET) == 0) break; //We're done!
    delay(1); //Let's not over burden the I2C bus
  }
}

void MAX86916::shutDown() {
  // Put IC into low power mode (datasheet)
  // During shutdown the IC will continue to respond to I2C commands but will
  // not update with or take new readings (such as temperature)
  mask(MAX86916_MODECONFIG, MAX86916_SHUTDOWN_MASK, MAX86916_SHUTDOWN);
}

void MAX86916::wakeUp() {
  // Pull IC out of low power mode (datasheet)
  mask(MAX86916_MODECONFIG, MAX86916_SHUTDOWN_MASK, MAX86916_WAKEUP);
}

void MAX86916::setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- IR only, RED+IR only, or FLEX.
  // See datasheet
  mask(MAX86916_MODECONFIG, MAX86916_MODE_MASK, mode);
}

void MAX86916::setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX86916_ADCRANGE_4096, _8192, _16384, _32768
  mask(MAX86916_MODECONFIG2, MAX86916_ADCRANGE_MASK, adcRange);
}

void MAX86916::setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX86916_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  mask(MAX86916_MODECONFIG2, MAX86916_SAMPLERATE_MASK, sampleRate);
}

void MAX86916::setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX86916_PULSEWIDTH_70, _120, _220, _420
  mask(MAX86916_MODECONFIG2, MAX86916_PULSEWIDTH_MASK, pulseWidth);
}

// See datasheet, page 30
void MAX86916::setLEDRange(uint8_t rangeMask, uint8_t range) {
  // rangeMask: one of MAX86916_LEDx_RGE_MASK
  // range: one of MAX86916_LEDx_RGE_X1, _X2, _X3, _X4
  if(rangeMask      == MAX86916_LED1_RGE_MASK) mask(MAX86916_LED_RANGE, rangeMask, range);
  else if(rangeMask == MAX86916_LED2_RGE_MASK) mask(MAX86916_LED_RANGE, rangeMask, range << 2);
  else if(rangeMask == MAX86916_LED3_RGE_MASK) mask(MAX86916_LED_RANGE, rangeMask, range << 4);
  else if(rangeMask == MAX86916_LED4_RGE_MASK) mask(MAX86916_LED_RANGE, rangeMask, range << 6);
  else mask(MAX86916_LED_RANGE, MAX86916_LED1_RGE_MASK, range);
}

void MAX86916::setAllLEDRange(uint8_t range){
  // range: one of MAX86916_LEDx_RGE_X1, _X2, _X3, _X4
  setLEDRange(MAX86916_LED1_RGE_MASK, range);
  setLEDRange(MAX86916_LED2_RGE_MASK, range);
  setLEDRange(MAX86916_LED3_RGE_MASK, range);
  setLEDRange(MAX86916_LED4_RGE_MASK, range);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0xFF = 50.4mA @ LEDX_RGE = 0x0, 0xFF = 201.6mA @ LEDX_RGE = 0x3
void MAX86916::setPA_IR(uint8_t amplitude) {
  writeRegister(MAX86916_I2C_ADDRESS, MAX86916_LED1_PULSEAMP, amplitude);
}

void MAX86916::setPA_RED(uint8_t amplitude) {
  writeRegister(MAX86916_I2C_ADDRESS, MAX86916_LED2_PULSEAMP, amplitude);
}

void MAX86916::setPA_GREEN(uint8_t amplitude) {
  writeRegister(MAX86916_I2C_ADDRESS, MAX86916_LED3_PULSEAMP, amplitude);
}

void MAX86916::setPA_BLUE(uint8_t amplitude) {
  writeRegister(MAX86916_I2C_ADDRESS, MAX86916_LED4_PULSEAMP, amplitude);
}

void MAX86916::setPA_PROX(uint8_t amplitude) {
  writeRegister(MAX86916_I2C_ADDRESS, MAX86916_LED_PROX_AMP, amplitude);
}

void MAX86916::setProximityThreshold(uint8_t threshMSB) {
  // Set the IR ADC count that will trigger the beginning of particle-sensing mode.
  // The threshMSB signifies only the 8 most significant-bits of the ADC count.
  // See datasheet, page 24.
  writeRegister(MAX86916_I2C_ADDRESS, MAX86916_PROXINTTHRESH, threshMSB);
}

//Given a slot number assign a data to it
//Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
//Assigning a SLOT_RED_LED will pulse LED
//Assigning a SLOT_RED_PILOT will ??
void MAX86916::enableSlot(uint8_t slotNumber, uint8_t device) {

  

  switch (slotNumber) {
    case (1):
      mask(MAX86916_LED_SEQREG1, MAX86916_SLOT1_MASK, device);
      break;
    case (2):
      mask(MAX86916_LED_SEQREG1, MAX86916_SLOT2_MASK, device << 4);
      break;
    case (3):
      mask(MAX86916_LED_SEQREG2, MAX86916_SLOT3_MASK, device);
      break;
    case (4):
      mask(MAX86916_LED_SEQREG2, MAX86916_SLOT4_MASK, device << 4);
      break;
    default:
      //Shouldn't be here!
      break;
  }
}

//Clears all slot assignments
void MAX86916::disableSlots() {
  writeRegister(MAX86916_I2C_ADDRESS, MAX86916_LED_SEQREG1, 0);
  writeRegister(MAX86916_I2C_ADDRESS, MAX86916_LED_SEQREG2, 0);
}

//
// FIFO Configuration
//

//Set sample average (Table 3, Page 18)
void MAX86916::setFIFOAverage(uint8_t numberOfSamples) {
  mask(MAX86916_FIFOCONFIG, MAX86916_SAMPLEAVG_MASK, numberOfSamples);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void MAX86916::clearFIFO() {
  writeRegister(MAX86916_I2C_ADDRESS, MAX86916_FIFO_WRITE_PTR, 0);
  writeRegister(MAX86916_I2C_ADDRESS, MAX86916_FIFO_OVERFLOW, 0);
  writeRegister(MAX86916_I2C_ADDRESS, MAX86916_FIFO_READ_PTR, 0);
}

//Enable roll over if FIFO over flows
void MAX86916::enableFIFORollover() {
  mask(MAX86916_FIFOCONFIG, MAX86916_ROLLOVER_MASK, MAX86916_ROLLOVER_ENABLE);
}

//Disable roll over if FIFO over flows
void MAX86916::disableFIFORollover() {
  mask(MAX86916_FIFOCONFIG, MAX86916_ROLLOVER_MASK, MAX86916_ROLLOVER_DISABLE);
}

//Set number of samples to trigger the almost full interrupt (Page 18)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void MAX86916::setFIFOAlmostFull(uint8_t numberOfSamples) {
  mask(MAX86916_FIFOCONFIG, MAX86916_A_FULL_MASK, numberOfSamples);
}

//Read the FIFO Write Pointer
uint8_t MAX86916::getWritePointer() {
  return (readRegister(MAX86916_I2C_ADDRESS, MAX86916_FIFO_WRITE_PTR));
}

//Read the FIFO Read Pointer
uint8_t MAX86916::getReadPointer() {
  return (readRegister(MAX86916_I2C_ADDRESS, MAX86916_FIFO_READ_PTR));
}

// Set the PROX_INT_THRESHold
void MAX86916::setPROXINTTHRESH(uint8_t val) {
  writeRegister(MAX86916_I2C_ADDRESS, MAX86916_PROXINTTHRESH, val);
}

//
// Device ID and Revision
//
uint8_t MAX86916::read_Part_ID() {
  return readRegister(MAX86916_I2C_ADDRESS, MAX86916_PARTID);
}

void MAX86916::read_Revision_ID() {
  revisionID = readRegister(MAX86916_I2C_ADDRESS, MAX86916_REVISIONID);
}

uint8_t MAX86916::getRevisionID() {
  return revisionID;
}



//
// Data Collection
//

//Tell caller how many samples are available
uint8_t MAX86916::available()
{
  int8_t numberOfSamples = sensor_data_buffer.head - sensor_data_buffer.tail;
  if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;

  return (numberOfSamples);
}

//Report the most recent IR value
uint32_t MAX86916::getIR()
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sensor_data_buffer.IR[sensor_data_buffer.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent red value
uint32_t MAX86916::getRed()
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sensor_data_buffer.red[sensor_data_buffer.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent Green value
uint32_t MAX86916::getGreen()
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sensor_data_buffer.green[sensor_data_buffer.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent Blue value
uint32_t MAX86916::getBlue()
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sensor_data_buffer.blue[sensor_data_buffer.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the next IR value in the FIFO
uint32_t MAX86916::getFIFOIR()
{
  return (sensor_data_buffer.IR[sensor_data_buffer.tail]);
}

//Report the next Red value in the FIFO
uint32_t MAX86916::getFIFORed()
{
  return (sensor_data_buffer.red[sensor_data_buffer.tail]);
}

//Report the next Green value in the FIFO
uint32_t MAX86916::getFIFOGreen()
{
  return (sensor_data_buffer.green[sensor_data_buffer.tail]);
}

//Report the next Blue value in the FIFO
uint32_t MAX86916::getFIFOBlue()
{
  return (sensor_data_buffer.blue[sensor_data_buffer.tail]);
}

//Advance the tail
void MAX86916::nextSample()
{
  if(available()) //Only advance the tail if new data is available
  {
    sensor_data_buffer.tail++;
    sensor_data_buffer.tail %= STORAGE_SIZE; //Wrap condition
  }
}

//Polls the sensor for new data
//Call regularly
//If new data is available, it updates the head and tail in the main struct
//Returns number of new samples obtained
uint16_t MAX86916::check()
{
  //Read register FIFO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  byte readPointer = getReadPointer();
  byte writePointer = getWritePointer();
 
	int numberOfSamples = 0;
  //Do we have new data?
  if (readPointer != writePointer) //if yes
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition
    

    //We now have the number of readings, now calc bytes to read
    //For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    //Get ready to read a burst of data from the FIFO register
    Wire.beginTransmission(MAX86916_I2C_ADDRESS);
    Wire.write(MAX86916_FIFO_DATA);
    Wire.endTransmission();

    //We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
    //I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
    //Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
        //If toGet is 32 this is bad because we read 6 bytes (IR+Red * 3 = 6) at a time
        //32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
        //32 % 12 (IR+Red+GREEN+BLUE) = 8 left over. We want to request 24.

        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
      }

      bytesLeftToRead -= toGet;

      //Request toGet number of bytes from sensor
      Wire.requestFrom(MAX86916_I2C_ADDRESS, toGet);
      
      while (toGet > 0)
      {
        sensor_data_buffer.head++; //Advance the head of the storage struct
        sensor_data_buffer.head %= STORAGE_SIZE; //Wrap condition

        byte temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
        uint32_t tempLong;

        //Burst read three bytes - IR
        temp[3] = 0;
        temp[2] = Wire.read();
        temp[1] = Wire.read();
        temp[0] = Wire.read();

        //Convert array to long
        memcpy(&tempLong, temp, sizeof(tempLong));
    
    tempLong &= 0x3FFFF; //Zero out all but 18 bits

        sensor_data_buffer.IR[sensor_data_buffer.head] = tempLong; //Store this reading into the sensor_data_buffer array

        if (activeLEDs > 1)
        {
          //Burst read three more bytes - Red
          temp[3] = 0;
          temp[2] = Wire.read();
          temp[1] = Wire.read();
          temp[0] = Wire.read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

      tempLong &= 0x3FFFF; //Zero out all but 18 bits
          
      sensor_data_buffer.red[sensor_data_buffer.head] = tempLong;
        }

        if (activeLEDs > 2)
        {
          //Burst read three more bytes - Green
          temp[3] = 0;
          temp[2] = Wire.read();
          temp[1] = Wire.read();
          temp[0] = Wire.read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

      tempLong &= 0x3FFFF; //Zero out all but 18 bits

          sensor_data_buffer.green[sensor_data_buffer.head] = tempLong;

          //Burst read three more bytes - Blue
          temp[3] = 0;
          temp[2] = Wire.read();
          temp[1] = Wire.read();
          temp[0] = Wire.read();

          //Convert array to long
          memcpy(&tempLong, temp, sizeof(tempLong));

      tempLong &= 0x3FFFF; //Zero out all but 18 bits

          sensor_data_buffer.blue[sensor_data_buffer.head] = tempLong;
        }

        toGet -= activeLEDs * 3;
      }

    } //End while (bytesLeftToRead > 0)

  } //End readPtr != writePtr

  return (numberOfSamples); //Let the world know how much new data we found
}

//Check for new data but give up after a certain amount of time
//Returns true if new data was found
//Returns false if new data was not found
bool MAX86916::safeCheck(uint8_t maxTimeToCheck)
{
  uint32_t markTime = millis();
  
  while(1)
  {
  if(millis() - markTime > maxTimeToCheck) return(false);

  if(check() ==true ) //We found new data!
    return(true);

  delay(1);
  }
}

//Given a register, read it, mask it, and then set the data
void MAX86916::mask(uint8_t reg, uint8_t mask, uint8_t data)
{
  // Grab current register context
  uint8_t originalContents = readRegister(MAX86916_I2C_ADDRESS, reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeRegister(MAX86916_I2C_ADDRESS, reg, originalContents | data);
}

//
// Low-level I2C Communication
//
uint8_t MAX86916::readRegister(uint8_t address, uint8_t reg) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom((uint8_t)address, (uint8_t)1); // Request 1 byte
  if (Wire.available())
  {
    return(Wire.read());
  }

  return (0); //Fail

}

void MAX86916::writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}



//Read Overflow counter register
uint8_t MAX86916::getOverflowCounter() {
  return (readRegister(MAX86916_I2C_ADDRESS, MAX86916_FIFO_OVERFLOW));
}
  
  



uint16_t MAX86916::nr_sample_FIFO()
{
  //Read register FIFO_DATA in (3-byte * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  byte readPointer = getReadPointer();
  byte writePointer = getWritePointer();
  byte overflowCounter= getOverflowCounter();
  int numberOfSamples = 0;
  
  //Do we have new data?
  if (overflowCounter==0)  //no overflow has occured
  {
  	if (writePointer> readPointer)
  	{
  		numberOfSamples = writePointer - readPointer;
	}
    else
	{
    	numberOfSamples = writePointer - readPointer+32;
	}
  }
  else 
  {
  	numberOfSamples=32;
  }
  
  return (numberOfSamples);



}
