void		initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts);
WarpStatus	readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
WarpStatus	writeSensorRegisterINA219(uint8_t deviceRegister, uint8_t payloadBtye);
WarpStatus 	configureSensorINA219(uint8_t payload_Config, uint8_t payload_Calibration);
void		printSensorDataINA219(bool hexModeFlag);
uint8_t		appendSensorDataINA219(uint8_t* buf);

const uint8_t bytesPerMeasurementINA219          = 2;
const uint8_t bytesPerReadingINA219                = 2;
const uint8_t numberOfReadingsPerMeasurementINA219 = 1;