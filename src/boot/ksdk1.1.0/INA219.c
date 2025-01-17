#include <stdlib.h>

/*
 *  config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"


extern volatile WarpI2CDeviceState  deviceINA219State;
extern volatile uint32_t        gWarpI2cBaudRateKbps;
extern volatile uint32_t        gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t        gWarpSupplySettlingDelayMilliseconds;


void set_Calibration_reg();
void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
    deviceINA219State.i2cAddress            = i2cAddress;
    deviceINA219State.operatingVoltageMillivolts    = operatingVoltageMillivolts;
    set_Calibration_reg();
    return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload)
{
    uint8_t     payloadByte[2], commandByte[1];
    i2c_status_t    status;

    switch (deviceRegister)
    {
        case 0x00: 
        case 0x05: 
        {
            /* OK */
            break;
        }

        default:
        {
            return kWarpStatusBadDeviceCommand;
        }
    }

    i2c_device_t slave =
        {
        .address = deviceINA219State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
    commandByte[0] = deviceRegister;
    payloadByte[0] = (payload >> 8) & 0xFF; // MSB
    payloadByte[1] = payload & 0xFF; // LSB
    warpEnableI2Cpins();

    status = I2C_DRV_MasterSendDataBlocking(
        0 /* I2C instance */,
        &slave,
        commandByte,
        1,
        payloadByte,
        2,
        gWarpI2cTimeoutMilliseconds);
    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

WarpStatus
configureSensorINA219(uint16_t payload_Config, uint16_t payload_Calibration)
{
    WarpStatus  i2cWriteStatus1, i2cWriteStatus2;


    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

    i2cWriteStatus1 = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219 /* register address config */,
                                                  payload_Config );

    i2cWriteStatus2 = writeSensorRegisterINA219(kWarpSensorCalibrationRegisterINA219 /* register address need to add to warp.h 0x00 */,
                            payload_Calibration /* payload: Should be user input*/
                            );


    return (i2cWriteStatus1 | i2cWriteStatus2);
}

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
    uint8_t     cmdBuf[1] = {0xFF};
    i2c_status_t    status;
    
    USED(numberOfBytes);
    switch (deviceRegister)
    {
        case 0x00: case 0x01: case 0x02: case 0x03: case 0x04:
        case 0x05: 
        {
            /* OK */
            break;
        }

        default:
        {
            return kWarpStatusBadDeviceCommand;
        }
    }

    i2c_device_t slave =
        {
        .address = deviceINA219State.i2cAddress,
        .baudRate_kbps = gWarpI2cBaudRateKbps
    };

    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
    cmdBuf[0] = deviceRegister;
    warpEnableI2Cpins();

    status = I2C_DRV_MasterReceiveDataBlocking(
        0 /* I2C peripheral instance */,
        &slave,
        cmdBuf,
        1,
        (uint8_t *)deviceINA219State.i2cBuffer,
        numberOfBytes,
        gWarpI2cTimeoutMilliseconds);

    if (status != kStatus_I2C_Success)
    {
        return kWarpStatusDeviceCommunicationFailed;
    }

    return kWarpStatusOK;
}

void
printSensorDataINA219(bool hexModeFlag)
{
    uint16_t    readSensorRegisterValueLSB;
    uint16_t    readSensorRegisterValueMSB;
    int16_t     readSensorRegisterValueCombined;
    WarpStatus  i2cReadStatus;


    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

    i2cReadStatus = readSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219, 2 /* numberOfBytes */);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB); 
    // fit MSB by shifting 8 bits and read lsb without shifting

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint(" ----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint("Config 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
        }
        else
        {
            warpPrint(" %d,", readSensorRegisterValueCombined);
        }
    }

    i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Shunt_Voltage, 2 /* numberOfBytes */);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB); 
    // fit MSB by shifting 8 bits and read lsb without shifting

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint(" ----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint("shunt_v 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
        }
        else
        {
            warpPrint("shunt_v %d,", readSensorRegisterValueCombined);
        }
    }

    i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Bus_Voltage, 2 /* numberOfBytes */);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB); 
    // fit MSB by shifting 8 bits and read lsb without shifting
    //then shift right shift 3 bits for bus matching
    readSensorRegisterValueCombined = readSensorRegisterValueCombined >> 3;

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint(" ----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint("bus_v 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
        }
        else
        {
            warpPrint("bus_v %d,", readSensorRegisterValueCombined);
        }
    }

    // kWarpSensorOutputRegisterINA219Power
    i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Power, 2);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = (((int16_t)readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);



    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint(" ----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint("Power 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
        }
        else
        {
            warpPrint("Power %d,", readSensorRegisterValueCombined);
        }
    }

    i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Current, 2 /* numberOfBytes */);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB); 
    // fit MSB by shifting 8 bits and read lsb without shifting

    // Change hex current into dec and current LSB is 1e-5A, actual current is the multiple of these two
    float current_LSB = 1e-4; //in A
    float dec_current = current_LSB*readSensorRegisterValueCombined/1000; //in mA

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint(" ----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint("Current 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
        }
        else
        {
            warpPrint("Current %d,", readSensorRegisterValueCombined);
        }
    }

    i2cReadStatus = readSensorRegisterINA219(kWarpSensorCalibrationRegisterINA219, 2 /* numberOfBytes */);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB); 
    // fit MSB by shifting 8 bits and read lsb without shifting

    if (i2cReadStatus != kWarpStatusOK)
    {
        warpPrint(" ----,");
    }
    else
    {
        if (hexModeFlag)
        {
            warpPrint("Calibration 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
        }
        else
        {
            warpPrint("Calibration %d,", readSensorRegisterValueCombined);
        }
    }
}

uint8_t
appendSensorDataINA219()
{   
    uint16_t calreg = 20480;
    writeSensorRegisterINA219(kWarpSensorCalibrationRegisterINA219, calreg);

    uint16_t readSensorRegisterValueLSB;
    uint16_t readSensorRegisterValueMSB;
    int16_t readSensorRegisterValueCombined;
    WarpStatus i2cReadStatus;

    
    warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
    i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219_Current, 2 /* numberOfBytes */);
    readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
    readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
    readSensorRegisterValueCombined = ((int16_t)readSensorRegisterValueMSB << 8) | (readSensorRegisterValueLSB); 
    warpPrint("%d\n", readSensorRegisterValueCombined);


}
void
set_Calibration_reg()
{

    uint16_t calreg = 20480;
    // Set Calibration register to 'Cal' calculated above
    writeSensorRegisterINA219(kWarpSensorCalibrationRegisterINA219, calreg);
}


