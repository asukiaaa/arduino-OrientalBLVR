#pragma once
#include <OrientalCommon_asukiaaa.hpp>
#include <OrientalCommon_asukiaaa/BLx.hpp>

namespace OrientalBLVR_asukiaaa {

namespace Register {
class DriveInfoSet {
 public:
  const uint16_t ModeH;
  const uint16_t ModeL;
  const uint16_t PositionH;
  const uint16_t PositionL;
  const uint16_t SpeedH;
  const uint16_t SpeedL;

  DriveInfoSet(uint16_t base)
      : ModeH(base),
        ModeL(base + 1),
        PositionH(base + 2),
        PositionL(base + 3),
        SpeedH(base + 4),
        SpeedL(base + 5) {}
};

const uint16_t DriveType = 0x0;
const uint16_t AlarmH = 0x0080;
const uint16_t AlarmL = 0x0081;
const uint16_t DirectDriveModeH = 0x005a;
const uint16_t DirectDriveModeL = 0x005b;
const uint16_t MotorControlH = 0x007c;
const uint16_t MotorControlL = 0x007d;
const uint16_t LoadSpeedH = 0x00ce;
const uint16_t LoadSpeedL = 0x00cd;
const uint16_t LoadTorqueH = 0x00d6;
const uint16_t LoadTorqueL = 0x00d7;
const DriveInfoSet DriveInfo0(0x1801);
const uint16_t RotationDirH = 0x0348;
const uint16_t RotationDirL = 0x0349;
const uint16_t ElectricBrakeH = 0x03ba;
const uint16_t ElectricBrakeL = 0x03bb;
const uint16_t AutoSOnH = 0x0e36;
const uint16_t AutoSOnL = 0x0e37;
}  // namespace Register

namespace DriveMode {
const uint16_t SlowDown = 0;
const uint16_t ConteniousSpeed = 16;
const uint16_t ExtendConteniousSpeed = 48;
}  // namespace DriveMode

struct Config {
  uint32_t rpmMax = 4000000;
};

const Config configDefault;

class Core : public OrientalCommon_asukiaaa::BLx::Base {
 public:
  rs485_asukiaaa::ModbusRtu::Central *modbus;

  Core(HardwareSerial *serial, uint8_t address, uint8_t dePin, uint8_t rePin,
       Config config = configDefault)
      : createdModbus(true), address(address), config(config) {
    modbus = new rs485_asukiaaa::ModbusRtu::Central(serial, dePin, rePin);
  }

  Core(rs485_asukiaaa::ModbusRtu::Central *modbus, uint8_t address,
       Config config = configDefault)
      : createdModbus(false),
        address(address),
        modbus(modbus),
        config(config) {}

  ~Core() {
    if (createdModbus) {
      delete modbus;
    }
  }

  void begin(unsigned long baudrate, unsigned long config = SERIAL_8E1) {
    modbus->begin(baudrate, config);
    modbus->msSilentInterval = getMsSilentInterval(baudrate);
    beginWithoutModbus();
  }

  void beginWithoutModbus() {
    writeSetupConfig();
    writeSpeed32t(0);
  }

  uint8_t writeForward() { return writeDirection(false); }
  uint8_t writeLock() {
    return modbus->writeRegisterBy16t(address, Register::ElectricBrakeL, 1);
  }
  uint8_t writeStop() {
    // SERIAL_DEBUG.println("write stop " + String(millis()));
    return writeSpeed32t(0);
    // return modbus->writeRegisterBy16t(address, Register::ElectricBrakeL, 0);
  }
  uint8_t writeReverse() { return writeDirection(true); }
  uint8_t writeSpeed32t(int32_t speed) {
    int32_t msChangeSpeed = 1000;
    int32_t torqueDiv10 = 2000;
    int32_t data[] = {
        speed == 0 ? DriveMode::SlowDown : DriveMode::ExtendConteniousSpeed,
        0,
        speed,
        msChangeSpeed,
        msChangeSpeed,
        torqueDiv10,
        1,
    };
    const uint16_t dataLen = sizeof(data) / sizeof(data[0]);
    return modbus->writeRegistersBy32t(address, Register::DirectDriveModeH,
                                       (uint32_t *)data, dataLen);
  }
  // uint8_t writeSpeed(uint16_t speed) {}

  uint8_t readAlarmU32t(uint32_t *alarm) {
    return modbus->readRegistersBy32t(address, Register::AlarmH, alarm, 1);
  }

  uint8_t readLoadTorque(int32_t *torque) {
    return modbus->readRegistersBy32t(address, Register::LoadTorqueH,
                                      (uint32_t *)torque, 1);
  }

  uint8_t readLoadTorquePercent(float *torquePercent) {
    int32_t torque;
    auto result = readLoadTorque(&torque);
    if (result == 0) {
      *torquePercent = (float)torque / 10;
    }
    return result;
  }

  uint8_t readFeedbackSpeed32t(int32_t *speed) {
    return modbus->readRegistersBy32t(address, Register::LoadSpeedH,
                                      (uint32_t *)speed, 1);
  }
  uint8_t writeSetupConfig() {
    return modbus->writeRegisterBy16t(address, Register::MotorControlL,
                                      motorControlConfigLToWrite);
  }
  uint8_t writeSetupConfigIfNeeded() {
    uint16_t u16Config;
    auto result = modbus->readRegistersBy16t(address, Register::MotorControlL,
                                             &u16Config, 1);
    if (result != 0) {
      return result;
    }
    if (u16Config != motorControlConfigLToWrite) {
      return writeSetupConfig();
    }
    return 0;
  }

  rs485_asukiaaa::ModbusRtu::Central *getModbus() { return modbus; };
  uint32_t getRpmMin() { return 0; };
  uint32_t getRpmMax() { return config.rpmMax; };

 private:
  const bool createdModbus;
  uint8_t address;
  Config config;
  const uint16_t motorControlConfigLToWrite = 1;

  unsigned long getMsSilentInterval(unsigned long baudrate) {
    return baudrate >= 19200 ? 3 : 5;
  }

  uint8_t writeDirection(bool reverse) {
    uint16_t dir = reverse ? 1 : 0;
    return modbus->writeRegistersBy16t(address, Register::RotationDirL, &dir,
                                       1);
  }
};

}  // namespace BLV_R_asukiaaa
