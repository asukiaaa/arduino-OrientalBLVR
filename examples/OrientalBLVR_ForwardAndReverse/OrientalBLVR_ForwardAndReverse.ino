#include <OrientalBLVR_asukiaaa.hpp>

#ifndef MOTOR_ADDRESS
#define MOTOR_ADDRESS 1
#endif
#ifndef MOTOR_BAUDRATE
#define MOTOR_BAUDRATE 230400
#endif
#ifndef RS485_SERIAL
#define RS485_SERIAL Serial1
#endif
#ifndef PIN_RS485_DE
#define PIN_RS485_DE 11
#endif
#ifndef PIN_RS485_RE
#define PIN_RS485_RE PIN_RS485_DE
#endif

OrientalBLVR_asukiaaa::Core motor(&RS485_SERIAL, MOTOR_ADDRESS, PIN_RS485_DE,
                                  PIN_RS485_RE);

void setup() {
  motor.begin(MOTOR_BAUDRATE);
  Serial.begin(9600);
}

void loop() {
  uint32_t alarm;
  auto result = motor.readAlarmU32t(&alarm);
  if (result != 0) {
    Serial.println("cannot read alarm because of error " + String(result));
  } else {
    Serial.println("alarm " + String(alarm));
  }
  Serial.println("forward");
  motor.writeSpeed32t(500);
  delay(2000);

  Serial.println("stop");
  motor.writeSpeed32t(0);
  delay(2000);

  Serial.println("reverse");
  motor.writeSpeed32t(-500);
  delay(2000);

  Serial.println("stop");
  motor.writeSpeed32t(0);
  delay(2000);
}
