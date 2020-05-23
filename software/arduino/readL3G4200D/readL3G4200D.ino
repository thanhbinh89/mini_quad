#include <SPI.h>
#include "L3G4200D.h"

volatile bool dataAvailable = false;
const int dataReadyPin = 2;
const int chipSelectPin = 10;

void setup() {
  Serial.begin(115200);
  SPI.begin();

  pinMode(dataReadyPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(dataReadyPin), interruptCallback, RISING);

  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);

  delay(100);

  Serial.print("WHO: 0x");
  Serial.println(readRegister(L3G4_WHO_AM_I), HEX);

  byte ctrl1 = 0x00;
  byte ctrl2 = 0x00;
  byte ctrl3 = 0x00;
  byte ctrl4 = 0x00;
  byte ctrl5 = 0x00;
  ctrl1 |= (byte) (L3G4_ODR_100 | L3G4_BW_0 | L3G4_NOR_SLE_MOD | L3G4_XYZEN);
  ctrl2 |= (byte) (L3G4_HPM_NOR_RES | L3G4_HPCF_9);
  ctrl3 |= (byte) (L3G4_INT1_DIS | L3G4_BOOT_INT1_DIS | L3G4_INT1_ACT_H | L3G4_INT1_PP | L3G4_DRDY_INT2_ENA | L3G4_WTM_INT2_DIS | L3G4_ORUN_INT2_DIS | L3G4_EMP_INT2_DIS);
  ctrl4 |= (byte) (L3G4_BDU_CONTINOUS | L3G4_BLE_LSB | L3G4_FS_2000 | L3G4_ST_NOR | L3G4_SPI_4_WIRE);
  ctrl5 |= (byte) (L3G4_BOOT_NOR | L3G4_FIFO_DIS | L3G4_HPF_ENA | L3G4_INT1_SEL_3 | L3G4_OUT_SEL_3);
  writeRegister(L3G4_CTRL_REG1, ctrl1);
  writeRegister(L3G4_CTRL_REG2, ctrl2);
  writeRegister(L3G4_CTRL_REG3, ctrl3);
  writeRegister(L3G4_CTRL_REG4, ctrl4);
  writeRegister(L3G4_CTRL_REG5, ctrl5);

  delay(100);

  Serial.print("CRTL1: ");
  Serial.println((ctrl1 == readRegister(L3G4_CTRL_REG1)) ? "OK" : "ERROR");
  Serial.print("CRTL2: ");
  Serial.println((ctrl2 == readRegister(L3G4_CTRL_REG2)) ? "OK" : "ERROR");
  Serial.print("CRTL3: ");
  Serial.println((ctrl3 == readRegister(L3G4_CTRL_REG3)) ? "OK" : "ERROR");
  Serial.print("CRTL4: ");
  Serial.println((ctrl4 == readRegister(L3G4_CTRL_REG4)) ? "OK" : "ERROR");
  Serial.print("CRTL5: ");
  Serial.println((ctrl5 == readRegister(L3G4_CTRL_REG5)) ? "OK" : "ERROR");
  Serial.print("TEMP: ");
  Serial.println(readRegister(L3G4_OUT_TEMP), DEC);

  readRegister(L3G4_OUT_X_L);
  readRegister(L3G4_OUT_X_H);
}

void loop() {

  if (dataAvailable) {
    dataAvailable = false;

    byte outXL = readRegister(L3G4_OUT_X_L);
    byte outXH = readRegister(L3G4_OUT_X_H);

    //    byte outYL = readRegister(L3G4_OUT_Y_L);
    //    byte outYH = readRegister(L3G4_OUT_Y_H);
    //
    //    byte outZL = readRegister(L3G4_OUT_Z_L);
    //    byte outZH = readRegister(L3G4_OUT_Z_H);

    Serial.println(L3G4_MAKE_OUT16(outXL, outXH), DEC);
    //    Serial.print(',');
    //    Serial.print(L3G4_MAKE_OUT16(outYL, outYH), DEC);
    //    Serial.print(',');
    //    Serial.println(L3G4_MAKE_OUT16(outZL, outZH), DEC);
  }
}

void interruptCallback() {
  dataAvailable = true;
}

byte readRegister(byte thisRegister) {
  byte result = 0;
  byte dataToSend = 0x80 | thisRegister;

  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(dataToSend);
  result = SPI.transfer(0x00);
  digitalWrite(chipSelectPin, HIGH);

  return (result);
}

void writeRegister(byte thisRegister, byte thisValue) {
  byte dataToSend = 0x00 | thisRegister;

  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(dataToSend);
  SPI.transfer(thisValue);
  digitalWrite(chipSelectPin, HIGH);
}
