#include <Arduino.h>
#include <Wire.h>
#include "at24c256.h"
//       _____
//  A0 -|  ^  |- VCC
//  A1 -|     |- WP(GND)
//  NC -|     |- SCL
// GND -|_____|- SDA

// I2C address is 0x50 = A0(GND), A0(GND)
//---------------------------------------------------------------------------------------------------------
at24c256::at24c256(uint8_t _i2c_addr) {
  m_chip_addr = _i2c_addr;                                   // I2C адрес на EEPROM
}
//---------------------------------------------------------------------------------------------------------
void at24c256::init() {                                      
  Wire.end(); delay(20); Wire.begin();Wire.setClock(400000); // set i2c frequency to 400000hz (eeprom speed)
}
//---------------------------------------------------------------------------------------------------------
void at24c256::close() {                                      
  Wire.end(); delay(20); Wire.begin();Wire.setClock(1000000); // reset i2c frequency to 1700000hz
}
//---------------------------------------------------------------------------------------------------------
void at24c256::write(uint16_t m_addr, uint8_t m_data) {      
  if (m_addr < 32768) {
    Wire.beginTransmission(m_chip_addr);
    Wire.write((uint8_t)(m_addr >> 8));
    Wire.write((uint8_t)m_addr);
    Wire.write(m_data);
    Wire.endTransmission();
    delay(5);
  }
}
//---------------------------------------------------------------------------------------------------------
void at24c256::update(uint16_t m_addr, uint8_t m_data) {     
  if (m_addr < 32768) {                                      
    uint8_t m_exist_data = read(m_addr);
    if (m_data != m_exist_data) {                            
      Wire.beginTransmission(m_chip_addr);
      Wire.write((uint8_t)(m_addr >> 8));
      Wire.write((uint8_t)m_addr);
      Wire.write(m_data);
      Wire.endTransmission();
      delay(5);
    }
  }
}
//---------------------------------------------------------------------------------------------------------
uint8_t at24c256::read(uint16_t m_addr) {                    // чете съдържанието на клетка с адрес m_addr
  uint8_t m_data = 0xFF;

  Wire.beginTransmission(m_chip_addr);
  Wire.write((uint8_t)(m_addr >> 8));
  Wire.write((uint8_t)m_addr);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)m_chip_addr, (uint8_t)1);
  if (Wire.available())
    m_data = Wire.read();
  return m_data;
}
