void EEPROM_Write(uint8_t addr, int8_t data)
{
  HWire.beginTransmission(eeprom_address);
  HWire.write(addr);
  HWire.write(data);
  HWire.endTransmission();
  delay(1);
}

int8_t EEPROM_Read(uint8_t addr)
{
  int8_t data = 0xFF;
  HWire.beginTransmission(eeprom_address);
  HWire.write(addr);
  HWire.endTransmission();
  HWire.requestFrom(eeprom_address, 1);

  if (HWire.available()) data = HWire.read();
  return data;
}
