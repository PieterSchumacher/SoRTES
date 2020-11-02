unsigned long sp = 0x0;
unsigned int record_size = sizeof(temperatureRecord);

#define TABLE_SIZE 1024

// --- AUXILLARY FUNCTIONS ---

void EEPROMReset() {
  EEPROM.put(0x0, (unsigned long) 4L);
}

void EEPROMInit() {
  sp = EEPROM.get(0x0, sp) + sizeof(long);
}

void printRecord(TemperatureRecord record) {
  Serial.print("Beacon details: ");
  Serial.println(record.beaconDetails);
  Serial.print("Temperature: ");
  Serial.println(record.temperature);
}

TemperatureRecord readRecord(unsigned long address) {
  return EEPROM.get(address, temperatureRecord);
}

void writeRecord(unsigned long address, TemperatureRecord data) {
  EEPROM.put(address, data);
  EEPROM.put(0x0, sp);
}

// --- REQUIRED COMMANDS ---

void addRecord(TemperatureRecord temperatureRecord) {
  xSemaphoreTake(stackSemaphore, portMAX_DELAY);
  writeRecord(sp, temperatureRecord);
  sp += record_size;
  xSemaphoreGive(stackSemaphore);
}

void printLatestRecord() {
  xSemaphoreTake(stackSemaphore, portMAX_DELAY);
  printRecord(readRecord(sp - record_size));
  xSemaphoreGive(stackSemaphore);
}

void printAllRecords() {
  xSemaphoreTake(stackSemaphore, portMAX_DELAY);
  for (unsigned long p = 0x4; p < sp; p += record_size) {
    TemperatureRecord record = readRecord(p);
    printRecord(record);
  }
  xSemaphoreGive(stackSemaphore);
}
