void checkSerialPort(void *pvParameters) {
  int command;
  for (;;) {
    while (Serial.available() > 0) {
      command = Serial.read();
      if (command == 49) // ASCII value for '1'
        printLatestRecord();
      else if (command == 50) // ASCII value for '2'
        printAllRecords();
      else if (command == 51) { // ASCII value for '3'
        xSemaphoreGive(sleepSemaphore); // Give sleepSemaphore
      } 
    }
    vTaskDelay(500/portTICK_PERIOD_MS);
  }
}
