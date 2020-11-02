// Takes a TemperatureRecord and transmits it's temperature value via LoRa
void transmitTemperature(TemperatureRecord temperatureRecord) {
  if (! LoRa.begin(BAND, PABOOST)) {
    return; // if LoRa does not start, return gewoon
  }
  LoRa.beginPacket();
  LoRa.print(temperatureRecord.temperature);
  LoRa.endPacket();
  LoRa.end();
}

void waitAndReceive(void *pvParameters) {
  int nextDelay = 0;
  for (;;) {
    xQueueReceive(delayQueueToReceive, &nextDelay, portMAX_DELAY);
    vTaskDelay((nextDelay*1000 - 200)/portTICK_PERIOD_MS); // wacht voor 200ms minder dan nextDelay seconden
    //Serial.println("gewacht nu");
    //digitalWrite(LED_BUILTIN, on = 1 - on);
    if (LoRa.begin(BAND, PABOOST))
      LoRa.receive();
    else
      Serial.println("Could not start LoRa");
  }
}

void readGatewayOnReceive(int packetSize) {
  if (packetSize == 0) return; // geen packet, return gewoon
  char read_buffer[3];
  int beacon_details, i;
  for (int i=0;i<4;i++) {
    LoRa.read(); // skip eerste 4 bytes
  }
  while (LoRa.available()) {
    read_buffer[i++] = (char) LoRa.read();
  }
  int is_decimal = sscanf(read_buffer, "%d", &beacon_details);
  if (is_decimal) {
    xQueueSendFromISR(delayQueueToConversion, &beacon_details, NULL);
    xQueueSendFromISR(delayQueueToReceive, &beacon_details, NULL);
  }
  LoRa.end();
}
