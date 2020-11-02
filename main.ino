#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <SPI.h>
#include <LoRa.h>
#include <avr/sleep.h>
#include <EEPROM.h>
#include <queue.h>
#include <avr/wdt.h>

#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    869500000  // 915E6
#define PABOOST true

#define WAKEUPPIN 2

#define AMOUNTOFBEACONS 20

struct TemperatureRecord {
  int beaconDetails;
  int temperature;
} temperatureRecord;

void readGatewayTransmission(void *pvParameters);
void enableADCAndStartConversion(void *pvParameters);
void handleData(void *pvParameters);
void waitAndReceive(void *pvParameters);
void checkSerialPort(void *pvParameters);
void deepSleep(void *pvParameters);

void vApplicationIdleHook(void); // Idle task hook

int tickCount = 0;
TemperatureRecord tr;

SemaphoreHandle_t sleepSemaphore;
SemaphoreHandle_t stackSemaphore;
QueueHandle_t temperatureQueue;
QueueHandle_t delayQueueToConversion;
QueueHandle_t delayQueueToReceive;

void setup() {
  wdt_disable();
  Serial.begin(9600);
  analogReference(INTERNAL);

  EEPROMReset();

  pinMode(WAKEUPPIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WAKEUPPIN), wakeUpFromDeepSleep, LOW);

  // Power init
  SMCR = 0;
  PRR0 = 0;
  PRR1 = 0;
  SMCR |= _BV(SE);     // enable SLEEP instruction
  ACSR |= _BV(ACD);    // Disable analog comparator
  PRR0 |= _BV(PRTWI);  // Disable TWI (Two Wire Interface)
  PRR0 |= _BV(PRTIM1); // Disable Timer1
  PRR1 |= 1 << 4;      // Disable Timer4
  PRR1 |= _BV(PRTIM3); // Disable Timer3
  PRR1 |= _BV(PRUSART1); // Disable USART

  // LoRa init
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND, PABOOST)) {
    Serial.println("ERROR");
  }
  LoRa.onReceive(readGatewayOnReceive);
  LoRa.receive();
  
  ADCSRA = 0b00000000;    // initialise the ADC completly off
  ADCSRB |= (1 << MUX5);  // Bit nodig om Temp Sensor te selecteren
  ADMUX = 0b11000111;     // meest significante bits: selecteer juiste voltage, minst significante: MUX bits voor selecteren temp sensor

  //FreeRTOS init
  vSemaphoreCreateBinary(sleepSemaphore);  // Create semaphore
  vSemaphoreCreateBinary(stackSemaphore); // Create semaphore
  temperatureQueue = xQueueCreate(2, sizeof(temperatureRecord));
  delayQueueToConversion = xQueueCreate(2, sizeof(int));
  delayQueueToReceive = xQueueCreate(2, sizeof(int));

  xSemaphoreGive(stackSemaphore);  // give semaphore

  if (sleepSemaphore == NULL); // Semaphore creation failed, do something about it

  //xTaskCreate(readGatewayTransmission, "readTransmission", 128, NULL, 3, NULL);
  xTaskCreate(enableADCAndStartConversion, "Temp", 128, NULL, 2, NULL);
  xTaskCreate(handleData, "DataHandler", 128, NULL, 2, NULL);
  xTaskCreate(waitAndReceive, "waitToReceive", 128, NULL, 3, NULL);
  xTaskCreate(checkSerialPort, "serialCommunication", 128, NULL, 1, NULL);
  xTaskCreate(deepSleep, "sleeper", 128, NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop() { 
}

void ADC_SLEEP() {
  cli();
  SMCR |= _BV(SM0);   // ADC noice reduction mode
  SMCR |= _BV(SE);    // enable SLEEP instruction
  //SMCR = 0b11;      // Enter ADC noice reduction and enable SLEEP
  sei();
  sleep_mode();
}

ISR(ADC_vect) {
  ADCSRA = 0;       // shut down ADC
  SMCR = 0;         // disable SLEEP mode
  
  tr.temperature = readConversionResult();

  BaseType_t higherProcessWoken;      // requires fix

  higherProcessWoken = pdFALSE;

  xQueueSendFromISR(temperatureQueue, &tr, &higherProcessWoken);
  if (higherProcessWoken == pdTRUE) {
    // Do a context switch
  }
}

void enableADCAndStartConversion(void *pvParameters) {
  int delayToUse = 0;
  for (;;) {
    xQueueReceive(delayQueueToConversion, &delayToUse, portMAX_DELAY);
    // vTaskDelay(1000/portTICK_PERIOD_MS);
    tr.beaconDetails = delayToUse;
    ADCSRA |= _BV(ADEN);                  // enable ADC
    delay(5);                             // wait for voltages to stabilise
    ADCSRB |= (1 << MUX5);                // MUX bit to select temperature sensor
    ADMUX = 0b11000111;                   // MSB: select correct voltage reference values. LSB: MUX bits to select temperature sensor
    ADCSRA |= _BV(ADIE);                  // Enable ADC interrupts
    ADC_SLEEP();                          // Go to sleep, start conversion
  }
}

void handleData(void *pvParameters) {
  //xSemaphoreTake(dataSemaphore, 0);                               // Poll semaphore, if available, take
  TemperatureRecord tRecord;
  for (;;) {
    if (xQueueReceive(temperatureQueue, &tRecord, portMAX_DELAY) == pdPASS) {
      addRecord(tRecord);                                             // write current temperature to EEPROM
      transmitTemperature(tRecord);                                   // broadcast current temperature via LoRa
      tickCount++;
      if (tickCount >= AMOUNTOFBEACONS) {
        xSemaphoreGive(sleepSemaphore);
      }
    }
  }
}

void vApplicationIdleHook() {
  cli();
  SMCR |= _BV(SM0);
  SMCR &= ~ _BV(SM1);  // set bit to 0
  SMCR &= ~ _BV(SM2);  // set bit to 0
//  
//  PRR0 |= _BV(PRTIM0); // Disable Timer0 -> nodig for something
//  PRR0 |= _BV(PRSPI);  // Disable SPI -> nodig voor LoRa
  sei();
  sleep_mode();
}

void deepSleep(void *pvParameters) {
  xSemaphoreTake(sleepSemaphore, 0);
  for (;;) {
    xSemaphoreTake(sleepSemaphore, portMAX_DELAY);
    vTaskEndScheduler();
    cli();
    SMCR = 0;
    SMCR |= 1;           // enable SLEEP instruction
    SMCR |= _BV(SM1);    // POWER DOWN mode
    
    PRR0 |= _BV(PRTIM0); // Disable Timer0
    PRR0 |= _BV(PRSPI);  // Disable SPI
    ADCSRA = 0;          // Disable ADC
    PRR0 |= _BV(PRADC);  // Disable ADC
    sei();
    asm volatile("SLEEP");
  }
}

void wakeUpFromDeepSleep() { // needs updates
  if (tickCount < AMOUNTOFBEACONS) return;  // return, interrupt was a misfire
  //digitalWrite(LED_BUILTIN, on = 1 - on);
//  // Enable Timer0 and SPI
  PRR0 &= ~ _BV(PRTIM0); // ~ = bitwise not
  PRR0 &= ~ _BV(PRSPI);
  PRR0 &= ~ _BV(PRADC);  // enable ADC
//  //xTaskResumeAll();
  tickCount = 0;
  sei();
  wdt_enable(WDTO_60MS);
  for (;;);
}

// --- AUXILLARY FUNCTIONS ---
float readConversionResult() {
  return (ADCW - 273) / 2.0;
}
