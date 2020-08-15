#include<Arduino.h>
#include <Arduino_FreeRTOS.h>
#include<DallasTemperature.h>
#include<OneWire.h>
#include <EEPROM.h>
#include <queue.h>
#include <stdio.h>


// define two Tasks for DigitalRead & AnalogRead
void sensorTask( void *pvParameters );
void terminalTask( void *pvParameters );
void eepromTask( void *pvParameters );
void buttonInterrupt();
float CalcMedia();
void clearEEPROM();
char *ftoa(char *a, double f, int precision);

#define DS18B20 7 //DEFINE O PINO DIGITAL UTILIZADO PELO SENSOR
#define MEMORY_SIZE 1024/sizeof(float)
#define MEMORY_FLAG ((float)0xffff)
const byte interruptPin = 2;
int memory_index = -1*sizeof(float);
bool FULL_MEMORY = false;

OneWire ourWire(DS18B20);
DallasTemperature sensors(&ourWire);
QueueHandle_t queue_1;
QueueHandle_t queue_2;

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  sensors.begin();

  // clear EEPROM, set all bits to 0;
  clearEEPROM();

  // set interrupt pin
  pinMode(interruptPin, INPUT_PULLUP);

  // set interrupt
  attachInterrupt(digitalPinToInterrupt(interruptPin), buttonInterrupt, CHANGE);

  // create queue's
  queue_1 = xQueueCreate(6, sizeof(char *));
  queue_2 = xQueueCreate(3, sizeof(float));

  if (queue_1 == NULL || queue_2 == NULL) {
    Serial.println("Queue can not be created");
  }

  // Now set up two Tasks to run independently.
  xTaskCreate(
    sensorTask
    ,  "LeSensor"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    terminalTask
    ,  "CalcMedia"
    ,  128  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL );

  xTaskCreate(
    eepromTask
    ,  "PrintMedia"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );   

    delay(1000);

  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
}

void loop()
{
  // Empty. Things are done in Tasks.
}


/////////////////////////  SENSOR TASK ///////////////////////////////////////////////////////
void sensorTask( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
    float current_temperature;

   while(1) 
  {
    sensors.requestTemperatures();            //SOLICITA QUE A FUNÇÃO INFORME A TEMPERATURA DO SENSOR
    current_temperature = sensors.getTempCByIndex(0);
    String mensagem = "Temperatura Sensor: " + (String)current_temperature;

    xQueueSend(queue_1, &mensagem, portMAX_DELAY);
    xQueueSend(queue_2, &current_temperature, portMAX_DELAY);

    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability 
  }
}

/////////////////////////  TERMINAL TASK ///////////////////////////////////////////////////////
void terminalTask( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
   char *mensagem;

  while(1) {
    if (xQueueReceive(queue_1, &mensagem, portMAX_DELAY) == pdPASS) {
      Serial.print(mensagem);
      Serial.print("\n--------------------------\n");
    }
    
  }

}

/////////////////////////  EEPROM TASK ///////////////////////////////////////////////////////
void eepromTask( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
    float temperature = 0.0;

   while(1) {
      if (xQueueReceive(queue_2, &temperature, portMAX_DELAY) == pdPASS) {
        memory_index += sizeof(float);
        EEPROM.put(memory_index, temperature);

        if(memory_index > MEMORY_SIZE){
           memory_index = -4;
           FULL_MEMORY = true;
        }
     }
    }
  
}

//////////////////////////////// INTERRUPT TASK MEDIA ///////////////////////////////////////
float CalcMedia(){

  int i = memory_index;
  float sum = 0;
  float temp;

  if(FULL_MEMORY)
    i = MEMORY_SIZE;
  
  for (; i >= 0;)
  { 
    EEPROM.get(i, temp);
    
    sum += temp;  
     i -= (sizeof(float));
  }
  

  if (FULL_MEMORY)
    return sum/MEMORY_SIZE;
  else{
    return sum/((memory_index/sizeof(float))+1);
  }

}

void buttonInterrupt(){

  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200){
  float media = CalcMedia();
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  String mensagem = "Média: " + (String)media;
  //Serial.print(mensagem);
  xQueueSendFromISR( queue_1, &mensagem, &xHigherPriorityTaskWoken);

  portYIELD_FROM_ISR();
  }
  last_interrupt_time = interrupt_time;
}

// EEPROM CLEAR

void clearEEPROM(){
  for (uint16_t i = 0 ; i <= MEMORY_SIZE ; i++) {
    EEPROM.put(i, 0);
  }
}
