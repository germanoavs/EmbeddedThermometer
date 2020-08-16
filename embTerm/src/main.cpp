/**
 * @mainpage Embedded Thermometer
 * @author Germano Sobroza
 * 1. This project works with the use of an Arduino Uno board, temperature sensor with a serial module and a push-button.
 * 2. The temperature sensor is read in real time and the measurements are sent to the terminal and memory tasks.
 * 3. If the push-button is pressed, an interruption is triggered and the averaged of the measures stored in memory are calculated 
 *  and sent to the terminal task.
 */
#include<Arduino.h>
#include <Arduino_FreeRTOS.h>
#include<DallasTemperature.h>
#include<OneWire.h>
#include <EEPROM.h>
#include <queue.h>
#include <stdio.h>


void sensorTask( void *pvParameters );
void terminalTask( void *pvParameters );
void eepromTask( void *pvParameters );
void buttonInterrupt();
float CalcMedia();
void clearEEPROM();

#define DS18B20 7                             ///< Define the digital pin used by the sensor
#define MEMORY_SIZE 1024/sizeof(float)        ///< Memory lenght

const byte interruptPin = 2;                  ///< Interrup pin used by the button
int memory_index = -1*sizeof(float);          ///< Memory index variable, initialized with -1*sizeof(float) for implentation purposes 
bool FULL_MEMORY = false;                     ///< when all positions have been written with some temperature, turn into true

OneWire ourWire(DS18B20);
DallasTemperature sensors(&ourWire);
QueueHandle_t queue_1;
QueueHandle_t queue_2;

/**
 * @brief The setup() function intialize the serial monitor, the DallasTemperature sensor, configure the interrupt pin,
 * clears the EEPROM memory, creates two queues and three tasks.
 * 
*/
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

  // Set up Tasks to run independently.
  xTaskCreate(
    sensorTask
    ,  "LeSensor"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    terminalTask
    ,  "Terminal"
    ,  128  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL );

  xTaskCreate(
    eepromTask
    ,  "Memória"
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


/**
* @brief sensorTask() reads the temperature from the sensor, convert it to a String 
* type and send it througth queue_1 to terminalTask(). After that, queue_2 sends the float
* value of the temperature to eepromTask. 
*
*/
void sensorTask( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
    float current_temperature;                ///< stores the read temperature

   while(1) 
  {
    sensors.requestTemperatures();                          
    current_temperature = sensors.getTempCByIndex(0);               // get the last temperature
    String mensagem = "Temperatura Sensor: " + (String)current_temperature;         // cast float to String

    xQueueSend(queue_1, &mensagem, portMAX_DELAY);                  // sends a char pointer throught queue_1
    xQueueSend(queue_2, &current_temperature, portMAX_DELAY);       // sends a float throught queue_2

    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability 
  }
}

/**
* @brief terminalTask() waits for messages sent thought queue_2 and print
* it in the terminal.
*/
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


/**
* @brief eepromTask() reiceves float temperature and writes in the EEPROM, addressed
* by memory_index. If memory_index is bigger than MEMORY_SIZE, memory_indez is set to
* -sizeof(float), working like a circular buffer.
*/
void eepromTask( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
    float temperature = 0.0;

   while(1) {
      if (xQueueReceive(queue_2, &temperature, portMAX_DELAY) == pdPASS) {
        memory_index += sizeof(float);

        if(memory_index > (MEMORY_SIZE - sizeof(float))){
           memory_index = -4;
           FULL_MEMORY = true;
        }

        EEPROM.put(memory_index, temperature);            // writes temperature in memory_index address
     }
    }
  
}

/**
* @brief CalMedia() is called by the buttonInterrupt() and calculates the mean
* of all temperatures wroten on the EEPROM
*/
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

/**
* @brief buttonInterrupt() deals with an interrupt generated by an external button. The ISR calls
* CalMedia function and send the mean the temperature throught queue_2
*/
void buttonInterrupt(){

  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200){
    float media = CalcMedia();
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    String mensagem = "Média: " + (String)media;
    xQueueSendFromISR( queue_1, &mensagem, &xHigherPriorityTaskWoken);

    // request a context switch from ISR
    portYIELD_FROM_ISR();
  }
  last_interrupt_time = interrupt_time;
}

/**
* @brief clearEEPROM() set all EEPROM bits to 0
*/
void clearEEPROM(){
  for (uint16_t i = 0 ; i <= MEMORY_SIZE ; i++) {
    EEPROM.put(i, 0);
  }
}
