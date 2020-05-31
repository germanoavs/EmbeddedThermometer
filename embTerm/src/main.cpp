#include<Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include<DallasTemperature.h>
#include<OneWire.h>


// define two Tasks for DigitalRead & AnalogRead
void Task0( void *pvParameters );
void Task1( void *pvParameters );
void Task2( void *pvParameters );

// semaphore
SemaphoreHandle_t xBufferSemaphore;

#define DS18B20 7 //DEFINE O PINO DIGITAL UTILIZADO PELO SENSOR
#define BUFFER_SIZE 10

uint8_t BUFFER[BUFFER_SIZE];
uint8_t bfindex = 0;
uint8_t media;

OneWire ourWire(DS18B20);
DallasTemperature sensors(&ourWire);

// the setup function runs once when you press reset or power the board
void setup() {

  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  sensors.begin();

    if ( xBufferSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xBufferSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Buffer
    if ( ( xBufferSemaphore ) != NULL )
      xSemaphoreGive( ( xBufferSemaphore ) );  // Make the Buffer available for use, by "Giving" the Semaphore.
  }
  

  // Now set up two Tasks to run independently.
  xTaskCreate(
    Task0
    ,  "ReadSensor"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    Task1
    ,  "CalMedia"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );

  xTaskCreate(
    Task2
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

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void Task0( void *pvParameters __attribute__((unused)) )  // This is a Task.
{


  for (;;) 
  {
    if ( xSemaphoreTake( xBufferSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      while (bfindex<BUFFER_SIZE)
      {
        BUFFER[bfindex] = sensors.getTempCByIndex(0);
        bfindex++;
      }
      xSemaphoreGive( xBufferSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability 
  }
}

void Task1( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    if ( xSemaphoreTake( xBufferSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    { 
      uint32_t sum = 0;
      uint8_t i = BUFFER_SIZE-1;

      while (i>=0)
      {
        sum+= BUFFER[i];
        i--;
      }
      
      media = sum/BUFFER_SIZE;
      bfindex=0;

      xSemaphoreGive( xBufferSemaphore ); // Now free or "Give" the Serial Port for others.
    }

    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability

  }
}

void Task2( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    Serial.print("Media Temperatura: "); //IMPRIME O TEXTO NA SERIAL
    Serial.print(media); //IMPRIME NA SERIAL O VALOR DE TEMPERATURA MEDIDO
    Serial.println("*C"); //IMPRIME O TEXTO NA SERIAL

    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability

  }
}