// RTOS Tasks for various modes and periphials

#include "evaluateValves.h"

#include "Flora.h"



void blinkBluePixel(int pixel, int count, int time);
int sendMQTTBlueTooth(String MacAddress, int temperature, int moisture, int brightness, int conductivity,  int battery,  int readCount);




void taskFetchInfrared( void * parameter)
{
  // Enter RTOS Task Loop
  for (;;)  {

    if (uxSemaphoreGetCount( xSemaphoreReadInfrared ) > 0)
    {
      xSemaphoreTake( xSemaphoreUseI2C, 10000);

      ReadAMG8833Sensor();


      sendMQTT(MQTTINFRARED, "");
      xSemaphoreGive( xSemaphoreUseI2C);

    }

    vTaskDelay(sensorCycle * 1000 / portTICK_PERIOD_MS);
  }

}


void taskReadSendHydroponics( void * parameter)
{
  // Enter RTOS Task Loop
  for (;;)  {

    if (uxSemaphoreGetCount( xSemaphoreHydroponicsReadSensor ) > 0)
    {

      if (HydroponicsMode == 1)
      {
        xSemaphoreTake( xSemaphoreSensorsBeingRead, 20000);
        readHydroponicsSensors();
        sendMQTT(MQTTHYDROPONICS, "");
        xSemaphoreGive( xSemaphoreSensorsBeingRead);
      }

    }

    //vTaskDelay(600000 / portTICK_PERIOD_MS);
    vTaskDelay(sensorCycle * 1000 / portTICK_PERIOD_MS);
    //vTaskDelay(5000 / portTICK_PERIOD_MS);
  }

}

void taskReadSendHydroponicsLevel( void * parameter)
{
  // Enter RTOS Task Loop
  for (;;)  {

    if (uxSemaphoreGetCount( xSemaphoreHydroponicsLevelReadSensor ) > 0)
    {
      if (HydroponicsLevelMode == 1)
      {
        xSemaphoreTake( xSemaphoreSensorsBeingRead, 20000);
        readHydroponicsLevelSensor();
        sendMQTT(MQTTHYDROPONICSLEVEL, "");
        xSemaphoreGive( xSemaphoreSensorsBeingRead);
      }
    }
    //vTaskDelay(600000 / portTICK_PERIOD_MS);
    vTaskDelay(sensorCycle * 1000 / portTICK_PERIOD_MS);
  }

}



void taskSetValves( void * parameter)
{
  // Enter RTOS Task Loop
  for (;;)  {

    if (uxSemaphoreGetCount( xSemaphoreEvaluateValves ) > 0)
    {
      xSemaphoreTake( xSemaphoreEvaluatingValves, 10000);
      xSemaphoreTake( xSemaphoreSensorsBeingRead, 10000);
      evaluateValves(100);
      xSemaphoreGive( xSemaphoreSensorsBeingRead);
      xSemaphoreGive( xSemaphoreEvaluatingValves);
      //vTaskDelay(50 / portTICK_PERIOD_MS);

    }
    vTaskDelay(100 / portTICK_PERIOD_MS);

  }

}







void taskRESTCommand( void * parameter)
{
  // Enter RTOS Task Loop
  for (;;)  {

    if (uxSemaphoreGetCount( xSemaphoreRESTCommand ) > 0)
    {


      xSemaphoreTake( xSemaphoreSensorsBeingRead, 10000);
      // Handle REST calls
      WiFiClient client = MyServer.available();

      int timeout;
      timeout = 0;
      if (client)
      {
        /*
          Serial.print("SA client=");
          Serial.println(client);

          Serial.print("SA client.available()=");
          Serial.println(client.available());

          // Thank you to MAKA69 for this suggestion
          while (!client.available()) {
          Serial.print(".");
          vTaskDelay(10 / portTICK_PERIOD_MS);
          timeout++;
          Serial.print("IL client.available()=");
          Serial.println(client.available());
          if (timeout > 1000) {
            Serial.println("INFINITE LOOP BREAK!");
            break;
          }
          }

          Serial.print("AI client.available()=");
          Serial.println(client.available());
        */
        if (client.available())
        {



          rest.handle(client);

        }
      }
      client.stop();
      xSemaphoreGive( xSemaphoreSensorsBeingRead);

      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }


  }

}

// Pixel Task and routines



void taskPixelCommand( void * parameter)
{
  int i = 0;


  // Enter RTOS Task Loop
  for (;;)  {

    xSemaphoreTake( xSemaphorePixelPulse, 10000);


    for (i = 5; i < 100; i++)
    {

      strip.SetPixelColor(0, RgbwColor(i, 0, 0, 0));

      strip.Show();
      vTaskDelay(15 / portTICK_PERIOD_MS);
      xSemaphoreGive( xSemaphorePixelPulse);
      xSemaphoreTake( xSemaphorePixelPulse, 10000);

    }
    xSemaphoreGive( xSemaphorePixelPulse);
    xSemaphoreTake( xSemaphorePixelPulse, 10000);

    for (i = 100; i > 5; i--)
    {
      strip.SetPixelColor(0, RgbwColor(i, 0, 0, 0));
      strip.Show();
      vTaskDelay(15 / portTICK_PERIOD_MS);
      xSemaphoreGive( xSemaphorePixelPulse);
      xSemaphoreTake( xSemaphorePixelPulse, 10000);
    }


    xSemaphoreGive( xSemaphorePixelPulse);
    vTaskDelay(10 / portTICK_PERIOD_MS);

  }

}




void taskMainLCDLoopDisplay(void * parameter)
{

  // Enter RTOS Task Loop
  for (;;)  {

    if (uxSemaphoreGetCount( xSemaphoreOLEDLoopUpdate ) > 0)
    {


      //Serial.println("--------->Entering Display Loop");

      updateDisplay(DISPLAY_IPNAMEID);
      vTaskDelay(3000 / portTICK_PERIOD_MS);

      updateDisplay(DISPLAY_STATUS);
      vTaskDelay(3000 / portTICK_PERIOD_MS);


      int i;

      for (i = 0; i < MAXBLUETOOTHDEVICES; i++)
      {

        if (BluetoothAddresses[i] != "")
        {
          updateDisplay(DISPLAY_BLUETOOTH + i);
          vTaskDelay(3000 / portTICK_PERIOD_MS);
        }


      }

      // display ADC sensors

      if (HydroponicsMode == 1)
      {
        for (i = 0; i < 4; i++)
        {


          updateDisplay(DISPLAY_MOISTURE_1 + i);
          vTaskDelay(3000 / portTICK_PERIOD_MS);

        }

      }


      //Serial.println("<-----------OutOfDisplay");

    }


    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}






void taskKeepMQTTAlive( void * parameter)
{
  // Enter RTOS Task Loop
  for (;;)  {

    if (uxSemaphoreGetCount( xSemaphoreKeepMQTTAlive ) > 0)
    {

      if (!MQTTclient.connected()) {
        MQTTreconnect(false);
        Serial.println("Task MQTT Alive reconnected");
      }

      MQTTclient.loop();
      vTaskDelay(500 / portTICK_PERIOD_MS);


    }
    else
    {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }


  }

}
