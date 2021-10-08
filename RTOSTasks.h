// RTOS Tasks for various modes and periphials

#include "evaluateValves.h"

#include "Flora.h"

FlowerCare* flowerCare;
// static FlowerCare* flowerCare;

void blinkBluePixel(int pixel, int count, int time);

int sendMQTTBlueTooth(std::string MacAddress, RealTimeEntry *myRealTimeEntry, int battery, std::string firmware, int readCount);

int deviceCount = 2;

// array of different xiaomi flora MAC addresses
char* FLORA_DEVICES[] = {
  "c4:7c:8d:6c:21:b8",
  "c4:7c:8d:6b:90:39",
  "c4:7c:8d:6c:9c:88"


};

void taskReadBluetooth( void * parameter)
{



  //flowerCare = new FlowerCare();
  int readCount = 0;
  long firstHeap, lastHeap;

  // Enter RTOS Task Loop
  for (;;)  {

    if (uxSemaphoreGetCount( xSemaphoreReadBluetooth ) > 0)
    {

      blinkBluePixel(0, 1, 150);
      Serial.print("Before FreeHeap=");
      firstHeap = ESP.getFreeHeap();
      Serial.println(firstHeap);

      // If the flag "doConnect" is true then we have scanned for and found the desired
      // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
      // connected we set the connected flag to be true.
      if (doConnect == true) {
        flowerCare->connectToServer();
        doConnect = false;
      }

      //try
      {

        if (connected == true)
        {
          Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>1 connected=true =");
          Serial.println(ESP.getFreeHeap());

          Serial.printf("Name: %s\n", flowerCare->getName().c_str());
          std::string myMacAddress = flowerCare->getMacAddress();
          Serial.printf("Mac address: %s\n", myMacAddress.c_str());
          int batterylevel = flowerCare->getBatteryLevel();

          Serial.printf("Battery level: %d %%\n", batterylevel);
          RealTimeEntry *myRealTimeEntry;
          std::string firmware;

          firmware = flowerCare->getFirmwareVersion();
          Serial.printf("Firmware version: %s\n", firmware.c_str());


          myRealTimeEntry = flowerCare->getRealTimeData();

          Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2  connected=true = ");
          Serial.println(ESP.getFreeHeap());

          Serial.printf("Real time data: \n%s\n", myRealTimeEntry->toString().c_str() );


          //Serial.printf("Historical data: \n");
          //int epochTimeInSeconds = flowerCare->getEpochTimeInSeconds();
          //Serial.printf("Epoch time: %d seconds or ", epochTimeInSeconds);
          //flowerCare->printSecondsInDays(epochTimeInSeconds);
          //std::vector<HistoricalEntry> historicalEntries = flowerCare->getHistoricalData();
          //for(std::vector<HistoricalEntry>::size_type i = 0; i != historicalEntries.size(); i++)
          // Serial.printf("Entry #%d:\n%s\n", i, historicalEntries[i].toString().c_str());

          readCount++;
          Serial.print("readCount=");
          Serial.println(readCount);
          Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5  connected=true = ");
          Serial.println(ESP.getFreeHeap());
          sendMQTTBlueTooth(myMacAddress, myRealTimeEntry, batterylevel, firmware, readCount);
          Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>3  connected=true = ");
          Serial.println(ESP.getFreeHeap());

          Serial.println("Waiting 20 seconds");
          Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>4 connected=true = ");
          Serial.println(ESP.getFreeHeap());

          vTaskDelay(20000 / portTICK_PERIOD_MS);

          connected = false;


        }
      }
      /* catch (...)
        {
         Serial.println(">>>>>>>>>Top level exception caught  - reinitializing");
         //flowerCare->scanBLE();
         //BLEDevice::deinit(false);
         flowerCare = new FlowerCare();
        }
      */

      vTaskDelay(6000 / portTICK_PERIOD_MS);

      Serial.print("Before  scanBLEFreeHeap=");
      Serial.println(ESP.getFreeHeap());
      //delete flowerCare;
      //flowerCare = new FlowerCare();
      flowerCare->scanBLE();

      Serial.print("After FreeHeap=");
      lastHeap = ESP.getFreeHeap();

      Serial.println(lastHeap);

      Serial.print("-------------->>>>>>Heap Difference:");
      Serial.println(lastHeap - firstHeap);
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);

  }
}


void taskSendSensors( void * parameter)
{
  // Enter RTOS Task Loop
  for (;;)  {

    if (uxSemaphoreGetCount( xSemaphoreReadSensor ) > 0)
    {
      xSemaphoreTake( xSemaphoreSensorsBeingRead, 10000);
      sendMQTT(MQTTSENSORS, "");
      xSemaphoreGive( xSemaphoreSensorsBeingRead);

    }
    //vTaskDelay(600000 / portTICK_PERIOD_MS);
    vTaskDelay(sensorCycle * 1000 / portTICK_PERIOD_MS);
  }

}


void taskReadSensors( void * parameter)
{
  // Enter RTOS Task Loop
  for (;;)  {

    if (uxSemaphoreGetCount( xSemaphoreReadSensor ) > 0)
    {
      xSemaphoreTake( xSemaphoreSensorsBeingRead, 10000);
      readSensors();
      xSemaphoreGive( xSemaphoreSensorsBeingRead);

    }
    //vTaskDelay(600000 / portTICK_PERIOD_MS);
    vTaskDelay(30 * 1000 / portTICK_PERIOD_MS);
  }

}


void taskSetValves( void * parameter)
{
  // Enter RTOS Task Loop
  for (;;)  {

    if (uxSemaphoreGetCount( xSemaphoreEvaluateValves ) > 0)
    {
      xSemaphoreTake( xSemaphoreEvaluatingValves, 1000);
      xSemaphoreTake( xSemaphoreSensorsBeingRead, 1000);
      evaluateValves(100);
      xSemaphoreGive( xSemaphoreEvaluatingValves);
      //vTaskDelay(50 / portTICK_PERIOD_MS);
      xSemaphoreGive( xSemaphoreSensorsBeingRead);
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
      for (i = 0; i < 4; i++)
      {
        if (moistureSensorEnable[i] == 1) // enabled
        {
          updateDisplay(DISPLAY_MOISTURE_1 + i);
          vTaskDelay(3000 / portTICK_PERIOD_MS);
        }





        //Serial.println("<-----------OutOfDisplay");

      }

    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
