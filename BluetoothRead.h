 // scan bluetooth sensors if required

void readBluetooth()
{

    if (uxSemaphoreGetCount( xSemaphoreReadBluetooth ) > 0)
    {


      firstHeap = ESP.getFreeHeap();
    #ifdef HEAPDEBUG
      Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>> 0  main loop ");
      Serial.println(ESP.getFreeHeap());
    #endif


      bluetoothDeviceCount = countBluetoothSensors();
      // process devices
      for (int i = 0; i < bluetoothDeviceCount; i++) {
        Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>> Processing Bluetooth: ");
        Serial.println(i);
        int tryCount = 0;
        char* deviceMacAddress = (char *) BluetoothAddresses[i].c_str();
        BLEAddress floraAddress(deviceMacAddress);

        while (tryCount < BLUETOOTHRETRY) {
          tryCount++;
          Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>> Bluetooth Try: ");
          Serial.println(tryCount);
          if (processFloraDevice(floraAddress, deviceMacAddress, readBattery, tryCount)) {
            readCount++;

            Serial.print("ReadCount==");
            Serial.println(readCount);
            if (latestFloraData.validData)
            {
              // Good data, send it.

              sendMQTTBlueTooth(BluetoothAddresses[i], latestFloraData.temperature, latestFloraData.moisture, latestFloraData.light, latestFloraData.conductivity, latestFloraData.battery,  readCount);
              break;
            }
    #ifdef HEAPDEBUG
            Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>> 0.3  main loop ");
            Serial.println(ESP.getFreeHeap());
    #endif

          }
          Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>> RETRYING Bluetooth: ");
          vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        vTaskDelay(1500 / portTICK_PERIOD_MS);
      }
    #ifdef HEAPDEBUG
      Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>> 0.4  main loop ");
      Serial.println(ESP.getFreeHeap());
    #endif
      vTaskDelay(6000 / portTICK_PERIOD_MS);
    #ifdef HEAPDEBUG
      Serial.print("Before  scanBLEFreeHeap=");
      Serial.println(ESP.getFreeHeap());


    #endif

      Serial.print("After FreeHeap=");

      lastHeap = ESP.getFreeHeap();

      Serial.println(lastHeap);

      Serial.print("-------------->>>>>>Heap Difference:");
      Serial.println(lastHeap - firstHeap);



    }

    //printSemaphoreStatus(" After BluetoothCheck");


}
