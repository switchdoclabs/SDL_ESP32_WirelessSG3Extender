//
// Read moisture sensors
//

#include "OWMAdafruit_ADS1015.h"

Adafruit_ADS1015 ads1015;


// C1 - Grove Capacitive Moisture Sensor - SwitchDoc Labs

#define CAP0PERCENT 945
#define CAP100PERCENT 462

float scaleMoisture(int rawValue)
{
  float moisture_humidity = 0.0;

  //rawValue = rawValue / 4; // Scale to 10 bits

  if (rawValue < CAP100PERCENT)
  {
    moisture_humidity = 100.0;
  }
  else
  {
    moisture_humidity = ((CAP0PERCENT - rawValue) * 100.0) / (CAP0PERCENT - CAP100PERCENT);
  }

  if (moisture_humidity < 0.0)
    moisture_humidity = 0.0;

  if (moisture_humidity > 100.0)
    moisture_humidity = 100.0;

  return moisture_humidity;



}


float reportMoisture(int rawValue)
{
  float moisture_humidity = 0.0;

  return scaleMoisture(rawValue);


}

void printSensors();


//Hydroponics Sensors

void readHydroponicsSensors()
{


  Serial.println("--ReadHydroponicsSensors");
  // Temperature
  if (OneWire_Present)
  {
    DS18B20sensor.requestTemperatures();
    float currentTemp;

    currentTemp = DS18B20sensor.getTempCByIndex(0);
    if (currentTemp < -100.0)
    {

    } else
    {
      latestHydroponicsData.temperature = currentTemp;

      Serial.print("--------------------------------------------------------->temperature=");
      Serial.println(currentTemp);
    }

  }
  else
  {
    latestHydroponicsData.temperature = -1;
  }
  Serial.println("--Before Level - ReadHydroponicsSensors");
  // Level

  if (Level_Present)
  {

    xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);
    hydroponicsLevelSensor.start();

    latestHydroponicsData.rawLevel = hydroponicsLevelSensor.getDistance();

    Serial.print("--------------------------------------------------------->raw_distance=");
    Serial.println(latestHydroponicsData.rawLevel);
  }
  else
  {
    latestHydroponicsData.rawLevel = -1.0;
  }
  xSemaphoreGive( xSemaphoreUseI2C);

  // Turbidity
  Serial.println("--Before Turbidity - ReadHydroponicsSensors");

  if (moistureSensorEnable[1] == 1)
    writeGPIOBit(1, true);
  else
    writeGPIOBit(1, false);



  vTaskDelay( 1000 / portTICK_PERIOD_MS);
  // delay with it ont
  if (moistureSensorEnable[1] == 1)
  {
    xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);
    unsigned int rawValue;

    rawValue = ads1015.readADC_SingleEnded(1);

    latestHydroponicsData.rawTurbidity = rawValue;
    Serial.print("--------------------------------------------------------->raw_turbidity=");
    Serial.println(rawValue);

    xSemaphoreGive( xSemaphoreUseI2C);
  }
  else
    latestHydroponicsData.rawTurbidity = -1;

  writeGPIOBit(1, false);




  // TDS

  Serial.println("--Before TDS - ReadHydroponicsSensors");
  if (moistureSensorEnable[0] == 1)
    writeGPIOBit(0, true);
  else
    writeGPIOBit(0, false);



  if (moistureSensorEnable[0] == 1)
  {
    xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);
    unsigned int rawValue;

    rawValue = ads1015.readADC_SingleEnded(0);

    latestHydroponicsData.rawTDS = rawValue;
    Serial.print("--------------------------------------------------------->raw_TDS=");
    Serial.println(rawValue);

    xSemaphoreGive( xSemaphoreUseI2C);
  }
  else
    latestHydroponicsData.rawTDS = -1;

  writeGPIOBit(0, false);


  Serial.println("--End of ReadHydroponicsSensors");

}
void readHydroponicsLevelSensor()
{


}

// other sensors


void readSensors()
{


  int i;
  for (i = 0; i < 4; i++)
  {
    if (moistureSensorEnable[i] == 1)
      writeGPIOBit(i, true);
    else
      writeGPIOBit(i, false);
  }



  xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);
  unsigned int rawValue;

  for (i = 0; i < 4; i++)
  {
    if (moistureSensorEnable[i] == 1)
    {
      rawValue = ads1015.readADC_SingleEnded(i);
      moistureSensors[i] = scaleMoisture(rawValue);
      moistureSensorsRaw[i] = rawValue;



#ifdef EXTDEBUG
      // Serial.print("Channel:"); Serial.print(i); Serial.print(" Raw: "); Serial.println(rawValue);
#endif
    }

  }

  xSemaphoreGive( xSemaphoreUseI2C);
  for (i = 0; i < 4; i++)
  {
    writeGPIOBit(i, false);
  }


#ifdef EXTDEBUG
  printSensors();
#endif

}

void printSensors()
{
  bool printone;
  printone = false;
  int i;
  for (i = 0; i < 4; i++)
  {
    if (moistureSensorEnable[i] == 1)
    {
#ifdef EXTDEBUG
      //Serial.print("Channel: "); Serial.print(i + 1); Serial.print("   Scaled:"); Serial.println (moistureSensors[i]);
#endif
      if (printone == false) printone = true;
    }


  }

  if (printone)
    Serial.println(" ");

}




void ReadAMG8833Sensor()
{
  /*Read temp*/
  amg8833.read_pixel_temperature(AMG8833_temp  );
  /*Print 8X8 pixels value.*/
  Serial.println("Temperature for 8X8 matrix are::");
  for (int i = 0; i < PIXEL_NUM; i++) {
    Serial.print(AMG8833_temp[i]);
    Serial.print("  ");
    if (0 == (i + 1) % 8) {
      Serial.println(" ");
    }
  }

}
