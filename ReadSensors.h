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
    }

  }
  else
  {
    latestHydroponicsData.temperature = -1;
  }

  // Level

  if (Level_Present)
  {

    hydroponicsLevelSensor.start();
    latestHydroponicsData.rawLevel = hydroponicsLevelSensor.getDistance();
  }
  else
  {
    latestHydroponicsData.rawLevel = -1.0;
  }


  // Turbidity


  if (moistureSensorEnable[1] == 1)
    writeGPIOBit(1, true);
  else
    writeGPIOBit(1, false);
    
  if (moistureSensorEnable[1] == 1)
  {
    xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);
    unsigned int rawValue;

    rawValue = ads1015.readADC_SingleEnded(1);

    latestHydroponicsData.rawTurbidity = rawValue;

    xSemaphoreGive( xSemaphoreUseI2C);
  }
  else
    latestHydroponicsData.rawTurbidity = -1;

  writeGPIOBit(1, false);




  // TDS


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

    xSemaphoreGive( xSemaphoreUseI2C);
  }
  else
    latestHydroponicsData.rawTDS = -1;

  writeGPIOBit(0, false);


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
