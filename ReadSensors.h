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

// Check and re-read if bad
int validateRawValue(int channel, int rawValue, int currentValue)
{
  //Serial.print("rawValue = ");
  //Serial.println(rawValue);
  //Serial.print("currentValue=");
  //Serial.println(currentValue);
  if (rawValue > 2040)
  {
    rawValue = ads1015.readADC_SingleEnded(channel);
    if (rawValue > 2040)
    {
      return currentValue;
    }
  }
  return rawValue;


}


void readHydroponicsSensors();

void readAndSendHydroponicSensors()
{

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
}

// test ADC sensors

String readADCSensors()
{
  String returnSensors;
  returnSensors = "";

  Serial.println("--ReadHydroponicsSensors");
  // Temperature
  if (OneWire_Present)
  {
    DS18B20sensor.requestTemperatures();
    float currentTemp;

    currentTemp = DS18B20sensor.getTempCByIndex(0);
    if (currentTemp < -100.0)
    {
      returnSensors += "-1,";
    } else
    {
      returnSensors += String(currentTemp) + ",";

      //Serial.print("--------------------------------------------------------->temperature=");
      //Serial.println(currentTemp);
    }

  }
  else
  {
    returnSensors += "-1,";
  }



  // TDS

  //Serial.println("--Before TDS - ReadHydroponicsSensors");

  writeGPIOBit(0, true);


  vTaskDelay( 500 / portTICK_PERIOD_MS);


  xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);
  unsigned int rawValue;

  rawValue = ads1015.readADC_SingleEnded(0);
  returnSensors += String(rawValue) + ",";
  //Serial.print("rawValue = ");
  //Serial.println(rawValue);
  //Serial.print(" latestHydroponicsData.rawTDS=");
  //Serial.println( latestHydroponicsData.rawTDS);
  //Serial.print("--------------------------------------------------------->raw_TDS=");
  //Serial.println(rawValue);

  xSemaphoreGive( xSemaphoreUseI2C);


  writeGPIOBit(0, false);


  // Turbidity
  //Serial.println("--Before Turbidity - ReadHydroponicsSensors");


  writeGPIOBit(1, true);




  vTaskDelay( 500 / portTICK_PERIOD_MS);
  // delay with it on

  xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);


  rawValue = ads1015.readADC_SingleEnded(1);
  returnSensors += String(rawValue) + ",";
  //Serial.print("--------------------------------------------------------->raw_turbidity=");
  //Serial.println(rawValue);

  xSemaphoreGive( xSemaphoreUseI2C);


  writeGPIOBit(1, false);



  // Ph sensor

  //Serial.println("--Before Ph - ReadHydroponicsSensors");

  writeGPIOBit(2, true);


  vTaskDelay( 500 / portTICK_PERIOD_MS);

  xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);


  rawValue = ads1015.readADC_SingleEnded(2);
  returnSensors += String(rawValue) + ",";

  //Serial.print("--------------------------------------------------------->raw_Ph=");
  //Serial.println(rawValue);

  xSemaphoreGive( xSemaphoreUseI2C);


  writeGPIOBit(2, false);


  // Port 4 capacitive level sensor
  if (Level_Present)
  {
    //Serial.println("--Before Port 4 cap level Moisture Sensor - ReadHydroponicsSensors");

    writeGPIOBit(3, true);


    vTaskDelay( 500 / portTICK_PERIOD_MS);
    xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);
    unsigned int rawValue;

    rawValue = ads1015.readADC_SingleEnded(3);
    returnSensors += String(rawValue) + ",";
    //Serial.print("--------------------------------------------------------->raw_Cap level4=");
    //Serial.println(rawValue);

    xSemaphoreGive( xSemaphoreUseI2C);

  }
  else

    returnSensors += "-1,";



  writeGPIOBit(3, false);

  //Serial.println("--End of ReadHydroponicsSensors");
  return returnSensors;

}
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
      latestHydroponicsData.temperature = -1;
    } else
    {
      latestHydroponicsData.temperature = currentTemp;

      //Serial.print("--------------------------------------------------------->temperature=");
      //Serial.println(currentTemp);
    }

  }
  else
  {
    latestHydroponicsData.temperature = -1;
  }



  // TDS

  //Serial.println("--Before TDS - ReadHydroponicsSensors");

  writeGPIOBit(0, true);


  vTaskDelay( 500 / portTICK_PERIOD_MS);


  xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);
  unsigned int rawValue;

  rawValue = ads1015.readADC_SingleEnded(0);
  rawValue = validateRawValue(0, rawValue, latestHydroponicsData.rawTDS);
  //Serial.print("rawValue = ");
  //Serial.println(rawValue);
  //Serial.print(" latestHydroponicsData.rawTDS=");
  //Serial.println( latestHydroponicsData.rawTDS);

  latestHydroponicsData.rawTDS = rawValue;
  //Serial.print("--------------------------------------------------------->raw_TDS=");
  //Serial.println(rawValue);

  xSemaphoreGive( xSemaphoreUseI2C);


  writeGPIOBit(0, false);


  // Turbidity
  //Serial.println("--Before Turbidity - ReadHydroponicsSensors");


  writeGPIOBit(1, true);




  vTaskDelay( 500 / portTICK_PERIOD_MS);
  // delay with it on

  xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);


  rawValue = ads1015.readADC_SingleEnded(1);
  rawValue = validateRawValue(1, rawValue, latestHydroponicsData.rawTurbidity);
  latestHydroponicsData.rawTurbidity = rawValue;
  //Serial.print("--------------------------------------------------------->raw_turbidity=");
  //Serial.println(rawValue);

  xSemaphoreGive( xSemaphoreUseI2C);


  writeGPIOBit(1, false);



  // Ph sensor

  //Serial.println("--Before Ph - ReadHydroponicsSensors");

  writeGPIOBit(2, true);


  vTaskDelay( 500 / portTICK_PERIOD_MS);

  xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);


  rawValue = ads1015.readADC_SingleEnded(2);
  rawValue = validateRawValue(2, rawValue, latestHydroponicsData.rawPh);

  latestHydroponicsData.rawPh = rawValue;
  //Serial.print("--------------------------------------------------------->raw_Ph=");
  //Serial.println(rawValue);

  xSemaphoreGive( xSemaphoreUseI2C);


  writeGPIOBit(2, false);


  // Port 4 capacitive level sensor
  if (Level_Present)
  {
    //Serial.println("--Before Port 4 cap level Moisture Sensor - ReadHydroponicsSensors");

    writeGPIOBit(3, true);


    vTaskDelay( 500 / portTICK_PERIOD_MS);
    xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);
    unsigned int rawValue;

    rawValue = ads1015.readADC_SingleEnded(3);
    rawValue = validateRawValue(3, rawValue, latestHydroponicsData.rawLevel);
    //Serial.print("--------------------------------------------------------->raw_Cap level4=");
    //Serial.println(rawValue);
    latestHydroponicsData.rawLevel = rawValue;

    xSemaphoreGive( xSemaphoreUseI2C);

  }
  else

    latestHydroponicsData.rawLevel = -1.0;



  writeGPIOBit(3, false);

  //Serial.println("--End of ReadHydroponicsSensors");

}

void readHydroponicsLevelSensor()
{
  Serial.println("--Fast Read Level HydroponicsSensors");

  writeGPIOBit(3, true);


  vTaskDelay( 1000 / portTICK_PERIOD_MS);
  xSemaphoreTake( xSemaphoreUseI2C,  portMAX_DELAY);
  unsigned int rawValue;

  rawValue = ads1015.readADC_SingleEnded(3);
  rawValue = validateRawValue(3, rawValue, latestHydroponicsData.rawLevel);
  Serial.print("--------------------------------------------------------->Fast Read raw_Cap level4=");
  Serial.println(rawValue);
  latestHydroponicsData.rawLevel = rawValue;

  xSemaphoreGive( xSemaphoreUseI2C);


}



void ReadAMG8833Sensor()
{
  /*Read temp*/
  amg8833.read_pixel_temperature(AMG8833_temp  );

  /*Print 8X8 pixels value.*/
  /*
    Serial.println("Temperature for 8X8 matrix are::");
    for (int i = 0; i < PIXEL_NUM; i++) {
    Serial.print(AMG8833_temp[i]);
    Serial.print("  ");
    if (0 == (i + 1) % 8) {
      Serial.println(" ");
    }
    }
  */
}

void ReadAndSendAMG8833Sensor()
{

  if (uxSemaphoreGetCount( xSemaphoreReadInfrared ) > 0)
  {
    xSemaphoreTake( xSemaphoreUseI2C, 10000);

    ReadAMG8833Sensor();
    xSemaphoreGive( xSemaphoreUseI2C);

    sendMQTT(MQTTINFRARED, "");


  }
}
