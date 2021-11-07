/*********************************************************************
  This is an example for our Monochrome OLEDs based on SSD1306 drivers


  This example is for a 128x64 size display using I2C to communicate

  Written by Limor Fried/Ladyada  for Adafruit Industries.
  BSD license, check license.txt for more information

  Heavily modified by SDL
*********************************************************************/

// prototypes

void setDisplayLineLCD(int lineNumber, char *value);
void writeAllDisplayLines(int DisplayMode);


// https://stackoverflow.com/questions/9072320/split-string-into-string-array

char displayLines[25][28];


// WeatherPlus Text Buffer Lines

void updateDisplay(int displayMode)
{

  if (LCD_Present) {

    char buffer[40];
    char returnString[200];
    /*
      #ifdef EXTDEBUG
      Serial.print("displayMode=");
      Serial.println(displayMode);
      #endif
    */
    String windDirection;

    //Serial.print("StartofupdateDisplay xi2c=");
    //Serial.println(uxSemaphoreGetCount( xSemaphoreUseI2C ));


    String currentTimeString;
    currentTimeString = returnDateTime(now());

    switch (displayMode)
    {





      case DISPLAY_POWERUP:
        Serial.println("SGS3WExt Booting Up");

        strcpy(buffer, "Ver: ");
        strcat(buffer, SGSEXTENDERESP32VERSION);

        setDisplayLineLCD(0,  "SG3WExt Boot");
        setDisplayLineLCD(1, buffer);


        break;

      case  DISPLAY_ACCESSPOINT:
        {





          setDisplayLineLCD(0,  "SGSExt2 AP");
          buffer[0] = '\0';
          strcpy(buffer, APssid.c_str());
          setDisplayLineLCD(1, buffer);
          buffer[0] = '\0';

          IPAddress myIp2 = WiFi.softAPIP();
          sprintf(buffer, "%d.%d.%d.%d", myIp2[0], myIp2[1], myIp2[2], myIp2[3]);
          setDisplayLineLCD(2, buffer);
        }
        break;


      case  DISPLAY_TRYING_AP:
        {





          setDisplayLineLCD(0,  "Trying WiFi AP");
          buffer[0] = '\0';
          strcpy(buffer, "192.168.4.1");
          setDisplayLineLCD(1, buffer);
          buffer[0] = '\0';


          String mySSID;
          mySSID = "SG3WExt-" + macID;
          buffer[0] = '\0';
          strcpy(buffer, mySSID.c_str());
          setDisplayLineLCD(2, buffer);
          setDisplayLineLCD(3, "");

        }
        break;

      case  DISPLAY_TRYING_SMARTCONFIG:
        {
          Serial.println("SG3WExt Setup");




          setDisplayLineLCD(0,  "Trying WiFi ");
          buffer[0] = '\0';
          strcpy(buffer, "Smart Config");
          setDisplayLineLCD(1, buffer);
          buffer[0] = '\0';
          setDisplayLineLCD(2, "");
          setDisplayLineLCD(3, "");

        }
        break;

      case  DISPLAY_TRYING_WPS:
        {
          Serial.println("SG3WExt Setup");




          setDisplayLineLCD(0,  "Trying WiFi WPS");
          buffer[0] = '\0';

          setDisplayLineLCD(1, "");
          setDisplayLineLCD(2, "");
          setDisplayLineLCD(3, "");

        }
        break;



      case  DISPLAY_FAILING_AP:
        {

          setDisplayLineLCD(0,  "Failing to connect WiFi AP");
          buffer[0] = '\0';
          strcpy(buffer, WiFi_SSID.c_str());
          setDisplayLineLCD(1, buffer);
          buffer[0] = '\0';
          setDisplayLineLCD(2, "Restarting SG3WExt");
          setDisplayLineLCD(3, "Try again....");

        }
        break;

      case DISPLAY_FAILED_RECONNECT:
        {

          setDisplayLineLCD(0,  "Failing to reconnect to WiFI");
          buffer[0] = '\0';
          strcpy(buffer, WiFi_SSID.c_str());
          setDisplayLineLCD(1, buffer);
          buffer[0] = '\0';
          setDisplayLineLCD(3, "Will try again....");

        }
        break;

      case DISPLAY_IPDISPLAY:

        {
          setDisplayLineLCD(0, "SG3WExt Connected");
          IPAddress myIp2 = WiFi.localIP();
          sprintf(buffer, "%d.%d.%d.%d", myIp2[0], myIp2[1], myIp2[2], myIp2[3]);
          setDisplayLineLCD(1, buffer);
        }
        break;

      case DISPLAY_IPNAMEID:
        {
          sprintf(buffer, "Device: %s", myID);
          setDisplayLineLCD(0, buffer);

          IPAddress myIp2 = WiFi.localIP();
          sprintf(buffer, "%d.%d.%d.%d", myIp2[0], myIp2[1], myIp2[2], myIp2[3]);
          setDisplayLineLCD(1, buffer);
          //       sprintf(buffer, "%s / %s", stationName.c_str(), myID.c_str());
          //        setDisplayLineLCD(2, buffer);
        }
        break;

      case DISPLAY_STATUS:


        sprintf(buffer, "SVS: %1i%1i%1i%1i%1i%1i%1i%1i", valveState[0], valveState[1], valveState[2], valveState[3], valveState[4], valveState[5], valveState[6], valveState[7]);
        setDisplayLineLCD(0, buffer);
        buffer[0] = '\0';
        sprintf(buffer, "MSE: %1i%1i%1i%1i", moistureSensorEnable[0], moistureSensorEnable[1], moistureSensorEnable[2], moistureSensorEnable[3]);
        setDisplayLineLCD(1, buffer);
        break;

      case DISPLAY_MOISTURE_1:
      case DISPLAY_MOISTURE_2:
      case DISPLAY_MOISTURE_3:
      case DISPLAY_MOISTURE_4:

        int unit;
        unit = displayMode - DISPLAY_MOISTURE_1;

        // Displays Moisture Levels
        buffer[0] = '\0';
        sprintf(buffer, "Sensor %s #%i", moistureSensorType[unit], unit + 1);
        setDisplayLineLCD(0, buffer);
        buffer[0] = '\0';
        if (moistureSensorType[unit] == "C1")
        {
          sprintf(buffer, "%3.1f%% Raw:%d", moistureSensors[unit], moistureSensorsRaw[unit]);
        }
        else
        {
          if (moistureSensorType[unit] == "TUR1")
            sprintf(buffer, "Raw:%d", latestHydroponicsData.rawTurbidity);
          else if (moistureSensorType[unit] == "TDS1")
            sprintf(buffer, "Raw:%d", latestHydroponicsData.rawTDS);
          else
            sprintf(buffer, "Raw:%d", moistureSensorsRaw[unit]);

        }
        setDisplayLineLCD(1, buffer);

        break;


      case DISPLAY_BLUETOOTH:
      case DISPLAY_BLUETOOTH+1:
      case DISPLAY_BLUETOOTH+2:
      case DISPLAY_BLUETOOTH+3:
      case DISPLAY_BLUETOOTH+4:
      case DISPLAY_BLUETOOTH+5:
      case DISPLAY_BLUETOOTH+6:
      case DISPLAY_BLUETOOTH+7:
        {
          unit = displayMode - DISPLAY_BLUETOOTH;

          // Displays Bluetooth Levels

          String pickAddress;
          pickAddress =  BluetoothAddresses[unit].substring(BluetoothAddresses[unit].length() - 5, BluetoothAddresses[unit].length());
          buffer[0] = '\0';
          sprintf(buffer, "BTS %s  T/H", pickAddress);
          setDisplayLineLCD(0, buffer);
          buffer[0] = '\0';
          if ((LastMoistureBluetoothRead[unit] < 0) || (LastTemperatureBluetoothRead[unit] < -500))
          {
            sprintf(buffer, "NA/NA");
          }
          else
          {

            if (EnglishOrMetric == 0)
            {
              sprintf(buffer, "%4.1fF/%d", LastTemperatureBluetoothRead[unit], LastMoistureBluetoothRead[unit]);
            }
            else
            {
              sprintf(buffer, "%4.1fC/%d", LastTemperatureBluetoothRead[unit], LastMoistureBluetoothRead[unit]);
            }
          }
          setDisplayLineLCD(1, buffer);
        }
        break;


      case DISPLAY_DEVICEPRESENT:

        {
          setDisplayLineLCD(0, "---Devices Present---");

          String myBuffer;
          myBuffer = "SAP: ";
          if (Solar_Present == true)
          {
            myBuffer = myBuffer + "X ";
          }
          else
          {
            myBuffer = myBuffer + "- ";
          }

          strcpy(buffer, myBuffer.c_str());
          setDisplayLineLCD(1, buffer );

          myBuffer = "AQ: ";
          if (AirQualityPresent == true)
          {
            myBuffer = myBuffer + "+ ";
          }
          else
          {
            myBuffer = myBuffer + "- ";
          }

          strcpy(buffer, myBuffer.c_str());
          setDisplayLineLCD(2, buffer );

          myBuffer = "SUNLT: ";
          if (TSL2591_Present == true)
          {
            myBuffer = myBuffer + "+ ";
          }
          else
          {
            myBuffer = myBuffer + "- ";
          }




          strcpy(buffer, myBuffer.c_str());
          setDisplayLineLCD(3, buffer );

          myBuffer = "";


          strcpy(buffer, myBuffer.c_str());
          setDisplayLineLCD(4, buffer );

          myBuffer = "SMLA: ";
          if (SolarMAXLA == 1)
          {
            myBuffer = myBuffer + "+ ";
          }
          else
          {
            myBuffer = myBuffer + "- ";
          }

          myBuffer = myBuffer + "SMLP: ";
          if (SolarMAXLiPo == true)
          {
            myBuffer = myBuffer + "+";
          }
          else
          {
            myBuffer = myBuffer + "-";
          }


          strcpy(buffer, myBuffer.c_str());
          setDisplayLineLCD(5, buffer );


        }
        break;



      case DISPLAY_SUNAIRPLUS:

        {
          setDisplayLineLCD(0, "Solar Readings");
          setDisplayLineLCD(1, "----------------");

          String stringSolar;
          stringSolar = "Battery:" + String(BatteryVoltage, 2) + "V/" + String(BatteryCurrent, 1) + "mA";
          setDisplayLineLCD(2, const_cast<char*>(stringSolar.c_str()) );

          stringSolar = "Solar:" + String(SolarPanelVoltage, 2) + "V/" + String(SolarPanelCurrent, 1) + "mA";
          setDisplayLineLCD(3, const_cast<char*>(stringSolar.c_str()) );

          stringSolar = "Load:" + String(LoadVoltage, 2) + "V/" + String(LoadCurrent, 1) + "mA";
          setDisplayLineLCD(4, const_cast<char*>(stringSolar.c_str()) );




        }
        break;



      default:
        break;
    }



    writeAllDisplayLines(displayMode);

  }

}



void setDisplayLineLCD(int lineNumber, char *value)
{
  if (lineNumber < 2)
  {

    strncpy( displayLines[lineNumber], value, 16);

  }




}


void writeAllDisplayLines(int DisplayMode)
{

  xSemaphoreTake( xSemaphoreUseI2C, 30000);
  lcd.clear();


  switch (DisplayMode)
  {
    case DISPLAY_POWERUP:
    case DISPLAY_UPDATE_FINISHED:
    case DISPLAY_UPDATING:
    case DISPLAY_NO_UPDATE_AVAILABLE:
    case DISPLAY_NO_UPDATE_FAILED:
    case DISPLAY_STATUS:

      {
        int textSize = 1;
        int i;
        for (i = 0; i < 2; i++)
        {

          lcd.setCursor(0, i);
          vTaskDelay(100 / portTICK_PERIOD_MS);

          lcd.print(String(displayLines[i]));

        }
      }

      break;

    case DISPLAY_MOISTURE_1:
    case DISPLAY_MOISTURE_2:
    case DISPLAY_MOISTURE_3:
    case DISPLAY_MOISTURE_4:
    case DISPLAY_BLUETOOTH:
    case DISPLAY_BLUETOOTH+1:
    case DISPLAY_BLUETOOTH+2:
    case DISPLAY_BLUETOOTH+3:
    case DISPLAY_BLUETOOTH+4:
    case DISPLAY_BLUETOOTH+5:
    case DISPLAY_BLUETOOTH+6:
    case DISPLAY_BLUETOOTH+7:
      {
        int textSize = 2;

        int i;
        for (i = 0; i < 2; i++)
        {

          lcd.setCursor(0, i);
          vTaskDelay(100 / portTICK_PERIOD_MS);

          lcd.print(displayLines[i]);

        }
      }
      break;




    case DISPLAY_IPDISPLAY:
    case DISPLAY_IPNAMEID:

    case DISPLAY_ACCESSPOINT:
    case DISPLAY_TRYING_AP:
    case DISPLAY_FAILING_AP:
    case DISPLAY_SDL2MQTTServer:
    case DISPLAY_TRYING_SMARTCONFIG:
    case DISPLAY_TRYING_WPS:

    case DISPLAY_FAILED_RECONNECT:


      {

        int i;
        for (i = 0; i < 2; i++)
        {
          lcd.setCursor(0, i);
          vTaskDelay(100 / portTICK_PERIOD_MS);

          lcd.print(displayLines[i]);

        }
      }

      break;


    default:
      break;

  }
  xSemaphoreGive( xSemaphoreUseI2C);
}
