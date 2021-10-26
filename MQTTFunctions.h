
char myAddString[250];  // special non-stack buffer for MQTTBlueTooth

int sendMQTTBlueTooth(String MacAddress, int temperature, int moisture, int brightness, int conductivity,  int battery,  int readCount)
{





  if (!MQTTclient.connected()) {
    MQTTreconnect(true);
    Serial.println("reconnected");
  }
  else
  {
    Serial.println("connected");
  }
  MQTTclient.loop();


  char buffer[64];



  /* Temperature:  24.30°C   >> Published
    06:54:29.060 -> -- Moisture:     0 %   >> Published
    06:54:29.060 -> -- Light:        2571 lux   >> Published
    06:54:29.060 -> -- Conductivity: 0 uS/cm   >> Published
    Battery
    Firmware
    client address
  */
  strcpy(myAddString , "{");

  strcat(myAddString, "\"id\": \"");

  strcat(myAddString, myID.c_str());
  strcat(myAddString, "\", \"messagetype\": \"");
  snprintf(buffer, sizeof(buffer), "%d", MQTTBLUETOOTHSENSOR);
  strcat(myAddString, buffer);
  strcat(myAddString, "\", \"timestamp\": \"");

  snprintf(buffer, sizeof(buffer), "%02d-%02d-%02d %02d:%02d:%02d", year(), month(), day(), hour(), minute(), second());

  //buffer now contains a string like 02/22/2013 05:03:0


  strcat(myAddString, buffer);

  strcat(myAddString, "\", \"macaddress\": \"");
  strcat(myAddString, MacAddress.c_str());

  strcat(myAddString, "\", \"temperature\": \"");

  snprintf(buffer, sizeof(buffer), "%d", temperature);
  strcat(myAddString, buffer);

  strcat(myAddString, "\", \"brightness\": \"");

  snprintf(buffer, sizeof(buffer), "%d", brightness);
  strcat(myAddString, buffer);

  strcat(myAddString, "\", \"moisture\": \"");


  snprintf(buffer, sizeof(buffer), "%d", moisture);
  strcat(myAddString, buffer);

  strcat(myAddString, "\", \"conductivity\": \"");

  snprintf(buffer, sizeof(buffer), "%d", conductivity);
  strcat(myAddString, buffer);


  strcat(myAddString, "\", \"battery\": \"");

  snprintf(buffer, sizeof(buffer), "%d", battery);
  strcat(myAddString, buffer);

  strcat(myAddString , "\", \"readCount\": \"");

  snprintf(buffer, sizeof(buffer), "%d", readCount);
  strcat(myAddString, buffer);

  strcat(myAddString, "\", \"sensorType\": \"");

  strcat(myAddString, "BT1");

  strcat(myAddString, "\"");





  strcat(myAddString, "}"); //Send the request
#ifdef HEAPDEBUG

  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.4  connected=true = ");
  Serial.println(ESP.getFreeHeap());
#endif
  // publish it

  Serial.println( "Sending Bluetooth MQTT Packet");
  Serial.println( myAddString);
  int result;
  // Once connected, publish an announcement...
  strcpy(buffer, "SGS/");
  strcat(buffer, myID.c_str());
  Serial.print("Topic=");
  Serial.println(buffer);
#ifdef HEAPDEBUG
  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.4.1  connected=true = ");
  Serial.println(ESP.getFreeHeap());
#endif
  result = MQTTclient.publish(buffer,  myAddString);
  Serial.print("MQTT publish result=");
  Serial.println(result);
#ifdef HEAPDEBUG
  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.6  connected=true = ");
  Serial.println(ESP.getFreeHeap());
#endif
  return result;
}






int sendMQTT(int messageType, String argument)
{

  String AddString;

  String SendString;

  if (!MQTTclient.connected()) {
    MQTTreconnect(true);
  }
  MQTTclient.loop();


  char buffer[32];

  snprintf(buffer, sizeof(buffer), "%02d/%02d/%4d %02d:%02d:%02d", month(), day(), year(), hour(), minute(), second());

  //buffer now contains a string like 02/22/2013 05:03:06

  String myTime;
  myTime = String(buffer);


  switch (messageType)
  {

    case MQTTVALVECHANGE:

      {
        AddString = "\"id\": \"";
        AddString += myID;
        AddString += "\", \"messagetype\": \"";
        AddString += messageType;
        AddString += "\", \"timestamp\": \"";
        AddString += myTime;
        AddString += "\", \"valvestate\": \"";


        AddString += "V";
        int i;
        for (i = 0; i < 8; i++)
        {
          AddString += String(valveState[i]);
        }

        AddString += "\"";

        SendString = "{" + AddString +  "}"; //Send the request


        break;
      }
    case MQTTTESTMESSAGE:
      {


        AddString = "\"id\": \"";
        AddString += myID;
        AddString += "\", \"messagetype\": \"";
        AddString += messageType;
        AddString += "\", \"timestamp\": \"";
        AddString += myTime;
        SendString = "{" + AddString +  "\"}"; //Send the request

        break;
      }

    case MQTTDEBUG:
      {
        AddString = "\"id\": \"";
        AddString += myID;
        AddString += "\", \"messagetype\": \"";
        AddString += messageType;
        AddString += "\", \"timestamp\": \"";
        AddString += myTime;
        AddString += "\", \"value\": \"";
        AddString += argument;

        SendString = "{" + AddString +  "\"}"; //Send the request
        break;


      }

    case MQTTREBOOT:
      {

        AddString = "\"id\": \"";
        AddString += myID;
        AddString += "\", \"messagetype\": \"";
        AddString += messageType;
        AddString += "\", \"timestamp\": \"";
        AddString += myTime;
        AddString += "\", \"value\": \"";
        AddString += argument;

        SendString = "{" + AddString +  "\"}"; //Send the request
        break;


      }


    case MQTTHYDROPONICS:
      {

        AddString = "\"id\": \"";
        AddString += myID;
        AddString += "\", \"messagetype\": \"";
        AddString += messageType;
        AddString += "\", \"timestamp\": \"";
        AddString += myTime;

        AddString += "\", \"temperature\": \"";
        AddString += String(latestHydroponicsData.temperature);

        AddString += "\", \"rawlevel\": \"";
        AddString += String(latestHydroponicsData.rawLevel) ;

        AddString += "\", \"rawturbidity\": \"";
        AddString += String(latestHydroponicsData.rawTurbidity) ;

        AddString += "\", \"rawtds\": \"";
        AddString += String(latestHydroponicsData.rawTDS) ;

        AddString += "\", \"rawph\": \"";
        AddString += String(latestHydroponicsData.rawPh) ;

        SendString = "{" + AddString +  "\"}"; //Send the request
        break;


      }

    case MQTTHYDROPONICSLEVEL:
      {

        AddString = "\"id\": \"";
        AddString += myID;
        AddString += "\", \"messagetype\": \"";
        AddString += messageType;
        AddString += "\", \"timestamp\": \"";
        AddString += myTime;

        AddString += "\", \"rawlevel\": \"";
        AddString += String(latestHydroponicsData.rawLevel) + ",";

        SendString = "{" + AddString +  "\"}"; //Send the request
        break;


      }

    case MQTTINFRARED:
      {

        AddString = "\"id\": \"";
        AddString += myID;
        AddString += "\", \"messagetype\": \"";
        AddString += messageType;
        AddString += "\", \"timestamp\": \"";
        AddString += myTime;

        AddString += "\", \"infrareddata\": \"";
        for(int i; i < PIXEL_NUM; i++)
        {
        AddString += String(AMG8833_temp[i]) + ",";
        }
        AddString += "\"";
        SendString = "{" + AddString + "}"; //Send the request
        break;


      }

    case MQTTSENSORS:
      {

        AddString = "\"id\": \"";
        AddString += myID;
        AddString += "\", \"messagetype\": \"";
        AddString += messageType;
        AddString += "\", \"timestamp\": \"";
        AddString += myTime;
        AddString += "\", \"enableSensors\": \"";

        int i;
        for (i = 0; i < 4; i++)
        {

          AddString += String(moistureSensorEnable[i]) + ",";

        }
        AddString += "\", \"sensorValues\": \"";
        for (i = 0; i < 4; i++)
        {
          if (moistureSensorEnable[i] == 0)
            AddString += String(-1) + ",";
          else
            AddString += String(moistureSensors[i]) + ",";

        }

        AddString += "\", \"rawSensorValues\": \"";
        for (i = 0; i < 4; i++)
        {
          if (moistureSensorEnable[i] == 0)
            AddString += String(-1) + ",";
          else
            AddString += String(moistureSensorsRaw[i]) + ",";

        }
        AddString += "\", \"sensorType\": \"";
        String mySensorType = "";
        for (i = 0; i < 4; i++)
        {
          mySensorType += moistureSensorType[i];
          if (i < 3)
            mySensorType += ",";
        }
        AddString += mySensorType + "\"";


        SendString = "{" + AddString +  "}"; //Send the request


        break;
      }



    default:
      break;


  }


  // publish it

  Serial.println( "Sending MQTT Packet");
  Serial.println(SendString);
  int result;
  // Once connected, publish an announcement...
  String Topic = "SGS/" + myID;
  Serial.print("Topic=");
  Serial.println(Topic);
  result = MQTTclient.publish(Topic.c_str(), SendString.c_str());
  Serial.print("MQTT publish result=");
  Serial.println(result);
  return result;
}
