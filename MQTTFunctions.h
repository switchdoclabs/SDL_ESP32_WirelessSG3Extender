
char myAddString[250];  // special non-stack buffer for MQTTBlueTooth

int sendMQTTBlueTooth(std::string MacAddress, RealTimeEntry *myRealTimeEntry, int battery, std::string firmware, int readCount)
{



  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.1  connected=true = ");
  Serial.println(ESP.getFreeHeap());


  if (!MQTTclient.connected()) {
    MQTTreconnect(true);
  }
  MQTTclient.loop();


  char buffer[64];
  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.2  connected=true = ");
  Serial.println(ESP.getFreeHeap());

  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.3  connected=true = ");
  Serial.println(ESP.getFreeHeap());


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

  snprintf(buffer, sizeof(buffer), "%02d/%02d/%4d %02d:%02d:%02d", month(), day(), year(), hour(), minute(), second());

  //buffer now contains a string like 02/22/2013 05:03:0

  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.3.1  connected=true = ");
  Serial.println(ESP.getFreeHeap());
  strcat(myAddString, buffer);

  strcat(myAddString, "\", \"macaddress\": \"");
  strcat(myAddString, MacAddress.c_str());

  strcat(myAddString, "\", \"temperature\": \"");

  snprintf(buffer, sizeof(buffer), "%d", myRealTimeEntry->temperature);
  strcat(myAddString, buffer);

  strcat(myAddString, "\", \"brightness\": \"");

  snprintf(buffer, sizeof(buffer), "%d", myRealTimeEntry->brightness);
  strcat(myAddString, buffer);

  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.3.2  connected=true = ");
  Serial.println(ESP.getFreeHeap());
  strcat(myAddString, "\", \"moisture\": \"");


  snprintf(buffer, sizeof(buffer), "%d", myRealTimeEntry->moisture);
  strcat(myAddString, buffer);

  strcat(myAddString, "\", \"conductivity\": \"");

  snprintf(buffer, sizeof(buffer), "%d", myRealTimeEntry->conductivity);
  strcat(myAddString, buffer);


  strcat(myAddString, "\", \"battery\": \"");

  snprintf(buffer, sizeof(buffer), "%d", battery);
  strcat(myAddString, buffer);

  strcat(myAddString, "\", \"firmware\": \"");
  strcat(myAddString , firmware.c_str());

  strcat(myAddString , "\", \"readCount\": \"");

  snprintf(buffer, sizeof(buffer), "%d", readCount);
  strcat(myAddString, buffer);
  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.3.3  connected=true = ");
  Serial.println(ESP.getFreeHeap());
  strcat(myAddString, "\", \"sensorType\": \"");

  strcat(myAddString, "BT1");

  strcat(myAddString, "\"");


  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.3.4  connected=true = ");
  Serial.println(ESP.getFreeHeap());


  strcat(myAddString, "}"); //Send the request
  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.4  connected=true = ");
  Serial.println(ESP.getFreeHeap());

  // publish it

  Serial.println( "Sending Bluetooth MQTT Packet");
  Serial.println( myAddString);
  int result;
  // Once connected, publish an announcement...
  strcpy(buffer, "SGS/");
  strcat(buffer, myID.c_str());
  Serial.print("Topic=");
  Serial.println(buffer);
  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.4.1  connected=true = ");
  Serial.println(ESP.getFreeHeap());
  result = MQTTclient.publish(buffer,  myAddString);
  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.5  connected=true = ");
  Serial.println(ESP.getFreeHeap());
  Serial.print("MQTT publish result=");
  Serial.println(result);
  Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>2.5.6  connected=true = ");
  Serial.println(ESP.getFreeHeap());
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

          AddString += String(moistureSensors[i]) + ",";

        }
        AddString += "\", \"sensorType\": \"";
        String mySensorType = "C1,C1,C1,C1";
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
