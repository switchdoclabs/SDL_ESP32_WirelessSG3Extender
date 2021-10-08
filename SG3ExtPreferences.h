unsigned int currentLEDMode;



unsigned int darkLight;

String WiFi_SSID = "";
String WiFi_psk = "";
int ClockTimeOffsetToUTC = 0;

void resetPreferences()
{
  preferences.begin("init", false);
  preferences.clear();
  preferences.end();

  ClockTimeOffsetToUTC = DEFAULTCLOCKTIMEOFFSETTOUTC;
  Serial.println("----Clearing Preferences---");


}


void readPreferences();

void writePreferences()
{
  preferences.begin("init", false);




  preferences.putString("WiFi_SSID", WiFi_SSID);
  preferences.putString("WiFi_psk", WiFi_psk);


  preferences.putString("stationName", stationName);

  preferences.putInt("COffsetToUTC", ClockTimeOffsetToUTC);


  preferences.putInt("EnglishOrMetric", EnglishOrMetric);

  preferences.putString("adminPassword", adminPassword);

  preferences.putInt("SolarMAXLA", SolarMAXLA);
  preferences.putInt("SolarMAXLiPo", SolarMAXLiPo);
  preferences.putString("MQTT_IP", MQTT_IP);
  preferences.putInt("MQTT_PORT", MQTT_PORT);
  preferences.putInt("SENSORCYCLE", sensorCycle);

  int i;
  for (i = 0; i < MAXBLUETOOTHDEVICES; i++)
  {
    
    String myString = "BTSENSOR"+String(i);

    preferences.putString(myString.c_str(), BluetoothAddresses[i]);
  }

  preferences.end();



#ifdef EXTDEBUG
  Serial.println("----Writing Preferences---");


  Serial.printf("SSID="); Serial.println(WiFi_SSID);
  Serial.printf("psk="); Serial.println(WiFi_psk);



  Serial.printf("stationName="); Serial.println(stationName);
  Serial.printf("COffsetToUTC="); Serial.println(ClockTimeOffsetToUTC);

  Serial.print("EnglishOrMetric:");
  Serial.println(EnglishOrMetric);
  Serial.print("Station Name:");
  Serial.println(stationName);

  Serial.print("Admin Password:");
  Serial.println(adminPassword.substring(0, 2) + "******");


  Serial.print("SolarMAXLA Enabled=");
  Serial.println(SolarMAXLA);
  Serial.print("SolarMAXLiPo Enabled=");
  Serial.println(SolarMAXLiPo);

  Serial.print("MQTT_IP=");
  Serial.println(MQTT_IP);
  Serial.print("MQTT_PORT=");
  Serial.println(MQTT_PORT);
  Serial.print("SensorCycle=");
  Serial.println(sensorCycle);

  for (i = 0; i < MAXBLUETOOTHDEVICES; i++)
  {

    Serial.print("BluetoothAddresses[");
    Serial.print(i);
    Serial.print("]=");
    Serial.println(BluetoothAddresses[i]);
  }
  Serial.println("--------------------------");

#endif


}

void readPreferences()
{

  Serial.print("preferencesfreeentries=");
  Serial.println(preferences.freeEntries());
  preferences.begin("init", false);


  WiFi_SSID = preferences.getString("WiFi_SSID", "");
  WiFi_psk = preferences.getString("WiFi_psk", "");

  stationName =   preferences.getString("stationName", "");

  ClockTimeOffsetToUTC = preferences.getInt("COffsetToUTC", DEFAULTCLOCKTIMEOFFSETTOUTC);


  EnglishOrMetric = preferences.getInt("EnglishOrMetric");

  adminPassword = preferences.getString("adminPassword", "admin");


  SolarMAXLA = preferences.getInt("SolarMAXLA", 0);
  SolarMAXLiPo = preferences.getInt("SolarMAXLiPo", 0);
  MQTT_IP = preferences.getString("MQTT_IP", "");
  MQTT_PORT = preferences.getInt("MQTT_PORT", 1883);
  sensorCycle = preferences.getInt("SENSORCYCLE", 600);

  int i;
  for (i = 0; i < MAXBLUETOOTHDEVICES; i++)
  {
    String myString = "BTSENSOR"+String(i);
    BluetoothAddresses[i] = preferences.getString(myString.c_str(), "");
  }

  preferences.end();

#ifdef EXTDEBUG
  Serial.println("----Reading Preferences---");



  Serial.printf("SSID="); Serial.println(WiFi_SSID);
  Serial.printf("psk="); Serial.println(WiFi_psk);


  Serial.printf("stationName="); Serial.println(stationName);
  Serial.printf("COffsetToUTC="); Serial.println(ClockTimeOffsetToUTC);

  Serial.print("EnglishOrMetric:");
  Serial.println(EnglishOrMetric);
  Serial.print("Station Name:");
  Serial.println(stationName);

  Serial.print("Admin Password:");
  Serial.println(adminPassword.substring(0, 2) + "******");


  Serial.print("SolarMAXLA Enabled=");
  Serial.println(SolarMAXLA);
  Serial.print("SolarMAXLiPo Enabled=");
  Serial.println(SolarMAXLiPo);
  Serial.print("MQTT_IP=");
  Serial.println(MQTT_IP);
  Serial.print("MQTT_PORT=");
  Serial.println(MQTT_PORT);
  Serial.print("SensorCycle=");
  Serial.println(sensorCycle);

  for (i = 0; i < MAXBLUETOOTHDEVICES; i++)
  {

    Serial.print("BluetoothAddresses[");
    Serial.print(i);
    Serial.print("]=");
    Serial.println(BluetoothAddresses[i]);
  }
  Serial.println("--------------------------");


#endif
}
