// Filename SDL_ESP32_WirelessSG3Extender
// Started Version 045 December 2021
// SwitchDoc Labs, LLC
//

#define SGSEXTENDERESP32VERSION "050"


#define CONTROLLERBOARD "V1"

#define SGSEXTENDERESP32NRESTPROTOCOL "SGSEXT003"

//#define DEFAULTCLOCKTIMEOFFSETTOUTC -21600
#define DEFAULTCLOCKTIMEOFFSETTOUTC 0

#define WEATHERPLUSMQTTPROTOCOL "SGSEXT001"

#define DEBUG

#define EXTDEBUG

#undef HEAPDEBUG

#define MQTT_DEBUG

#define SX1502ADDRESS 0x20
#define ADS1015ADDRESS 0x49
#define RELAYADDRESS 0x11
#define LCDADDRESS 0x3E


// how often to read the Bluetooth Sensors
//#define BLUETOOTHREADDELAY 900000L
#define BLUETOOTHREADDELAY 60000L

int readCount = 0;
long firstHeap, lastHeap;   // memory problem debug

int bluetoothDeviceCount;

//#include "BLEDevice.h"



#include "MQTTMessages.h"

int sendMQTT(int messageType, String argument);




// BOF preprocessor bug prevent - insert on top of your arduino-code
#if 1
__asm volatile ("nop");
#endif

// Board options

#pragma GCC diagnostic ignored "-Wwrite-strings"
// Board options


/*
  extern "C" {
  #include "user_interface.h"
  }
*/

#if defined(ARDUINO) && ARDUINO >= 100
// No extras
#elif defined(ARDUINO) // pre-1.0
// No extras
#elif defined(ESP_PLATFORM)
#include "arduinoish.hpp"
#endif

#define BLINKPIN 13

#define WEB_SERVER_PORT 80


// display modes
#define DISPLAY_POWERUP 0
#define DISPLAY_IPDISPLAY 1
#define DISPLAY_STATE_SMALL 2
#define DISPLAY_WEATHER_MEDIUM 3
#define DISPLAY_STATE_LARGE 4
#define DISPLAY_STATUS 5
#define DISPLAY_ACCESSPOINT 6
#define DISPLAY_WEATHER_DEMO 7
#define DISPLAY_TRYING_AP 8
#define DISPLAY_FAILING_AP 9
#define DISPLAY_DATETIME 10
#define DISPLAY_UPDATING 11
#define DISPLAY_NO_UPDATE_AVAILABLE 12
#define DISPLAY_NO_UPDATE_FAILED 13
#define DISPLAY_UPDATE_FINISHED 14
#define DISPLAY_SUNAIRPLUS 16
#define DISPLAY_WXLINK 17
#define DISPLAY_SDL2MQTTServer 18
#define DISPLAY_FAILED_RECONNECT 19
#define DISPLAY_LIGHTNING_STATUS 20
#define DISPLAY_LIGHTNING_DISPLAY 21
#define DISPLAY_TRYING_SMARTCONFIG 22
#define DISPLAY_TRYING_WPS 24
#define DISPLAY_DEVICEPRESENT 25

#define DISPLAY_MOISTURE_1 26
#define DISPLAY_MOISTURE_2 27
#define DISPLAY_MOISTURE_3 28
#define DISPLAY_MOISTURE_4 29
#define DISPLAY_IPNAMEID 30
#define DISPLAY_BLUETOOTH 31

// Next display count would be DISPLAY_BLUETOOTH + MAXBLUETOOTHDEVICES (currently 16, so next display would be 47)

#define NUMFLAKES 10
#define XPOS 0
#define YPOS 1
#define DELTAY 2

// Global Variables
int currentRSSI;
//Local WiFi
int WiFiSetupFlag = 0;
String APssid;
String Wssid;
String WPassword;
IPAddress myConnectedIp;
IPAddress myConnectedGateWay;
IPAddress myConnectedMask;
bool WiFiPresent = false;
bool Solar_Present = false;
bool AirQualityPresent = false;
bool TSL2591_Present = false;
bool GPIOExt_Present = false;
bool RELAY_Present = false;
bool ADCExt_Present = false;
bool LCD_Present = false;
bool OneWire_Present = false;
bool Level_Present = false;
bool AMG8833_Present = false;



String stationName;
String adminPassword;
int EnglishOrMetric;   // 0 = English units, 1 = Metric
int SolarMAXLA = 0;
int SolarMAXLiPo = 0;
int sensorCycle = 600;
String MQTT_IP;
int MQTT_PORT;

int HydroponicsMode = 0;
int HydroponicsLevelMode = 0;

String myID;

String macID;

float BatteryVoltage;
float BatteryCurrent;
float LoadVoltage;
float LoadCurrent;
float SolarPanelVoltage;
float SolarPanelCurrent;

// state variables

int valveState[8];
float valveTime[8];

String moistureSensorType[4];


#include "SDL_Arduino_SX1502.h"

SDL_Arduino_SX1502 sx1502(SX1502ADDRESS);

#define MAXBLUETOOTHDEVICES 16
// Bluetooth Sensors array
String BluetoothAddresses[MAXBLUETOOTHDEVICES];
float LastTemperatureBluetoothRead[MAXBLUETOOTHDEVICES];
int LastMoistureBluetoothRead[MAXBLUETOOTHDEVICES];


// main thread task schedule
unsigned long bluetoothPeriod;
unsigned long hydroponicsSensorPeriod;
unsigned long infraredPeriod;
unsigned long valveCheckPeriod;
unsigned long tickPeriod;

unsigned long time_now_1;
unsigned long time_now_2;
unsigned long time_now_3;
unsigned long time_now_4;
unsigned long time_now_5;

// For REST return Values

String RESTreturnString = "";


// RTOS

#include "RTOSDefs.h"


// Import required libraries
#include "TimeLib.h"
#include <TimeLib.h>
#include "Clock.h"
#include "Utility.h"
#include <WiFi.h>

#include <Preferences.h>;


/* create an instance of Preferences library */
Preferences preferences;
#include "SG3ExtPreferences.h"


#include "ArduinoJson-v6.14.1.h"



#include "ValveOutputs.h"

#include <esp_wps.h>
#include <esp_smartconfig.h>


#define ESP_WPS_MODE WPS_TYPE_PBC

// Kludge for latest ESP32 SDK - July 1, 2018

#define WPS_CONFIG_INIT_DEFAULT(type) { \
    .wps_type = type, \
                .crypto_funcs = &g_wifi_default_wps_crypto_funcs, \
  }

esp_wps_config_t config = WPS_CONFIG_INIT_DEFAULT(ESP_WPS_MODE);


// Hydroponics


struct HydroponicsData {
  float temperature = -1000.0;
  int rawTDS = -1;
  int rawTurbidity = -1;
  int rawLevel = -1;
  int rawPh = -1;
};

HydroponicsData latestHydroponicsData;

// AMG8833 pixels

#include "Seeed_AMG8833_driver.h"



float AMG8833_temp[PIXEL_NUM] = {0};

AMG8833 amg8833;

// MQTT




#include "MODaREST.h"





#include "NTPClient.h"

WiFiUDP ntpUDP;


NTPClient timeClient(ntpUDP);




#include <WiFiClientSecure.h>
#include <HTTPClient.h>




#include "WiFiManager.h"          //https://github.com/tzapu/WiFiManager

//gets called when WiFiManager enters configuration mode


void configModeCallback (WiFiManager *myWiFiManager)
//void configModeCallback ()
{

  Serial.println("Entered AP config mode");

  Serial.println(WiFi.softAPIP());

}

#include "aPubSubClient.h"
WiFiClient espClient;
PubSubClient MQTTclient(espClient);
unsigned long MQTTlastMsg = 0;
#define MQTTMSG_BUFFER_SIZE  (400)
char MQTTmsg[MQTTMSG_BUFFER_SIZE];



void MQTTcallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Arrived Topic:");
  Serial.println(topic);
  Serial.print("Message arrived [");

  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  char buffer[400];
  int i;
  for (i = 0; i < length; i++)
  {
    buffer[i] = payload[i];
  }
  buffer[i] = 0;

  StaticJsonDocument<400> myJSON;
  // serialize to JSON then interpret
  DeserializationError error = deserializeJson(myJSON, buffer);
  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }


  int messagetype = myJSON["messagetype"];
  Serial.print("messagetype=");
  Serial.println(messagetype);


  switch (messagetype)
  {

    case MQTTPUBVALVESET:
      {

        xSemaphoreTake( xSemaphoreEvaluatingValves, 1000);
        int i;

        int myValve = myJSON["valve"];
        float myTime = myJSON["timeon"];
        int myState = myJSON["state"];
        if ((myValve < 1) || (myValve > 8))
        {
          xSemaphoreGive( xSemaphoreEvaluatingValves);
          return;
        }

        if (((myState == 0 ) || (myState == 1)) != true)
        {
          xSemaphoreGive( xSemaphoreEvaluatingValves);
          return;
        }

        valveState[myValve - 1] = myState;
        valveTime[myValve - 1] = myTime;


        turnOnAppropriateValves();

#ifdef EXTDEBUG

        Serial.println("____________________________________________");
        printValveState();
        Serial.println("____________________________________________");
#endif

        xSemaphoreGive( xSemaphoreEvaluatingValves);
        break;
      }
    default:

      Serial.print("unsupported incoming MQTT Message:");

      Serial.print(buffer);

      Serial.println();
      break;
  }
  return;

}




void MQTTreconnect(bool reboot) {
  // Loop until we're reconnected
  if (!MQTTclient.connected()) {
    int i = 0;
#ifdef HEAPDEBUG
    Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>6.1  ");
    Serial.println(ESP.getFreeHeap());
#endif
    while (i < 5)
    {
      Serial.print("Attempting MQTT connection...");
      // Create a random client ID
      String clientId = "SGSWireless-";
      clientId += String(random(0xffff), HEX);
      // Attempt to connect
      Serial.print("client name=");
      Serial.println(clientId);
#ifdef HEAPDEBUG
      Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>6.2  ");
      Serial.println(ESP.getFreeHeap());
#endif
      if (MQTTclient.connect(clientId.c_str())) {
        Serial.println("connected");
        // Once connected, publish an announcement...
        //String Topic = "SGS/" + myID;
        //MQTTclient.publish(Topic.c_str(), "hello world");
        String topic;
        topic = "SGS/" + myID + "/Valves";
        Serial.print("sub to topic=");
        Serial.println(topic);
#ifdef HEAPDEBUG
        Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>6.3  ");
        Serial.println(ESP.getFreeHeap());
#endif
        MQTTclient.subscribe(topic.c_str());
#ifdef HEAPDEBUG
        Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>6.3.1  ");
        Serial.println(ESP.getFreeHeap());
#endif
        break;

      } else {
        Serial.print("failed, rc=");
        Serial.print(MQTTclient.state());
#ifdef HEAPDEBUG
        Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>6.4  ");
        Serial.println(ESP.getFreeHeap());
#endif
        Serial.println(" try again in 2 seconds");
        // Wait 2 seconds before retrying
        vTaskDelay(2000 / portTICK_PERIOD_MS);

      }
      i++;
    }
#ifdef HEAPDEBUG
    Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>6.5  ");
    Serial.println(ESP.getFreeHeap());
#endif
    // check for 5 failures and then reboot
    if ((i >= 5) && (reboot == true))
    {
      if (MQTT_IP != "")   // dont reboot if no MQTT IP yet.
      {
        // Force Exception and reboot

        // wipe MQTT_IP out (mistake or reboot)
        MQTT_IP = "";
        writePreferences();
        delay(1000);

        // force divide by zero exception

        int j;

        j = 343 / 0;
        Serial.print (j);
      }
    }
  }
}



// OTA updated
#include <WiFiMulti.h>



#include <HTTPClient.h>
#include <HTTPUpdate.h>



// include GET IP routines
#include "SDL_ESP32_BC24_GETIP.h"



/*

  HslColor hslRed(red);
  HslColor hslGreen(green);
  HslColor hslBlue(blue);
  HslColor hslWhite(white);
  HslColor hslBlack(black);
*/





#undef DEBUGBLYNK

#define BLYNK_NO_BUILTIN

#define BLYNK_PRINT Serial // Defines the object that is used for printing
#undef BLYNK_DEBUG
#define BLYNK_USE_128_VPINS

//#include <BlynkSimpleEsp32.h>






#define min(a,b) ((a)<(b)?(a):(b))

#include "lcddisplay.h"

#include "OLEDDisplay.h"






// Create aREST instance
aREST rest = aREST();



// Create an instance of the server
WiFiServer MyServer(WEB_SERVER_PORT);


// rssi

int fetchRSSI()
{

  wifi_ap_record_t wifidata;
  if (esp_wifi_sta_get_ap_info(&wifidata) == 0) {

    return wifidata.rssi;
  }
  return 0;
}


// NeoPixels
#include "NeoPixelBus.h"


const uint16_t PixelCount = 4; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 21;  // make sure to set this to the correct pin

#define colorSaturation 128

// three element pixels, in different order and speeds
NeoPixelBus<NeoRgbwFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);

RgbwColor green(colorSaturation, 0, 0, 0);
RgbwColor red(0, colorSaturation, 0, 0);
RgbwColor blue(0, 0, colorSaturation, 0);
RgbwColor white( 0, 0, 0, colorSaturation);
RgbwColor black(0);

struct MyAnimationState
{
  RgbwColor StartingColor;
  RgbwColor EndingColor;
};



// GPIO where the DS18B20 is connected to
const int oneWireBus = 16;

#include "OneWire.h"
#include "DallasTemperature.h"

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature DS18B20sensor(&oneWire);

bool checkForOneWire()
{
  DS18B20sensor.requestTemperatures();
  float Temperature = DS18B20sensor.getTempCByIndex(0);
  if (Temperature < -100.0)
    return false;
  else
  {
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    DS18B20sensor.requestTemperatures();
    Temperature = DS18B20sensor.getTempCByIndex(0);
    Serial.print("Initial DS18B20 Senssor =");
    Serial.println(Temperature);
    return true;
  }
}


// readSensors

#include "ReadSensors.h"

// RTOS

#include "RTOSTasks.h"

#include "MQTTFunctions.h"


#include "myPixelRoutines.h"

#include "aRestFunctions.h"
// state initialization

void initialState()
{
  int i;
  for (i = 0; i < 8; i++)
  {
    valveState[i] = 0;  // valve off
    valveTime[i] = 0.0; // valve off

  }

  if (HydroponicsMode == 1)
  {

    moistureSensorType[0] = "TDS1";
    moistureSensorType[1] = "TUR1";
    moistureSensorType[2] = "PH1";
    moistureSensorType[3] = "LFS1";
  }
  else
  {

    moistureSensorType[0] = "C1";
    moistureSensorType[1] = "C1";
    moistureSensorType[2] = "C1";
    moistureSensorType[3] = "C1";
  }


}

#include "I2CScanner.h"

int countBluetoothSensors()
{
  int i;
  int count = 0;
  for (i = 0; i < MAXBLUETOOTHDEVICES; i++)
  {
    if ( BluetoothAddresses[i].length() != 0)
      count++;
  }
  return count;
}






void restartMQTT()
{
  Serial.println("--------");
  Serial.println("MQTT Restart");
  Serial.println("--------");
  /* Serial.print("MQTT_IP=");
    Serial.println(MQTT_IP);
    Serial.print("MQTT_PORT=");
    Serial.println(MQTT_PORT);
  */
  MQTTclient.disconnect();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  MQTTreconnect(true);
  //delete MQTTclient;
  /*
    MQTTclient = PubSubClient(espClient);
    MQTTclient.setServer(MQTT_IP.c_str(), MQTT_PORT);
    MQTTclient.setCallback(MQTTcallback);
    //blinkIPAddress();
    MQTTreconnect(true);
  */
}

bool checkForLevelSensor()
{
  /*
    long RangeInMillimeters;
    RangeInMillimeters = ultrasoniclevel.MeasureInMillimeters();
    if (RangeInMillimeters > 0)
    {
    Serial.println("Ultrasonic Level Found");
    return true;
    }
    else
    {
    Serial.println("Ultrasonic Level NOT Found");
    }
  */
  return true;
}



#include "BluetoothRead.h"

void setup()
{

  Wire.begin();
  int i;

  for (i = 0; i < MAXBLUETOOTHDEVICES; i++)
  {
    BluetoothAddresses[i] = "";
    LastTemperatureBluetoothRead[i] = -1000.0;
    LastMoistureBluetoothRead[i] = -100;
  }


  // Start Serial
  Serial.begin(115200);

  startupSetupGPIOBits();
  Serial.println();
  Serial.println("##############################");
  Serial.println("SmartGarden3 Wireless Extender");
  Serial.println("##############################");
  Serial.print("Software Version = ");
  Serial.println(SGSEXTENDERESP32VERSION);

  readPreferences();

  // Init Bluetooth Addresses



  pinMode(15, INPUT);

  ads1015.begin();


  if (checkForI2CAddress(0x11))
  {
    Serial.println("Relay Present - 0x11");
    RELAY_Present = true;
  }
  else
    Serial.println("Relay NOT Present");

  if (checkForI2CAddress(0x20))
  {
    Serial.println("GPIO Extender Present - 0x20");
    GPIOExt_Present = true;
  }
  else
    Serial.println("GPIO Extender NOT Present");

  if (checkForI2CAddress(0x3E))
  {
    Serial.println("LCD Present - 0x3E");
    LCD_Present = true;
  }
  else
    Serial.println("LCD NOT Present");

  if (checkForI2CAddress(0x49))
  {
    Serial.println("ADC Present - 0x49");
    ADCExt_Present = true;
  }
  else
    Serial.println("ADC NOT Present");

  if (checkForI2CAddress(0x68))
  {
    Serial.println("AMG8833 Present - 0x68");
    AMG8833_Present = true;
  }
  else
    Serial.println("AMG8833 NOT Present");

  if (checkForOneWire())
  {
    Serial.println("TempHum Present - OneWire");
    OneWire_Present = true;
  }
  else
    Serial.println("TempHum NOT Present");

  if (checkForLevelSensor())
  {
    Serial.println("Water Level Sensor Present - ADC3");

    Level_Present = true;
  }
  else
    Serial.println("Water Level Sensor NOT Present");
  // start up lcd
  if (LCD_Present)
  {
    startlcd();
  }


  // Start the DS18B20 sensor
  if (OneWire_Present)
  {
    DS18B20sensor.begin();
  }


  // start the Infrared camera if present
  if (AMG8833_Present)
  {
    if (amg8833.init()) {
      Serial.println("AMG8833 Not Present");
      AMG8833_Present == false;

    }
    else
      Serial.println("AMG8833 Present");
  }

  // this resets all the neopixels to an off state
  strip.Begin();
  strip.Show();
  strip.SetPixelColor(0, black);

  strip.Show();


  if (digitalRead(15) == 0)
  {
    resetPreferences();
    Serial.println("Preferences Reset by AUX Button");

  }


  // RTOS


  xSemaphoreRESTCommand = xSemaphoreCreateBinary();
  xSemaphoreGive( xSemaphoreRESTCommand);   // initialize
  //xSemaphoreTake( xSemaphoreRESTCommand, 10);   // start with this off

  xSemaphoreHydroponicsReadSensor = xSemaphoreCreateBinary();
  xSemaphoreGive( xSemaphoreHydroponicsReadSensor);   // initialize
  Serial.print("HydroponicsMode - xReadSensor=");
  Serial.println(HydroponicsMode);
  if (HydroponicsMode == 0)
  {
    xSemaphoreTake( xSemaphoreHydroponicsReadSensor, 10);   // start with this off
  }



  xSemaphoreHydroponicsLevelReadSensor = xSemaphoreCreateBinary();
  xSemaphoreGive( xSemaphoreHydroponicsLevelReadSensor);   // initialize
  xSemaphoreTake( xSemaphoreHydroponicsLevelReadSensor, 10);   // start with this off


  xSemaphoreSensorsBeingRead = xSemaphoreCreateBinary();
  xSemaphoreGive( xSemaphoreSensorsBeingRead);   // initialize it on

  xSemaphoreEvaluateValves = xSemaphoreCreateBinary();
  xSemaphoreGive( xSemaphoreEvaluateValves);   // initialize it on

  xSemaphoreEvaluatingValves = xSemaphoreCreateBinary();
  xSemaphoreGive( xSemaphoreEvaluatingValves);   // initialize
  xSemaphoreTake( xSemaphoreEvaluatingValves, 10);   // start with this off

  xSemaphorePixelPulse = xSemaphoreCreateBinary();
  xSemaphoreGive( xSemaphorePixelPulse);   // initialize
  xSemaphoreTake( xSemaphorePixelPulse, 10);   // start with this off


  xSemaphoreOLEDLoopUpdate = xSemaphoreCreateBinary();
  xSemaphoreGive( xSemaphoreOLEDLoopUpdate);   // initialize it on
  xSemaphoreTake( xSemaphoreOLEDLoopUpdate, 10);   // start with this off


  xSemaphoreReadBluetooth = xSemaphoreCreateBinary();
  xSemaphoreGive( xSemaphoreReadBluetooth);   // initialize it on
  xSemaphoreTake( xSemaphoreReadBluetooth, 10);   // start with this off

  xSemaphoreUseI2C = xSemaphoreCreateBinary();
  xSemaphoreGive( xSemaphoreUseI2C);   // initialize

  xSemaphoreReadInfrared = xSemaphoreCreateBinary();
  xSemaphoreGive( xSemaphoreReadInfrared);   // initialize it on
  xSemaphoreTake( xSemaphoreReadInfrared, 10);   // start with this off




  xSemaphoreKeepMQTTAlive = xSemaphoreCreateBinary();
  xSemaphoreGive( xSemaphoreKeepMQTTAlive);   // initialize it on
  //xSemaphoreTake( xSemaphoreKeepMQTTAlive, 10);   // start with this off

  Serial.print("StartxSemaphoreUseI2C=");
  Serial.println(uxSemaphoreGetCount( xSemaphoreUseI2C ));

  printSemaphoreStatus("After Semaphore Initialization");

  // set all valves I/Os to off

  initializeValvesAndSensors();



  //---------------------
  // Setup WiFi Interface
  //---------------------
  // 5 cases:
  // 1) Use stored SSID/psk
  // 2) SmartConfig received
  // 3) WPS Button
  // 4) Local AP
  // 5) Wifi Fails


  WiFiPresent = false;
  WiFi.persistent(false);
  WiFi.begin();

#define WL_MAC_ADDR_LENGTH 6
  // Append the last two bytes of the MAC (HEX'd) to string to make unique
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);
  macID = String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
          String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();
  myID = macID;

  // check for SSID

  if (WiFi_SSID.length() != 0)
  {
    // use existing SSID
    Serial.println("Using existing SSID/psk");

    Serial.printf("SSID="); Serial.println(WiFi_SSID);
    Serial.printf("psk="); Serial.println(WiFi_psk);
    WiFi.begin(WiFi_SSID.c_str(), WiFi_psk.c_str());
    //Wait for WiFi to connect to AP
    Serial.println("Waiting for Saved WiFi");
#define WAITFORCONNECT 15
    for (int i = 0; i < WAITFORCONNECT * 2; i++)
    {
      if (WiFi.status() == WL_CONNECTED)
      {

        Serial.println("");
        Serial.println("WiFI Connected.");

        Serial.printf("SSID="); Serial.println(WiFi_SSID);
        Serial.printf("psk="); Serial.println(WiFi_psk);
        WiFiPresent = true;




        break;
      }

      Serial.print(".");
      WiFiPresent = false;
      //blinkPixel(0, 1, red, 300);
      vTaskDelay(1000 / portTICK_PERIOD_MS);

    }
    Serial.println();

  }

  /*
    if (WiFiPresent == false)
    {
      // do SmartConfig
    #define WAITFORSTART 15
    #define WAITFORCONNECT 20
      updateDisplay(DISPLAY_TRYING_SMARTCONFIG);

      WiFiPresent = SmartConfigGetIP(WAITFORSTART, WAITFORCONNECT);

    }

    if (WiFiPresent != true)
    {
    #define WPSTIMEOUTSECONDS 30
      // now try WPS Button
      updateDisplay(DISPLAY_TRYING_WPS);
      WiFiPresent = WPSGetIP(WPSTIMEOUTSECONDS);

    }
  */

  if (WiFiPresent != true)
  {
#define APTIMEOUTSECONDS 300
    updateDisplay(DISPLAY_TRYING_AP);
    WiFiPresent = localAPGetIP(APTIMEOUTSECONDS);
  }

  if (WiFiPresent == false)
  {
    // reboot and try again (this fixes the WiFi down recovery problem and gives a forever time to use the AP to setup)

    // force divide by zero exception

    int j;

    j = 343 / 0;
    Serial.print (j);
  }


  if (WiFiPresent == true)
  {


    WiFi_SSID = WiFi.SSID();
    WiFi_psk = WiFi.psk();
  }
  // write out preferences

  writePreferences();
  if (WiFiPresent == true)
  {
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    Serial.print("WiFi Channel= ");
    Serial.println(WiFi.channel());

    Serial.print("Gateway Address:");
    myConnectedGateWay = WiFi.gatewayIP();
    Serial.println(WiFi.gatewayIP());

    Serial.print("Subnet Mask:");
    myConnectedMask = WiFi.subnetMask();
    Serial.println(WiFi.subnetMask());


    Serial.println("--------");
    Serial.println("MQTT Start");
    Serial.println("--------");
    Serial.print("MQTT_IP=");
    Serial.println(MQTT_IP);
    Serial.print("MQTT_PORT=");
    Serial.println(MQTT_PORT);
    MQTTclient.setServer(MQTT_IP.c_str(), MQTT_PORT);
    MQTTclient.setCallback(MQTTcallback);
    //blinkIPAddress();
    MQTTreconnect(false);// should be true?


    updateDisplay(DISPLAY_IPDISPLAY);
  }

  blinkPixel(0, 3, blue, 150);

  // send debug boot up MQTT message


  currentRSSI = fetchRSSI();
  printf("rssi:%d\r\n", currentRSSI);
  //---------------------
  // End Setup WiFi Interface
  //---------------------


  // Now do NTP (set's time to 00:00:00 if not present)  January 1, 1970

  Serial.println("--------");
  Serial.println("NTP Time Fetch");
  Serial.println("--------");

  // changed later with setTimeOffset() ). Additionaly you can specify the
  // update interval (in milliseconds, can be changed using setUpdateInterval() ).
  timeClient.begin();
  //timeClient.forceUpdate();

  timeClient.setTimeOffset(ClockTimeOffsetToUTC);
  if (WiFiPresent == true)
  {
    timeClient.setUpdateInterval(3600000);
    //
    timeClient.update();
  }
  time_t t;
  if (WiFiPresent == true)
  {
    /* Serial.print("Getting epochTime = ");
      t = timeClient.getEpochTime();
      Serial.println(t);
      if (t < 2000)
      {
       // Try again to update
       timeClient.forceUpdate();

       Serial.print("Getting epochTime Try 2= ");
       t = timeClient.getEpochTime();
       Serial.println(t);

       // and again
       if (t < 2000)
       {
         timeClient.forceUpdate();
         Serial.print("Getting epochTime Try 3= ");
         t = timeClient.getEpochTime();
         Serial.println(t);

       }

      }
    */
    int timeout;
    timeout = 0;
    while (!timeClient.update()) {
      timeClient.forceUpdate();
      timeout++;
      if (timeout > 1000) {
        Serial.println("NTP Timeout - time set to default");
        break;
      }


    }

    Serial.print("NTP timeout count = ");
    Serial.println(timeout);
    Serial.print("Getting epochTime = ");
    t = timeClient.getEpochTime();
    Serial.println(t);
  }
  else
  {
    Serial.println("No WiFi Present - NTP not set");
    t = 1;
  }


  setTime(t);


  digitalClockDisplay();


  // start up REST



  MyServer.begin();


  // Function to be exposed
  rest.function("led", ledControl);
  rest.function("setValves", setValves);
  rest.function("checkForID", checkForID);
  rest.function("getValveState", getValveState);
  rest.function("setSingleValve", setSingleValve);

  rest.function("readHydroponicsSensors", readHydroponicsSensorsCommand);
  rest.function("blinkPixelCommand", blinkPixelCommand);
  rest.function("setStationName", setStationName);
  rest.function("setClockOffset", setClockOffset);
  rest.function("setSensorCycle", setSensorCycle);
  rest.function("restartMQTT", restartMQTT);
  rest.function("assignBluetoothSensors", assignBluetoothSensors);
  rest.function("enableHydroponicsMode", enableHydroponicsMode);

  rest.function("updateSGS", updateSGS);

  // Give name & ID to the device (ID should be 6 characters long)
  rest.set_id(myID);
  rest.set_name("SG3WEXTV1");



  Serial.print("CPU0 reset reason: ");
  print_reset_reason(rtc_get_reset_reason(0));
  Serial.println(rtc_get_reset_reason(0));

  Serial.print("CPU1 reset reason: ");
  print_reset_reason(rtc_get_reset_reason(1));

  // send debug boot up MQTT message
#ifdef EXTDEBUG
  String myMQTTMessage;
  myMQTTMessage = SGSEXTENDERESP32VERSION;
  myMQTTMessage += ", reboot, CPU0 reset reason: ";
  myMQTTMessage +=  String(return_reset_reason(rtc_get_reset_reason(0)));
  myMQTTMessage += ", reboot, CPU1 reset reason: ";
  myMQTTMessage +=  String( return_reset_reason(rtc_get_reset_reason(1)));


  sendMQTT(MQTTREBOOT, myMQTTMessage);
#endif

  if (LCD_Present) {

    updateDisplay(DISPLAY_POWERUP);
  }


  // Serial.print("Pre Initial State SetupxSemaphoreUseI2C=");
  //Serial.println(uxSemaphoreGetCount( xSemaphoreUseI2C ));
  // initialize valves
  initialState();
  /*
    xSemaphoreTake( xSemaphoreSensorsBeingRead, 10000);

    if (HydroponicsMode == 1)
    {
      readHydroponicsSensors();
      sendMQTT(MQTTHYDROPONICS, "");
    }
    xSemaphoreGive( xSemaphoreSensorsBeingRead);
  */
  if (AMG8833_Present == true)
  {
    xSemaphoreGive( xSemaphoreReadInfrared);   // start the reads

  }


  //xTaskCreatePinnedToCore(
  //taskReadSendHydroponics,          /* Task function. */
  //"taskReadSendHydroponics",        /* String with name of task. */
  //7000,            /* Stack size in words. */
  //NULL,             /* Parameter passed as input of the task */
  //5,                /* Priority of the task. */
  //NULL,             /* Task handle. */
  //0);               // Specific Core


  // xTaskCreatePinnedToCore(
  //   taskReadSendHydroponicsLevel,          /* Task function. */
  //   "taskReadSendHydroponicsLevel",        /* String with name of task. */
  //   7000,            /* Stack size in words. */
  //   NULL,             /* Parameter passed as input of the task */
  //  2,                /* Priority of the task. */
  //  NULL,             /* Task handle. */
  //  1);               // Specific Core
  //Serial.print("Pre taskREST SetupxSemaphoreUseI2C=");
  //Serial.println(uxSemaphoreGetCount( xSemaphoreUseI2C ));

  xTaskCreatePinnedToCore(
    taskRESTCommand,          /* Task function. */
    "TaskRESTCommand",        /* String with name of task. */
    7000,            /* Stack size in words. */
    NULL,             /* Parameter passed as input of the task */
    3,                /* Priority of the task. */
    NULL,             /* Task handle. */
    0);               // Specific Core
  //Serial.print("Pre taskSetValues SetupxSemaphoreUseI2C=");
  // Serial.println(uxSemaphoreGetCount( xSemaphoreUseI2C ));

  //xTaskCreatePinnedToCore(
  //  taskSetValves,          /* Task function. */
  // "taskSetValves",        /* String with name of task. */
  // 7000,            /* Stack size in words. */
  // NULL,             /* Parameter passed as input of the task */
  //  3,                /* Priority of the task. */
  //  NULL,             /* Task handle. */
  //  1);               // Specific Core
  //Serial.print("Pre taskSetValues SetupxSemaphoreUseI2C=");
  //Serial.println(uxSemaphoreGetCount( xSemaphoreUseI2C ));

  xTaskCreatePinnedToCore(
    taskPixelCommand,          /* Task function. */
    "TaskPixelCommand",        /* String with name of task. */
    7000,            /* Stack size in words. */
    NULL,             /* Parameter passed as input of the task */
    3,                /* Priority of the task. */
    NULL,             /* Task handle. */
    1);               // Specific Core


  // xTaskCreatePinnedToCore(
  //  taskReadBluetooth,          /* Task function. */
  //  "taskReadBluetooth",        /* String with name of task. */
  //   20000,            /* Stack size in words. */
  //   NULL,             /* Parameter passed as input of the task */
  //   2,                /* Priority of the task. */
  //   NULL,             /* Task handle. */
  //   0);               // Specific Core


  xTaskCreatePinnedToCore(
    taskMainLCDLoopDisplay,          /* Task function. */
    "taskMainLCDLoopDisplay",        /* String with name of task. */
    7000,            /* Stack size in words. */
    NULL,             /* Parameter passed as input of the task */
    2,                /* Priority of the task. */
    NULL,             /* Task handle. */
    1);               // Specific Core


  //xTaskCreatePinnedToCore(
  //  taskFetchInfrared,          /* Task function. */
  //  "taskFetchInfrared",        /* String with name of task. */
  //  5000,            /* Stack size in words. */
  //  NULL,             /* Parameter passed as input of the task */
  //  2,                /* Priority of the task. */
  //  NULL,             /* Task handle. */
  //  1);               // Specific Core


  xTaskCreatePinnedToCore(
    taskKeepMQTTAlive,          /* Task function. */
    "taskKeepMQTTAlive",        /* String with name of task. */
    5000,            /* Stack size in words. */
    NULL,             /* Parameter passed as input of the task */
    2,                /* Priority of the task. */
    NULL,             /* Task handle. */
    1);               // Specific Core

  Serial.println("RTOS Tasks Starting");

  //Serial.print("EndSetupxSemaphoreUseI2C=");
  //Serial.println(uxSemaphoreGetCount( xSemaphoreUseI2C ));


  // Start OLED Loop

  vTaskDelay(2000 / portTICK_PERIOD_MS);
  //Serial.print("BUpdateSemaphoreUseI2C=");
  //Serial.println(uxSemaphoreGetCount( xSemaphoreUseI2C ));
  xSemaphoreGive( xSemaphoreOLEDLoopUpdate);   // initialize it on

  bluetoothDeviceCount = countBluetoothSensors();

  Serial.print("countBlueToothSensors=");
  Serial.println(bluetoothDeviceCount);
  if (bluetoothDeviceCount > 0)
  { // Startup Task
    Serial.println("Starting up Bluetooth");

    Serial.println("Initialize BLE client...");
    BLEDevice::init("");
    BLEDevice::setPower(ESP_PWR_LVL_P7);
    xSemaphoreGive( xSemaphoreReadBluetooth);

  }

  bluetoothPeriod = (unsigned long)sensorCycle * 1000l;
  hydroponicsSensorPeriod = (unsigned long)sensorCycle * 1000l;
  infraredPeriod = (unsigned long)sensorCycle * 1000l;
  valveCheckPeriod = 100l;
  tickPeriod = 6000l;

  unsigned long temp;
  temp = millis() - 10l;
  time_now_1 = temp - bluetoothPeriod;  // so will execute at start
  time_now_2 = temp - hydroponicsSensorPeriod;
  time_now_3 = temp - infraredPeriod;
  time_now_4 = temp - valveCheckPeriod;
  time_now_5 = temp - tickPeriod;

  Serial.println("Main Thread Task Start");
} //end setup





void loop() {

  MQTTclient.loop();
  /*
    Serial.printf("millis - time_now_1=%d\n", (unsigned long) (millis() - time_now_1));
    Serial.print("loop() running on core ");
    Serial.println(xPortGetCoreID());
    Serial.print("Loop: priority = ");
    Serial.println(uxTaskPriorityGet(NULL));
  */


  if ((unsigned long)(millis() - time_now_5) > tickPeriod) {
    time_now_5 = millis();
    Serial.print("Tick - ");
    digitalClockDisplay();
    Serial.println();

    IPAddress myIp2 = WiFi.localIP();
    Serial.printf("LocalIP=%d.%d.%d.%d\n", myIp2[0], myIp2[1], myIp2[2], myIp2[3]);

  }

  // build main thread non-threaded scheduler for i2c issues
  if ((unsigned long)(millis() - time_now_1) > bluetoothPeriod) {
    time_now_1 = millis();
    Serial.println("bluetoothPeriod");
    readBluetooth();
  }

  if ((unsigned long)(millis() - time_now_2) > hydroponicsSensorPeriod) {
    time_now_2 = millis();
    Serial.println("hydroponicsSensorPeriod");
    readAndSendHydroponicSensors();
  }

  if ((unsigned long)(millis() - time_now_3) > infraredPeriod) {
    time_now_3 = millis();
    Serial.println("infraredPeriod");
    ReadAndSendAMG8833Sensor();
  }


  if ((unsigned long)(millis() - time_now_4) > valveCheckPeriod) {
    time_now_4 = millis();
    //Serial.println("valveCheckPeriod");
    DoEvaluateValves();
  }





  vTaskDelay(10 / portTICK_PERIOD_MS);

}
