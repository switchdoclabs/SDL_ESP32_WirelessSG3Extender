
//
#include "NimBLEDevice.h"

#define FLORADEBUG

struct floraSensorData {
  int temperature;
  int moisture;
  int light;
  int conductivity;
  int battery;
  bool validData;
};

floraSensorData latestFloraData;



// the remote service we wish to connect to
static BLEUUID serviceUUID("00001204-0000-1000-8000-00805f9b34fb");

// the characteristic of the remote service we are interested in
static BLEUUID uuid_version_battery("00001a02-0000-1000-8000-00805f9b34fb");
static BLEUUID uuid_sensor_data("00001a01-0000-1000-8000-00805f9b34fb");
static BLEUUID uuid_write_mode("00001a00-0000-1000-8000-00805f9b34fb");


const String  MQTT_BASE_TOPIC = "flora";

#define BLUETOOTHRETRY 3

boolean readBattery = true;

BLEClient* getFloraClient(BLEAddress floraAddress) {
  BLEClient* floraClient = BLEDevice::createClient();

  if (!floraClient->connect(floraAddress)) {
#ifdef FLORADEBUG
    Serial.println("- Connection failed, skipping");
#endif

    return nullptr;
  }
#ifdef FLORADEBUG
  Serial.println("- Connection successful");
#endif
  return floraClient;
}

BLERemoteService* getFloraService(BLEClient* floraClient) {
  BLERemoteService* floraService = nullptr;

  try {
    floraService = floraClient->getService(serviceUUID);
  }
  catch (...) {
    // something went wrong
  }
  if (floraService == nullptr) {
#ifdef FLORADEBUG
    Serial.println("- Failed to find data service");
#endif
  }
  else {
#ifdef FLORADEBUG
    Serial.println("- Found data service");
#endif
  }

  return floraService;
}

bool forceFloraServiceDataMode(BLERemoteService* floraService) {
  BLERemoteCharacteristic* floraCharacteristic;

  // clear latestFloraData structure
  latestFloraData.temperature = 0;
  latestFloraData.moisture = 0;
  latestFloraData.light = 0;
  latestFloraData.conductivity = 0;
  latestFloraData.battery = 0;
  latestFloraData.validData = false;

  // get device mode characteristic, needs to be changed to read data
#ifdef FLORADEBUG
  Serial.println("- Force device in data mode");
#endif
  floraCharacteristic = nullptr;
  try {
    floraCharacteristic = floraService->getCharacteristic(uuid_write_mode);
  }
  catch (...) {
    // something went wrong
  }
  if (floraCharacteristic == nullptr) {
#ifdef FLORADEBUG
    Serial.println("-- Failed, skipping device");
#endif
    return false;
  }

  // write the magic data
  uint8_t buf[2] = {0xA0, 0x1F};
  floraCharacteristic->writeValue(buf, 2, true);

  vTaskDelay(500 / portTICK_PERIOD_MS);
  return true;
}

bool readFloraDataCharacteristic(BLERemoteService* floraService, String baseTopic) {
  BLERemoteCharacteristic* floraCharacteristic = nullptr;



  // get the main device data characteristic
#ifdef FLORADEBUG
  Serial.println("- Access characteristic from device");
#endif
  try {
    floraCharacteristic = floraService->getCharacteristic(uuid_sensor_data);
  }
  catch (...) {
    // something went wrong
  }
  if (floraCharacteristic == nullptr) {
    #ifdef FLORADEBUG
    Serial.println("-- Failed, skipping device");
    #endif
    return false;
  }

  // read characteristic value
  #ifdef FLORADEBUG
  Serial.println("- Read value from characteristic");
  #endif
  std::string value;
  try {
    value = floraCharacteristic->readValue();
  }
  catch (...) {
    // something went wrong
    #ifdef FLORADEBUG
    Serial.println("-- Failed, skipping device");
    #endif
    return false;
  }
  const char *val = value.c_str();

#ifdef FLORADEBUG
  Serial.print("Hex: ");
  for (int i = 0; i < 16; i++) {
    Serial.print((int)val[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");

#endif

  int16_t* temp_raw = (int16_t*)val;
  int16_t temperature = (*temp_raw);
#ifdef FLORADEBUG
  Serial.print("-- Temperature: ");
  Serial.println(temperature);
#endif
  int moisture = val[7];
#ifdef FLORADEBUG
  Serial.print("-- Moisture: ");
  Serial.println(moisture);
#endif
  int light = val[3] + val[4] * 256;
#ifdef FLORADEBUG
  Serial.print("-- Light: ");
  Serial.println(light);
#endif
  int conductivity = val[8] + val[9] * 256;
#ifdef FLORADEBUG
  Serial.print("-- Conductivity: ");
  Serial.println(conductivity);
#endif

  if (temperature > 2000) {
#ifdef FLORADEBUG
    Serial.println("-- Unreasonable values received, skip publish");
#endif
    latestFloraData.validData = false;
    return false;
  }



  latestFloraData.temperature = temperature;
  latestFloraData.moisture = moisture;
  latestFloraData.light = light;
  latestFloraData.conductivity = conductivity;
  latestFloraData.validData = true;



  return true;
}

bool readFloraBatteryCharacteristic(BLERemoteService * floraService, String baseTopic) {
  BLERemoteCharacteristic* floraCharacteristic = nullptr;

  // get the device battery characteristic
  #ifdef FLORADEBUG
  Serial.println("- Access battery characteristic from device");
#endif
  if (latestFloraData.validData == false)
  {
    // Bad read of data, skip battery
    return false;
  }
  try {
    floraCharacteristic = floraService->getCharacteristic(uuid_version_battery);
  }
  catch (...) {
    // something went wrong
  }
  if (floraCharacteristic == nullptr) {
#ifdef FLORADEBUG
    Serial.println("-- Failed, skipping battery level");
#endif
    latestFloraData.validData = false;
    return false;
  }

  // read characteristic value
#ifdef FLORADEBUG
  Serial.println("- Read value from characteristic");
#endif
  std::string value;
  try {
    value = floraCharacteristic->readValue();
  }
  catch (...) {
    // something went wrong
#ifdef FLORADEBUG
    Serial.println("-- Failed, skipping battery level");
#endif
    latestFloraData.validData = false;
    return false;
  }
  const char *val2 = value.c_str();
  int battery = val2[0];

  latestFloraData.battery = battery;
  latestFloraData.validData = true;
#ifdef FLORADEBUG
  Serial.print("-- Battery: ");
  Serial.println(battery);
#endif
  return true;
}

bool processFloraService(BLERemoteService * floraService, char* deviceMacAddress, bool readBattery) {
  // set device in data mode
  if (!forceFloraServiceDataMode(floraService)) {
    return false;
  }

  String baseTopic = MQTT_BASE_TOPIC + "/" + deviceMacAddress + "/";
  bool dataSuccess = readFloraDataCharacteristic(floraService, baseTopic);

  bool batterySuccess = true;
  if (readBattery) {
    batterySuccess = readFloraBatteryCharacteristic(floraService, baseTopic);
  }

  return dataSuccess && batterySuccess;
}

bool processFloraDevice(BLEAddress floraAddress, char* deviceMacAddress, bool getBattery, int tryCount) {
#ifdef FLORADEBUG
  Serial.print("Processing Flora device at ");
  Serial.print(floraAddress.toString().c_str());
  Serial.print(" (try ");
  Serial.print(tryCount);
  Serial.println(")");
#endif
  // connect to flora ble server
  BLEClient* floraClient = getFloraClient(floraAddress);
  if (floraClient == nullptr) {
    return false;
  }

  // connect data service
  BLERemoteService* floraService = getFloraService(floraClient);
  if (floraService == nullptr) {
    floraClient->disconnect();
    return false;
  }

  // process devices data
  bool success = processFloraService(floraService, deviceMacAddress, getBattery);

  if (success == true)
  {

    int i;
    for (i = 0; i < MAXBLUETOOTHDEVICES; i++)
    {

      if (strcasecmp(BluetoothAddresses[i].c_str(), floraAddress.toString().c_str()) == 0)
      {

        // update last read array

        if (EnglishOrMetric == 0)
        {
          LastTemperatureBluetoothRead[i] = (1.8 * (latestFloraData.temperature / 10.0)) + 32.0;

        }
        else
        {
          LastTemperatureBluetoothRead[i] = latestFloraData.temperature / 10.0;
        }


        LastMoistureBluetoothRead[i] = latestFloraData.moisture;
        i = MAXBLUETOOTHDEVICES;

      }
    }
  }

  // disconnect from device
  floraClient->disconnect();

  return success;
}
