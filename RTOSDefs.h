//
// freeRTOS variable Definitions
//



/* Handles for the tasks create by setup(). */




SemaphoreHandle_t  xSemaphoreHydroponicsReadSensor;

SemaphoreHandle_t  xSemaphoreHydroponicsLevelReadSensor;

SemaphoreHandle_t  xSemaphoreRESTCommand;

SemaphoreHandle_t  xSemaphoreSensorsBeingRead;

SemaphoreHandle_t  xSemaphoreEvaluateValves;

SemaphoreHandle_t  xSemaphoreEvaluatingValves;

SemaphoreHandle_t  xSemaphorePixelPulse;

SemaphoreHandle_t  xSemaphoreOLEDLoopUpdate;

SemaphoreHandle_t  xSemaphoreUseI2C;

SemaphoreHandle_t  xSemaphoreReadBluetooth;

SemaphoreHandle_t  xSemaphoreReadInfrared;

SemaphoreHandle_t  xSemaphoreKeepMQTTAlive;



// display semaphores
void printSemaphoreStatus(String Where)
{

  Serial.println("----------------");
  Serial.print("Semaphore Status -");
  Serial.println(Where);
  Serial.println("----------------");
  Serial.println("----------------");
  Serial.println("RTOS Task Enables");
  Serial.println("----------------");


  Serial.print("xSemaphoreHydroponicsReadSensor=");
  Serial.println(uxSemaphoreGetCount( xSemaphoreHydroponicsReadSensor ));

  Serial.print("xSemaphoreHydroponicsLevelReadSensor=");
  Serial.println(uxSemaphoreGetCount(xSemaphoreHydroponicsLevelReadSensor ));

  Serial.print("xSemaphoreRESTCommand=");
  Serial.println(uxSemaphoreGetCount( xSemaphoreRESTCommand ));

  Serial.print("xSemaphoreEvaluateValves=");
  Serial.println(uxSemaphoreGetCount( xSemaphoreEvaluateValves ));

  Serial.print("xSemaphoreOLEDLoopUpdate=");
  Serial.println(uxSemaphoreGetCount( xSemaphoreOLEDLoopUpdate ));

  Serial.print("xSemaphoreReadBluetooth=");
  Serial.println(uxSemaphoreGetCount( xSemaphoreReadBluetooth ));

  Serial.print("xSemaphoreReadInfrared=");
  Serial.println(uxSemaphoreGetCount( xSemaphoreReadInfrared ));

  Serial.print("xSemaphoreKeepMQTTAlive=");
  Serial.println(uxSemaphoreGetCount( xSemaphoreKeepMQTTAlive ));

  Serial.println("----------------");
  Serial.println("RTOS Resource Control");
  Serial.println("----------------");

  Serial.print("xSemaphoreUseI2C=");
  Serial.println(uxSemaphoreGetCount( xSemaphoreUseI2C ));

  Serial.print("xxSemaphoreEvaluatingValves=");
  Serial.println(uxSemaphoreGetCount( xSemaphoreEvaluatingValves ));

  Serial.print("xSemaphoreSensorsBeingRead =");
  Serial.println(uxSemaphoreGetCount( xSemaphoreSensorsBeingRead ));
  
  Serial.print("xSemaphorePixelPulse=");
  Serial.println(uxSemaphoreGetCount( xSemaphorePixelPulse ));

  Serial.println("----------------");
}
