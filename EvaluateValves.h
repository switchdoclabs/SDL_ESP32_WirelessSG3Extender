//
// do realtime evaluation of valves
//


void evaluateValves(int timeDecrementMS);


void DoEvaluateValves()
{

 if (uxSemaphoreGetCount( xSemaphoreEvaluateValves ) > 0)
    {
      xSemaphoreTake( xSemaphoreEvaluatingValves, 10000);
      xSemaphoreTake( xSemaphoreSensorsBeingRead, 10000);
      evaluateValves(100);
      xSemaphoreGive( xSemaphoreSensorsBeingRead);
      xSemaphoreGive( xSemaphoreEvaluatingValves);
      //vTaskDelay(50 / portTICK_PERIOD_MS);

    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  
}

void evaluateValves(int timeDecrementMS)
{
  // Loop Around Valves

#ifdef EXTDEBUG
  //printValveState();
#endif
  int i;
  bool valveChange;
  valveChange = false;

  for (i = 0; i < 8; i++)
  {


    if (valveState[i] == 1)
    {
      valveTime[i] = valveTime[i] - (float)timeDecrementMS / 1000;

      if (valveTime[i] <= 0.0)
      {
        valveTime[i] = 0.0;
        valveState[i] = 0;
        writeValve(i + 1, 0);
        //Serial.print("valve #:");
        //Serial.print(i + 1);
        //Serial.println(" Turned Off");
        valveChange = true;
      }

    }

  }

  if (valveChange == true)
  {
    sendMQTT(MQTTVALVECHANGE, "");
  }



}
