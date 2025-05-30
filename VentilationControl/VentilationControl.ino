/*
 Repeat timer example

 This example shows how to use hardware timer in ESP32. The timer calls onTimer
 function every second. The timer can be stopped with button attached to PIN 0
 (IO0).

 This example code is in the public domain.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "MotorController.h"


// *****************************************************************************
// *****************************************************************************
// Section: Defines
// *****************************************************************************
// *****************************************************************************
#define DEBUG_ENABLE

// *****************************************************************************
// *****************************************************************************
// Section: Variables
// *****************************************************************************
// *****************************************************************************
hw_timer_t* timer = NULL;
volatile SemaphoreHandle_t timerSemaphore;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t DbgCycleCnt;

// volatile uint32_t isrCounter = 0;
// volatile uint32_t lastIsrAt = 0;

// *****************************************************************************
// *****************************************************************************
// Section: CALLBACK Functions
// *****************************************************************************
// *****************************************************************************
void ARDUINO_ISR_ATTR onTimer() {
  // Increment the counter and set the time of ISR
  // portENTER_CRITICAL_ISR(&timerMux);
  // isrCounter++;
  // lastIsrAt = millis();
  // portEXIT_CRITICAL_ISR(&timerMux);

  // Give a semaphore that we can check in the loop
  xSemaphoreGiveFromISR(timerSemaphore, NULL);

  // It is safe to use digitalRead/Write here if you want to toggle an output
}

// *****************************************************************************
// *****************************************************************************
// Section: Functions implemetation
// *****************************************************************************
// *****************************************************************************

//==============================================================================
// FUNCTION
//==============================================================================
void setup()
{
  DbgCycleCnt = 0U;

  Serial.begin(115200);

  // Digital inputs - initialization
  pinMode(I0_0, INPUT);   // User button - ventilation shutter close command
  pinMode(I0_1, INPUT);   // User button - ventilation shutter open command
  pinMode(I0_2, INPUT);   // Sensor - shutter closed
  pinMode(I0_3, INPUT);   // Sensor - shutter open

  // Digital outputs - initialization
  pinMode(Q0_0, OUTPUT);  // ventilation shutter close output
  pinMode(Q0_1, OUTPUT);  // ventilation shutter open output
  pinMode(Q0_2, OUTPUT);  // Output for light control - shutter open or error
  pinMode(Q0_3, OUTPUT);  // Output for light control - shutter closed or error
  // Set outputs to work at supply voltage
  pinMode(S0_24v, OUTPUT);
  pinMode(S1_24v, OUTPUT);
  pinMode(S2_24v, OUTPUT);
  pinMode(S3_24v, OUTPUT); 
  delay(100); // From Industrial Shields example - may not be necessary
  digitalWrite(S0_24v, HIGH);
  digitalWrite(S1_24v, HIGH);
  digitalWrite(S2_24v, HIGH);
  digitalWrite(S3_24v, HIGH);



  // Create semaphore to inform us when the timer has fired
  timerSemaphore = xSemaphoreCreateBinary();

  // Initialize Motor Control component
  MCtrl_Init();


  // ===== Timer settings =====
  // ==========================
  // Use 1st timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32 Technical Reference Manual for more info).
  timer = timerBegin(0, 80, true);

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every 10 milisecconds (value in microseconds).
  // Repeat the alarm (third parameter)
  timerAlarmWrite(timer, 10000, true);

  // Start an alarm
  timerAlarmEnable(timer);
} // setup() end

//==============================================================================
// FUNCTION
//==============================================================================
void loop() {
  // If Timer has fired
  if (xSemaphoreTake(timerSemaphore, 0) == pdTRUE)
  {
    uint32_t isrCount = 0, isrTime = 0;

    // Read the interrupt count and time
    // portENTER_CRITICAL(&timerMux);
    // isrCount = isrCounter;
    // isrTime = lastIsrAt;
    // portEXIT_CRITICAL(&timerMux);

    // Toggle Q0.0
    //digitalWrite(Q0_0, !digitalRead(Q0_0));

     MCtrl_Main();

#ifdef DEBUG_ENABLE
    DbgCycleCnt++;

    if (DbgCycleCnt >= 50U)
    {
      DbgCycleCnt = 0U;

      //uint32_t VentDuctShutterSM_State = Get_VentDuctShutterSM_State();
      Serial.print("VDS_State = "); Serial.print(Get_VentDuctShutterSM_State());
      Serial.print(" ShutterCloseLightOutput_DbgVal = "); Serial.print(Get_MCtrl_StateInidication_ShutterCloseLightOutput_DbgVal());
      Serial.print(" ShutterOpenLightOutput_DbgVal = "); Serial.print(Get_MCtrl_StateInidication_ShutterOpenLightOutput_DbgVal());
      Serial.print(" Error__DbgVal = "); Serial.print(Get_MCtrl_Error_DbgVal());
      Serial.print(" MCtrl_Error = "); Serial.print(Get_MCtrl_ErrorReg());
      Serial.print("\n");
    }
#endif

  }
} // loop() end
