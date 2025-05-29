
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include "esp32-hal-gpio.h"
#include "bsw_cntman.h"
#include "MotorController.h"

// *****************************************************************************
// *****************************************************************************
// Section: Defines
// *****************************************************************************
// *****************************************************************************


// *****************************************************************************
// *****************************************************************************
// Section: Type definitions
// *****************************************************************************
// *****************************************************************************
// Define Digital Output states
typedef enum
{
  OFF = 0u,
  ON
} DigitaInputlOuptut_State_t;

// States of SM that controls the Sutter
typedef enum
{
  Idle = 0,
  Opening,
  Closing,
  Error
} VentDuctShutterSM_t;

// Error IDs
typedef enum uint32_t
{
  NoErr = 0,
  ErrTimeOutClosing = 2,
  ErrTimeOutOpening = 4,
  ErrLostClosedSensor = 8,
  ErrLostOpenSensor = 16,
  ErrBothSensorsConfirmed = 32
} error_t;

// System state
// typedef enum
// {
//   SysState_Init,
//   SysState_ShutterClosing,
//   SysState_ShutterOpening,
//   SysState_ShutterIsOpen,
//   SysState_ShutterIsClosed,
//   SysState_ErrClosingTimeout,
//   SysState_ErrOpeningTimeout,
//   SysState_ErrBothSensorsConfirmed,
// } SystemState_t;

// *****************************************************************************
// *****************************************************************************
// Section: Variables
// *****************************************************************************
// *****************************************************************************
static VentDuctShutterSM_t VentDuctShutterSM;
static bool ShutterOpen_BtnState, ShutterOpen_BtnState_Previous;
static bool ShutterClose_BtnState, ShutterClose_BtnState_Previous;
static bool ev_ShutterOpen_Btn_Pressed;
static bool ev_ShutterClose_Btn_Pressed;
static bool ev_ShutterOPEN_Btn_Pressed_Signaling;
static bool ev_ShutterCLOSE_Btn_Pressed_Signaling;
static error_t ErrorRegister1;
static T_COUNT_L toutClosing;
static T_COUNT_L toutOpening;
static uint32_t ShutterCloseLightOutput_DbgVal;
static uint32_t ShutterOpenLightOutput_DbgVal;

static const uint32_t TIMEOUT_CLOSING = 3000; // 30 seconds = 30000 ms; 30000 ms / 10ms = 3000
static const uint32_t TIMEOUT_OPENING = 3000; // same as above

static const uint32_t TIME_BLINK_ON_MOT = 50;  // when motor moving output is 0.5 s ON then 0.5 s OFF
static const uint32_t TIME_BLINK_ON_ERR = 35;  // when error is present output is 0.25 s ON then 0.25 s OFF
uint32_t ShutterClose_Blink_Duration;
uint32_t ShutterOpen_Blink_Duration;
static T_COUNT_W ShutterClose_Blink_Tmr;    // 16 bit timer for implemented indication output control (this output controls an LED)
static T_COUNT_W ShutterOpen_Blink_Tmr;


// *****************************************************************************
// *****************************************************************************
// Section: Local functions declaration
// *****************************************************************************
// *****************************************************************************
static void SetOutput_ShutterOpen(DigitaInputlOuptut_State_t state);
static void SetOutput_ShutterClose(DigitaInputlOuptut_State_t state);
static void SetOutput_Light_ShutterOpen(DigitaInputlOuptut_State_t state);
static void SetOutput_Light_ShutterClose(DigitaInputlOuptut_State_t state);
static DigitaInputlOuptut_State_t GetShutterOpenSensorState(void);
static DigitaInputlOuptut_State_t GetShutterCloseSensorState(void);
static DigitaInputlOuptut_State_t ShutterOpen_BtnPressed(void);
static DigitaInputlOuptut_State_t ShutterClose_BtnPressed(void);

static void ErrClearAllErrors(error_t * const errReg);
static void ErrSetError(error_t * const errReg, error_t err);
//static void ErrClrError(error_t * const errReg, error_t err);
//static bool ErrGetState(error_t * const errReg, error_t err);

static void MCtrl_Main_Begin(void);
static void MCtrl_Main_End(void);
static void MCtrl_StateInidication(void);


// *****************************************************************************
// *****************************************************************************
// Section: Functions implemetation
// *****************************************************************************
// *****************************************************************************

//==============================================================================
// FUNCTION
//==============================================================================
void MCtrl_Init(void)
{
  ev_ShutterOpen_Btn_Pressed = false;
  ev_ShutterClose_Btn_Pressed = false;
  ev_ShutterOPEN_Btn_Pressed_Signaling = false;
  ev_ShutterCLOSE_Btn_Pressed_Signaling = false;


  ShutterClose_Blink_Duration = 0u;
  ShutterOpen_Blink_Duration = 0u;

  ErrClearAllErrors(&ErrorRegister1);

  STOP_COUNT_L(toutClosing);
  STOP_COUNT_L(toutOpening);
  STOP_COUNT_W(ShutterClose_Blink_Tmr);
  STOP_COUNT_W(ShutterOpen_Blink_Tmr);

  ShutterOpen_BtnState_Previous = ShutterOpen_BtnState = ShutterOpen_BtnPressed();
  ShutterClose_BtnState_Previous = ShutterClose_BtnState = ShutterClose_BtnPressed();
}

//==============================================================================
// FUNCTION
//==============================================================================
void MCtrl_Main(void)
{
  MCtrl_Main_Begin();

  // Process timers used by this functionality
  bsw_DecCnt_Long(&toutClosing);
  bsw_DecCnt_Long(&toutOpening);



  // *********************************************
  // State machine to open/close ventilation duct
  // *********************************************
  switch (VentDuctShutterSM)
  {
    case Idle:
      // User button Open Shutter was pressed
      if (ev_ShutterOpen_Btn_Pressed)
      {
        // Use and clear (consume) the event
        ev_ShutterOpen_Btn_Pressed = false;
        // Clear all exisiting errors
        ErrClearAllErrors(&ErrorRegister1);

        // Start motor - opening direction
        SetOutput_ShutterOpen(ON);
        // Start operation monitoring timer
        bsw_LoadCnt_Long(&toutOpening, TIMEOUT_OPENING);

        // Go to opening state
        VentDuctShutterSM = Opening;
      }

      // User button Close Shutter was pressed
      if (ev_ShutterClose_Btn_Pressed)
      {
        // Use and clear (consume) the event
        ev_ShutterClose_Btn_Pressed = false;
        // Clear all exisiting errors
        ErrClearAllErrors(&ErrorRegister1);

        // Start motor - closing direction
        SetOutput_ShutterClose(ON);
        bsw_LoadCnt_Long(&toutClosing, TIMEOUT_CLOSING);

        // Go to closing state
        VentDuctShutterSM = Closing;
      }

      // Both sensors are ON => error
      if ( (GetShutterOpenSensorState() == ON) && (GetShutterCloseSensorState() == ON) )
      {
        // Error
        ErrSetError(&ErrorRegister1, ErrBothSensorsConfirmed);
      }
    break;

    case Opening:
      // End position sensor detected OR Shutter Close button is pressed then stop the movement
      if (GetShutterOpenSensorState() == ON)
      {
        // Stop monitoring timer
        STOP_COUNT_L(toutOpening);
        // Stop motor
        SetOutput_ShutterOpen(OFF);
        // Go to IDLE state
        VentDuctShutterSM = Idle;
      }
      else if (ev_ShutterClose_Btn_Pressed)
      {
        ev_ShutterClose_Btn_Pressed = false;
        // Stop monitoring timer
        STOP_COUNT_L(toutOpening);
        // Stop motor
        SetOutput_ShutterOpen(OFF);
        // Go to IDLE state
        VentDuctShutterSM = Idle;
      }
      else
      {
        // Timeout - sensor not found after a given time
        if ( bsw_GetCnt_Long(&toutOpening) == L_TIMER_OFF )
        {
          // Error
          ErrSetError(&ErrorRegister1, ErrTimeOutOpening);
          // Go to Error state
          VentDuctShutterSM = Error;
        }
      }
    break;

    case Closing:
      // End position sensor detected OR Shutter Open button is pressed then stop the movement
      if (GetShutterCloseSensorState() == ON)
      {
        // Stop monitoring timer
        STOP_COUNT_L(toutClosing);
        // Stop motor
        SetOutput_ShutterClose(OFF);
        // Go to IDLE state
        VentDuctShutterSM = Idle;
      }
      else if (ev_ShutterOpen_Btn_Pressed)
      {
        ev_ShutterOpen_Btn_Pressed = false;
        // Stop monitoring timer
        STOP_COUNT_L(toutClosing);
        // Stop motor
        SetOutput_ShutterClose(OFF);
        // Go to IDLE state
        VentDuctShutterSM = Idle;
      }
      else
      {
        // Timeout - sensor not found after a given time
        if ( bsw_GetCnt_Long(&toutClosing) == L_TIMER_OFF )
        {
          // Error
          ErrSetError(&ErrorRegister1, ErrTimeOutClosing);
          // Go to Error state
          VentDuctShutterSM = Error;
        }
      }
    break;

    case Error:
      // Stop motor
      SetOutput_ShutterOpen(OFF);
      SetOutput_ShutterClose(OFF);
      
      // Go to IDLE state
      VentDuctShutterSM = Idle;
    break;
  
    default:
      VentDuctShutterSM = Idle;
    break;
  }

  // State (and alarms) signalling
  MCtrl_StateInidication();

  MCtrl_Main_End();
} // End of MCtrl_Main(void)

//==============================================================================
// FUNCTION
//==============================================================================
uint32_t Get_VentDuctShutterSM_State(void)
{
  return (uint32_t)VentDuctShutterSM;
}

//==============================================================================
// FUNCTION
//==============================================================================
uint32_t Get_MCtrl_ErrorReg(void)
{
  return ErrorRegister1;
}

//==============================================================================
// FUNCTION
//==============================================================================
uint32_t Get_MCtrl_StateInidication_ShutterCloseLightOutput_DbgVal(void)
{
  return ShutterCloseLightOutput_DbgVal;
}

//==============================================================================
// FUNCTION
//==============================================================================
uint32_t Get_MCtrl_StateInidication_ShutterOpenLightOutput_DbgVal(void)
{
  return ShutterOpenLightOutput_DbgVal;
}



// *****************************************************************************
// *****************************************************************************
// Section: PRIVATE FUNCTIONS
// *****************************************************************************
// *****************************************************************************

//==============================================================================
// FUNCTION
//==============================================================================
static void MCtrl_Main_Begin(void)
{
  // User buttons to control ventilation duct - edge detection
  // **********************************************************
  ShutterOpen_BtnState = ShutterOpen_BtnPressed();
  if ( (ShutterOpen_BtnState_Previous == false) && (ShutterOpen_BtnState == true) )
  {
    ev_ShutterOpen_Btn_Pressed = true;
    ev_ShutterOPEN_Btn_Pressed_Signaling = true;
  }
  else
  {
    ev_ShutterOpen_Btn_Pressed = false;
    ev_ShutterOPEN_Btn_Pressed_Signaling = false;
  }

  ShutterClose_BtnState = ShutterClose_BtnPressed();
  if ( (ShutterClose_BtnState_Previous == false) && (ShutterClose_BtnState == true) )
  {
    ev_ShutterClose_Btn_Pressed = true;
    ev_ShutterCLOSE_Btn_Pressed_Signaling = true;
  }
  else
  {
    ev_ShutterClose_Btn_Pressed = false;
    ev_ShutterCLOSE_Btn_Pressed_Signaling = false;
  }

  ShutterOpen_BtnState_Previous = ShutterOpen_BtnState;
  ShutterClose_BtnState_Previous = ShutterClose_BtnState;
}

//==============================================================================
// FUNCTION
//==============================================================================
static void MCtrl_Main_End(void)
{
  
}

//==============================================================================
// FUNCTION
//==============================================================================
static void SetOutput_ShutterOpen(DigitaInputlOuptut_State_t state)
{
  if (state == ON)
  {
    digitalWrite(Q0_0, HIGH);
  }
  else
  {
    digitalWrite(Q0_0, LOW);
  }
}

//==============================================================================
// FUNCTION
//==============================================================================
static void SetOutput_ShutterClose(DigitaInputlOuptut_State_t state)
{
  if (state == ON)
  {
    digitalWrite(Q0_1, HIGH);
  }
  else
  {
    digitalWrite(Q0_1, LOW);
  }
}

//==============================================================================
// FUNCTION
//==============================================================================
static void SetOutput_Light_ShutterOpen(DigitaInputlOuptut_State_t state)
{
  if (state == ON)
  {
    digitalWrite(Q0_2, HIGH);
  }
  else
  {
    digitalWrite(Q0_2, LOW);
  }
}

//==============================================================================
// FUNCTION
//==============================================================================
static void SetOutput_Light_ShutterClose(DigitaInputlOuptut_State_t state)
{
  if (state == ON)
  {
    digitalWrite(Q0_3, HIGH);
  }
  else
  {
    digitalWrite(Q0_3, LOW);
  }
}


//==============================================================================
// FUNCTION
//==============================================================================
static DigitaInputlOuptut_State_t GetShutterOpenSensorState(void)
{
  if (digitalRead(I0_3) == HIGH)
  {
    return ON;
  }
  else
  {
    return OFF;
  }
}

//==============================================================================
// FUNCTION
//==============================================================================
static DigitaInputlOuptut_State_t GetShutterCloseSensorState(void)
{
  if (digitalRead(I0_2) == HIGH)
  {
    return ON;
  }
  else
  {
    return OFF;
  }
}

//==============================================================================
// FUNCTION
//==============================================================================
static DigitaInputlOuptut_State_t ShutterClose_BtnPressed(void)
{
  if (digitalRead(I0_1) == HIGH)
  {
    return ON;
  }
  else
  {
    return OFF;
  }
}

//==============================================================================
// FUNCTION
//==============================================================================
static DigitaInputlOuptut_State_t ShutterOpen_BtnPressed(void)
{
  if (digitalRead(I0_0) == HIGH)
  {
    return ON;
  }
  else
  {
    return OFF;
  }
}

//==============================================================================
// FUNCTION
//==============================================================================
static void MCtrl_StateInidication(void)
{
  // Blink command
  typedef enum
  {
    NoBlink,
    OutputNormalBlink,
    OutputFastBlink
  } Output_Light_Shutter_Blink_Cmd_t;

  // Status for debug
  static Output_Light_Shutter_Blink_Cmd_t Output_Light_ShutterClose_Blink_Cmd = NoBlink;
  static Output_Light_Shutter_Blink_Cmd_t Output_Light_ShutterOpen_Blink_Cmd = NoBlink;

  // Blinking SM
  typedef enum
  {
    Blink_Idle,
    Blink_ON,
    Blink_OFF,
    Blink_Delay_ON,
    Blink_Delay_OFF
  } Shutter_Blink_SM_t;

  // Two identical state machines - one to drive the output for Ligh signalisation for closing
  // and one to drive the light signalisation for closing
  static Shutter_Blink_SM_t ShutterClose_Blink_SM = Idle;
  static Shutter_Blink_SM_t ShutterOpen_Blink_SM = Idle;

  // State value used for dbug
  ShutterCloseLightOutput_DbgVal = 0u;
  ShutterOpenLightOutput_DbgVal = 0u;

  // Timer to implement blinking periods
  bsw_DecCnt_Word(&ShutterClose_Blink_Tmr);
  bsw_DecCnt_Word(&ShutterOpen_Blink_Tmr);

  // If a button is pressed stop timers - for sincronisation in case of error
  if (ev_ShutterOPEN_Btn_Pressed_Signaling || ev_ShutterCLOSE_Btn_Pressed_Signaling)
  {
    STOP_COUNT_W(ShutterClose_Blink_Tmr);
    STOP_COUNT_W(ShutterOpen_Blink_Tmr);

    // Stop blinking - command
    Output_Light_ShutterClose_Blink_Cmd = NoBlink;
    Output_Light_ShutterOpen_Blink_Cmd = NoBlink;

    // reset blink state machine
    ShutterClose_Blink_SM = Idle;
    ShutterOpen_Blink_SM = Idle;

    // Switch off the outputs
    SetOutput_Light_ShutterClose(OFF);
    SetOutput_Light_ShutterOpen(OFF);
  }

  // ===============================================================================================================
  // (1) State transitions - Output Shuter CLOSE/ERROR light control
  // ===============================================================================================================  
  // No error AND sensor confirmed AND motor not moving
  if ( (ErrorRegister1 == NoErr) && (GetShutterCloseSensorState() == ON) && (VentDuctShutterSM == Idle) )
  {
    SetOutput_Light_ShutterClose(ON);
    Output_Light_ShutterClose_Blink_Cmd = NoBlink;
    ShutterCloseLightOutput_DbgVal = 1u;
  }
  // No error AND sensor not confirmed AND motor not moving
  else if ( (ErrorRegister1 == NoErr) && (GetShutterCloseSensorState() == OFF) && (VentDuctShutterSM == Idle) )
  {
    SetOutput_Light_ShutterClose(OFF);
    Output_Light_ShutterClose_Blink_Cmd = NoBlink;
    ShutterCloseLightOutput_DbgVal = 2u;
  }
  else if ( (ErrorRegister1 == NoErr) && (VentDuctShutterSM == Closing) )
  {
    Output_Light_ShutterClose_Blink_Cmd = OutputNormalBlink;
  }
  // Error
  else if (ErrorRegister1 != NoErr)
  {
    Output_Light_ShutterClose_Blink_Cmd = OutputFastBlink;
  }
  else
  {
    // Nothing to do
  }

  // In case of error and during motor movement output 
  // Output blink State machine
  switch(ShutterClose_Blink_SM)
  {
    case Blink_Idle:
      if (Output_Light_ShutterClose_Blink_Cmd == OutputNormalBlink)
      {
        // Load timer for normal blink
        // and change state to blink the output - motor moving bliking
        ShutterClose_Blink_Duration = TIME_BLINK_ON_MOT;
        bsw_LoadCnt_Word(&ShutterClose_Blink_Tmr, ShutterClose_Blink_Duration);
        ShutterClose_Blink_SM = Blink_ON;
      }
      else if (Output_Light_ShutterClose_Blink_Cmd == OutputFastBlink)
      {
        // Load timer for fast blink
        // and change state to blink the output - error blinking
        ShutterClose_Blink_Duration = TIME_BLINK_ON_ERR;
        bsw_LoadCnt_Word(&ShutterClose_Blink_Tmr, ShutterClose_Blink_Duration);
        ShutterClose_Blink_SM = Blink_ON;
      }
      else
      {
        // Nothing to do - wait in this state
      }
    break;

    case Blink_ON:
      SetOutput_Light_ShutterClose(ON);
      ShutterClose_Blink_SM = Blink_Delay_ON;
    break;

    case Blink_Delay_ON:
      ShutterCloseLightOutput_DbgVal = 3u;

      // Wait to expire timer
      if (bsw_GetCnt_Word(&ShutterClose_Blink_Tmr) == W_TIMER_OFF)
      {
        bsw_LoadCnt_Word(&ShutterClose_Blink_Tmr, ShutterClose_Blink_Duration);
        ShutterClose_Blink_SM = Blink_OFF;
      }

      // Quit blinking if any button is pressed 
      if (ev_ShutterOPEN_Btn_Pressed_Signaling || ev_ShutterCLOSE_Btn_Pressed_Signaling)
      {
        ShutterClose_Blink_SM = Blink_Idle;
        STOP_COUNT_W(ShutterClose_Blink_Tmr);
      }
    break;

    case Blink_OFF:
      SetOutput_Light_ShutterClose(OFF);
      ShutterClose_Blink_SM = Blink_Delay_OFF;
    break;

    case Blink_Delay_OFF:
      ShutterCloseLightOutput_DbgVal = 4u;

      // Wait to expire timer
      if (bsw_GetCnt_Word(&ShutterClose_Blink_Tmr) == W_TIMER_OFF)
      {
        bsw_LoadCnt_Word(&ShutterClose_Blink_Tmr, ShutterClose_Blink_Duration);
        ShutterClose_Blink_SM = Blink_Idle;
      }

      // Quit blinking if any button is pressed 
      if (ev_ShutterOPEN_Btn_Pressed_Signaling || ev_ShutterCLOSE_Btn_Pressed_Signaling)
      {
        ShutterClose_Blink_SM = Blink_Idle;
        STOP_COUNT_W(ShutterClose_Blink_Tmr);
      }
    break;

    default:
      ShutterClose_Blink_SM = Blink_Idle;
    break;
  }  // End of ShutterClose_Blink_SM


  // ===============================================================================================================
  // (2) State transitions - Output Shuter OPEN/ERROR light control
  // ===============================================================================================================
  // No error AND sensor confirmed AND motor not moving
  if ( (ErrorRegister1 == NoErr) && (GetShutterOpenSensorState() == ON) && (VentDuctShutterSM == Idle) )
  {
    SetOutput_Light_ShutterOpen(ON);
    Output_Light_ShutterOpen_Blink_Cmd = NoBlink;
    ShutterOpenLightOutput_DbgVal = 1u;
  }
  // No error AND sensor not confirmed AND motor not moving
  else if ( (ErrorRegister1 == NoErr) && (GetShutterOpenSensorState() == OFF) && (VentDuctShutterSM == Idle) )
  {
    SetOutput_Light_ShutterOpen(OFF);
    Output_Light_ShutterOpen_Blink_Cmd = NoBlink;
    ShutterOpenLightOutput_DbgVal = 2u;
  }
  else if ( (ErrorRegister1 == NoErr) && (VentDuctShutterSM == Opening) )
  {
      Output_Light_ShutterOpen_Blink_Cmd = OutputNormalBlink;
  }
  // Error
  else if (ErrorRegister1 != NoErr)
  {
    Output_Light_ShutterOpen_Blink_Cmd = OutputFastBlink;
  }
  else
  {
    // Nothing to do
  }

  // In case of error and during motor movement output 
  // Output blink State machine
  switch(ShutterOpen_Blink_SM)
  {
    case Blink_Idle:
      if (Output_Light_ShutterOpen_Blink_Cmd == OutputNormalBlink)
      {
        // Load timer for normal blink
        // and change state to blink the output - motor moving bliking
        ShutterOpen_Blink_Duration = TIME_BLINK_ON_MOT;
        bsw_LoadCnt_Word(&ShutterOpen_Blink_Tmr, ShutterOpen_Blink_Duration);
        ShutterOpen_Blink_SM = Blink_ON;
      }
      else if (Output_Light_ShutterOpen_Blink_Cmd == OutputFastBlink)
      {
        // Load timer for fast blink
        // and change state to blink the output - error blinking
        ShutterOpen_Blink_Duration = TIME_BLINK_ON_ERR;
        bsw_LoadCnt_Word(&ShutterOpen_Blink_Tmr, ShutterOpen_Blink_Duration);
        ShutterOpen_Blink_SM = Blink_ON;
      }
      else
      {
        // Nothing to do - wait in this state
      }
    break;

    case Blink_ON:
      SetOutput_Light_ShutterOpen(ON);
      ShutterOpen_Blink_SM = Blink_Delay_ON;
    break;

    case Blink_Delay_ON:
      ShutterOpenLightOutput_DbgVal = 3u;

      // Wait to expire timer
      if (bsw_GetCnt_Word(&ShutterOpen_Blink_Tmr) == W_TIMER_OFF)
      {
        bsw_LoadCnt_Word(&ShutterOpen_Blink_Tmr, ShutterOpen_Blink_Duration);
        ShutterOpen_Blink_SM = Blink_OFF;
      }

      // Quit blinking if any button is pressed 
      if (ev_ShutterOPEN_Btn_Pressed_Signaling || ev_ShutterCLOSE_Btn_Pressed_Signaling)
      {
        ShutterClose_Blink_SM = Blink_Idle;
        STOP_COUNT_W(ShutterOpen_Blink_Tmr);
      }
    break;

    case Blink_OFF:
      SetOutput_Light_ShutterOpen(OFF);
      ShutterOpen_Blink_SM = Blink_Delay_OFF;
    break;

    case Blink_Delay_OFF:
      ShutterOpenLightOutput_DbgVal = 4u;

      // Wait to expire timer
      if (bsw_GetCnt_Word(&ShutterOpen_Blink_Tmr) == W_TIMER_OFF)
      {
        bsw_LoadCnt_Word(&ShutterOpen_Blink_Tmr, ShutterOpen_Blink_Duration);
        ShutterOpen_Blink_SM = Blink_Idle;
      }

      // Quit blinking if any button is pressed 
      if (ev_ShutterOPEN_Btn_Pressed_Signaling || ev_ShutterCLOSE_Btn_Pressed_Signaling)
      {
        ShutterClose_Blink_SM = Blink_Idle;
        STOP_COUNT_W(ShutterOpen_Blink_Tmr);
      }
    break;

    default:
      ShutterOpen_Blink_SM = Blink_Idle;
    break;
  } // End of ShutterOpen_Blink_SM

  ev_ShutterOPEN_Btn_Pressed_Signaling  = false;
  ev_ShutterCLOSE_Btn_Pressed_Signaling = false;
}

//==============================================================================
// FUNCTION
//==============================================================================
static void ErrClearAllErrors(error_t * const errReg)
{
  *errReg = NoErr;
}

//==============================================================================
// FUNCTION
//==============================================================================
static void ErrSetError(error_t * const errReg, error_t err)
{
  *errReg = (*errReg) | err;
}

//==============================================================================
// FUNCTION
//==============================================================================
// static void ErrClrError(error_t * const errReg, error_t err)
// {
//   *errReg = (*errReg) & (~err);
// }

//==============================================================================
// FUNCTION
//==============================================================================
// static bool ErrGetState(error_t * const errReg, error_t err)
// {
//   if ((*errReg) | err)
//   {
//     return true;
//   }
//   else
//   {
//     return false;
//   }
// }

