#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H


#if defined(__cplusplus)
extern "C"
{
#endif

void MCtrl_Init(void);
void MCtrl_Main(void);

uint32_t Get_VentDuctShutterSM_State(void);
uint32_t Get_MCtrl_ErrorReg(void);
uint32_t Get_MCtrl_StateInidication_DbgVal(void);
uint32_t Get_MCtrl_StateInidication_ShutterOpenLightOutput_DbgVal(void);
uint32_t Get_MCtrl_StateInidication_ShutterCloseLightOutput_DbgVal(void);


#if defined(__cplusplus)
}
#endif

#endif // MOTOR_CONTROLLER_H
