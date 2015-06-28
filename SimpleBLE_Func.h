#ifndef _SIMPLEBLE_FUNC_H_
#define _SIMPLEBLE_FUNC_H_
#include "hal_types.h"
#include "hal_led.h"
#include "hal_lcd.h"
#define FNC_DHT_EVENT                             0x0001
#define FNC_MQ_EVENT                              0x0002 
#define FNC_RED_EVENT                             0x0004
#define DATA_PIN P0_0        //定义P0.0口为传感器的输入端
#define SBP_RED_EVT_PERIOD                  1000
#define SBP_RED_DHT_PERIOD                  500
#define SBP_RED_MQ_PERIOD                  500
#define SENSOR_DHT11
extern void DelayMS(uint8 msec);
extern void SimpleBLEFunc_Init( uint8 task_id );
extern uint16 SimpleBLE_FuncProcessEvent(uint8 task_id , uint16 events);
extern uint8 SimpleBLEFunc_TaskID;
#endif 