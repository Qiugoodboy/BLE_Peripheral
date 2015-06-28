/*
系统提供的设置事件和清除事件的函数
uint8 osal_set_event(uint8 task_id, uint16 event_flag );

uint8 osal_clear_event( uint8 task_id, uint16 event_flag );

启动定时器，仅一次
uint8 osal_start_timerEx( uint8 taskID, uint16 event_id ,uint32 timeout_value );

启动定时器，自动重装
uint8 osal_start_reload_timer( uint8 taskID, uint16 event_id, uint32 timeout_value );

停止定时器
uint8  osal_stop_timerEx( uint8 task_id, uint16 event_id );

返回系统时钟(一般用于对比现在与历史的时间)
uint32 osal_GetSystemClock( void );

led灯闪烁
void HalLedBlink( uint8 leds, uint8 numBlinks, uint8 percent, uint16 period );

param:
leds : 对应的led
numBlinks: 闪烁次数
percent: 占空比
period: 周期(ms)

例如要led1输出1hz的方波:
HalLedBlink( HAL_LED_1, 100, 50, 1000);

协议栈的特征值读取函数
bStatus_t GATT_ReadCharValue( uint16 connHandle, attReadReq_t *pReq, uint8 taskId );


协议栈的特征值写入函数
extern bStatus_t GATT_WriteCharValue( uint16 connHandle, attWriteReq_t *pReq, uint8 taskId );

读取特征值的数据
//SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );

写特征值的数据
extern bStatus_t SimpleProfile_SetParameter( uint8 param, uint8 len, void *value );

osal_strlen osal_memcpy osal_memcmp等函数在osal.c文件上定义..
*/