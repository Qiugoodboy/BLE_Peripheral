/*
ϵͳ�ṩ�������¼�������¼��ĺ���
uint8 osal_set_event(uint8 task_id, uint16 event_flag );

uint8 osal_clear_event( uint8 task_id, uint16 event_flag );

������ʱ������һ��
uint8 osal_start_timerEx( uint8 taskID, uint16 event_id ,uint32 timeout_value );

������ʱ�����Զ���װ
uint8 osal_start_reload_timer( uint8 taskID, uint16 event_id, uint32 timeout_value );

ֹͣ��ʱ��
uint8  osal_stop_timerEx( uint8 task_id, uint16 event_id );

����ϵͳʱ��(һ�����ڶԱ���������ʷ��ʱ��)
uint32 osal_GetSystemClock( void );

led����˸
void HalLedBlink( uint8 leds, uint8 numBlinks, uint8 percent, uint16 period );

param:
leds : ��Ӧ��led
numBlinks: ��˸����
percent: ռ�ձ�
period: ����(ms)

����Ҫled1���1hz�ķ���:
HalLedBlink( HAL_LED_1, 100, 50, 1000);

Э��ջ������ֵ��ȡ����
bStatus_t GATT_ReadCharValue( uint16 connHandle, attReadReq_t *pReq, uint8 taskId );


Э��ջ������ֵд�뺯��
extern bStatus_t GATT_WriteCharValue( uint16 connHandle, attWriteReq_t *pReq, uint8 taskId );

��ȡ����ֵ������
//SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );

д����ֵ������
extern bStatus_t SimpleProfile_SetParameter( uint8 param, uint8 len, void *value );

osal_strlen osal_memcpy osal_memcmp�Ⱥ�����osal.c�ļ��϶���..
*/