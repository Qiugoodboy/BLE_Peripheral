#include "SimpleBLE_Func.h"
#include "hal_uart.h"
#include "OSAL_Timers.h"
#include "DHT11.h"
#include "osal.h"
#include<stdio.h>

uint8 sensorValue[4];
uint8 SimpleBLEFunc_TaskID;
void SimpleBLEFunc_Init( uint8 task_id )
{
  SimpleBLEFunc_TaskID = task_id;
}
static void performPeriodicTask_Red( void )
{
   P0DIR &= ~0x01;          //P0.0����Ϊ�����    
   P2INP |= 0x20;
   if(DATA_PIN == 1)
   {
     // DelayMS(10);     //��ʱ10������Ϊ�˲��� ʵ����Ҳû��ʲô�ã� ������ô���ȽϹ淶һЩ
     //  if( DATA_PIN == 1 )
     // {      
         HalLedSet( HAL_LED_3, HAL_LED_MODE_ON );    //����ʱLED1���� �ߵ�ƽ����
     // }
   }    	
    else
   {
       HalLedSet( HAL_LED_3, HAL_LED_MODE_OFF );          //����ʱLED1Ϩ�� �͵�ƽϨ��
   }
   osal_start_timerEx( SimpleBLEFunc_TaskID, FNC_RED_EVENT ,SBP_RED_EVT_PERIOD );
}

static void performPeriodicTask_MQ( void )
{
   
   HAL_DISABLE_INTERRUPTS(); //���ж�
    P0DIR &= ~0x01;           //P0.0����Ϊ�����
    if(DATA_PIN == 0)         //��Ũ�ȸ����趨ֵʱ ��ִ����������        
    {     
       DelayMS(5);          //��ʱ������
       if(DATA_PIN == 0)     //ȷ�� Ũ�ȸ����趨ֵʱ ��ִ����������
       {
          HalLedSet( HAL_LED_3, HAL_LED_MODE_ON );         
       }
    }
   else
    {
       HalLedSet( HAL_LED_3, HAL_LED_MODE_OFF );
    }
   HAL_ENABLE_INTERRUPTS();  //���ж�
    osal_start_timerEx( SimpleBLEFunc_TaskID, FNC_MQ_EVENT ,SBP_RED_EVT_PERIOD );
}
static void performPeriodicTask_DHT( void )
{
  #if defined( SENSOR_DHT11 )
   char TemppktBuffer[32];
   char HumpktBuffer[32];
   
    //uint8 returnBytes;
    {
        //uint16 numBytes;

        char pktBuffer[64];
        int reti = 1;

       

//        HAL_DISABLE_INTERRUPTS(); //���ж�
        reti = ReadValue(sensorValue);
//        HAL_ENABLE_INTERRUPTS();  //���ж�
        if(reti == 0)
        {
            sprintf((char *)pktBuffer, "Dht11:fail\r\n");
        }
        else
        {
            sprintf((char *)TemppktBuffer, "T=%d.%d%%\r\n", 
                sensorValue[2], sensorValue[3]);
            sprintf((char *)HumpktBuffer, "H=%d.%d%%\r\n", 
                sensorValue[0], sensorValue[1]);
                  
        }

        HalLcdWriteString(TemppktBuffer,  HAL_LCD_LINE_5 );
        HalLcdWriteString(TemppktBuffer,  HAL_LCD_LINE_5 );
        HalLcdWriteString(HumpktBuffer,  HAL_LCD_LINE_6 );
        
    }
#else
  uint8 valueToCopy;
  uint8 stat;
  
  // Call to retrieve the value of the third characteristic in the profile
  stat = SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &valueToCopy, &returnBytes);

  if( stat == SUCCESS )
  {
    /*
     * Call to set that value of the fourth characteristic in the profile. Note
     * that if notifications of the fourth characteristic have been enabled by
     * a GATT client device, then a notification will be sent every time this
     * function is called.
     */
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof(uint8), &valueToCopy);
  }
#endif  
  osal_start_timerEx( SimpleBLEFunc_TaskID, FNC_DHT_EVENT ,SBP_RED_EVT_PERIOD );
}

uint16 SimpleBLE_FuncProcessEvent(uint8 task_id , uint16 events)
{
    switch( events )
    {
    case FNC_DHT_EVENT:
      performPeriodicTask_DHT();
      return ( events ^ FNC_DHT_EVENT );
      break;
      
    case FNC_MQ_EVENT:
      performPeriodicTask_MQ();
      return ( events ^ FNC_MQ_EVENT );
      break;
      
    case FNC_RED_EVENT:
      performPeriodicTask_Red();
      return ( events ^ FNC_RED_EVENT );
      break;
      
    default:
      break;
    }
    return 0;
}




/****************************************************************************
* ��    ��: DelayMS()
* ��    ��: �Ժ���Ϊ��λ��ʱ 16MʱԼΪ535,ϵͳʱ�Ӳ��޸�Ĭ��Ϊ16M
* ��ڲ���: msec ��ʱ������ֵԽ����ʱԽ��
* ���ڲ���: ��
****************************************************************************/
void DelayMS(uint8 msec)
{ 
    int i,j;//uint8�����ܵ�535
    
    for (i=0; i<msec; i++)
        for (j=0; j<535; j++);
}
