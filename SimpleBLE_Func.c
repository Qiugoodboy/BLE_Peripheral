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
   P0DIR &= ~0x01;          //P0.0定义为输入口    
   P2INP |= 0x20;
   if(DATA_PIN == 1)
   {
     // DelayMS(10);     //延时10毫秒作为滤波， 实际上也没有什么用， 但是这么做比较规范一些
     //  if( DATA_PIN == 1 )
     // {      
         HalLedSet( HAL_LED_3, HAL_LED_MODE_ON );    //有人时LED1亮， 高电平点亮
     // }
   }    	
    else
   {
       HalLedSet( HAL_LED_3, HAL_LED_MODE_OFF );          //无人时LED1熄灭， 低电平熄灭
   }
   osal_start_timerEx( SimpleBLEFunc_TaskID, FNC_RED_EVENT ,SBP_RED_EVT_PERIOD );
}

static void performPeriodicTask_MQ( void )
{
   
   HAL_DISABLE_INTERRUPTS(); //关中断
    P0DIR &= ~0x01;           //P0.0定义为输入口
    if(DATA_PIN == 0)         //当浓度高于设定值时 ，执行条件函数        
    {     
       DelayMS(5);          //延时抗干扰
       if(DATA_PIN == 0)     //确定 浓度高于设定值时 ，执行条件函数
       {
          HalLedSet( HAL_LED_3, HAL_LED_MODE_ON );         
       }
    }
   else
    {
       HalLedSet( HAL_LED_3, HAL_LED_MODE_OFF );
    }
   HAL_ENABLE_INTERRUPTS();  //开中断
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

       

//        HAL_DISABLE_INTERRUPTS(); //关中断
        reti = ReadValue(sensorValue);
//        HAL_ENABLE_INTERRUPTS();  //开中断
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
* 名    称: DelayMS()
* 功    能: 以毫秒为单位延时 16M时约为535,系统时钟不修改默认为16M
* 入口参数: msec 延时参数，值越大，延时越久
* 出口参数: 无
****************************************************************************/
void DelayMS(uint8 msec)
{ 
    int i,j;//uint8不可能到535
    
    for (i=0; i<msec; i++)
        for (j=0; j<535; j++);
}
