/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the Simple BLE Peripheral sample application
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "hal_uart.h"
#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "stdlib.h"
#include "gatt.h"
#include <string.h>
#include "hci.h"
#include <stdio.h>
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"
#include "SimpleBLE_Func.h"
#include "oled_show.h"
//#include <math.h>
#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "simpleBLEPeripheral.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */
extern uint8 sensorValue[4];
unsigned char uartbuf[128];
// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                  5000
#define SBP_DHT11_PERIOD                         200
// What is the advertising interval when device is discoverable (units of 625us, 160=100ms)
//从机广播间隔数值越大功率就越低，但广播的包的时间间隔就越大
#define DEFAULT_ADVERTISING_INTERVAL             160

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely

#if defined ( CC2540_MINIDK )
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_LIMITED
#else
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL
#endif  // defined ( CC2540_MINIDK )

 
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     20//   ????????????, ??????, ??????????????

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     20//  ????????????, ??????, ??????????????

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          300 //1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         3 //6

// Company Identifier: Texas Instruments Inc. (13)
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
//static void UART_CB(uint8 port ,uint8 event);
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing
static void simpleBLEPeripheral_HandleSerial(mtOSALSerialData_t *cmdMsg);
static gaprole_States_t gapProfileState = GAPROLE_INIT;  
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 FUNC_VALUE =0 ;
static uint8 scanRspData[] =
{
  // complete name
  0x14,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x53,   // 'S'
  0x69,   // 'i'
  0x6d,   // 'm'
  0x70,   // 'p'
  0x6c,   // 'l'
  0x65,   // 'e'
  0x42,   // 'B'
  0x4c,   // 'L'
  0x45,   // 'E'
  0x50,   // 'P'
  0x65,   // 'e'
  0x72,   // 'r'
  0x69,   // 'i'
  0x70,   // 'p'
  0x68,   // 'h'
  0x65,   // 'e'
  0x72,   // 'r'
  0x61,   // 'a'
  0x6c,   // 'l'

  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  //广播的功率，系统默认为0dbm
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( SIMPLEPROFILE_SERV_UUID ),
  HI_UINT16( SIMPLEPROFILE_SERV_UUID ),

};

enum flag{
    LCD_DHT11 = 1,
    LCD_MQ2 = 2,
    LCD_RED = 3
}LCD_FLAG;

enum table{
    LCD_TABLE1,
    LCD_TABLE2,
    LCD_TABLE3
}LCD_TABLE;

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "AmoMcu BLE Peripheral";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void performPeriodicTask( void );
static void simpleProfileChangeCB( uint8 paramID );
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys );


#if (defined HAL_LCD) && (HAL_LCD == TRUE)
static char *bdAddr2Str ( uint8 *pAddr );
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)



/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{ 
  //uint8 char6value[5] = "aaaaa";
     HalUARTWrite( HAL_UART_PORT_0, "HELLO UART0\n", 12 );
  UartInit();//initial uart 0
  //SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, 5, "AAAAA" );
  
  // SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR6, &char6value );
 // HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
 // HalUARTWrite( HAL_UART_PORT_0, char6value, 5 );
  //HalUARTWrite( HAL_UART_PORT_0, "LCD_MQ2" ,9);
  table_show();
  //Func_show(); 
  HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);
  simpleBLEPeripheral_TaskID = task_id;
  RegisterForSerial( simpleBLEPeripheral_TaskID );
  RegisterForKeys( simpleBLEPeripheral_TaskID );
  // Setup the GAP
  VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
  // Setup the GAP Peripheral Role Profile
  {
    #if defined( CC2540_MINIDK )
      // For the CC2540DK-MINI keyfob, device doesn't start advertising until button is pressed
      uint8 initial_advertising_enable = FALSE;
    #else
      // For other hardware platforms, device starts advertising upon initialization
      uint8 initial_advertising_enable = TRUE;
    #endif

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;

    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    //广播使能
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile
#if defined FEATURE_OAD
  VOID OADTarget_AddService();                    // OAD Profile
#endif

  // Setup the SimpleProfile Characteristic Values
  {
  //  uint8 charValue1 = 1;
    uint8 charValue2 = 2;
    uint8 charValue3 = 3;
    uint8 charValue4 = 4;
    uint8 charValue5[SIMPLEPROFILE_CHAR5_LEN] = { 1, 2, 3, 4, 5 };
  //  uint8 charValue6[SIMPLEPROFILE_CHAR6_LEN] = {1,2,3,4,5};
    uint8 charValue7[SIMPLEPROFILE_CHAR7_LEN] = {1,2,3,4,5};
  //  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, sizeof ( uint8 ), &charValue1 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint8 ), &charValue2 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint8 ), &charValue3 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &charValue4 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN, charValue5 );
  //  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, SIMPLEPROFILE_CHAR6_LEN, charValue6 );
    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR7, SIMPLEPROFILE_CHAR7_LEN, charValue7 );
  }


#if defined( CC2540_MINIDK )

  SK_AddService( GATT_ALL_SERVICES ); // Simple Keys Profile

  // Register for all key events - This app will handle all key events
  

  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );

  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output

  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low

#endif // #if defined( CC2540_MINIDK )

#if (defined HAL_LCD) && (HAL_LCD == TRUE)

#if defined FEATURE_OAD
  #if defined (HAL_IMAGE_A)
    HalLcdWriteStringValue( "BLE Peri-A", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #else
    HalLcdWriteStringValue( "BLE Peri-B", OAD_VER_NUM( _imgHdr.ver ), 16, HAL_LCD_LINE_1 );
  #endif // HAL_IMAGE_A
#else
  //HalLcdWriteString( "BLE Peripheral", HAL_LCD_LINE_1 );
    HalUARTWrite(HAL_UART_PORT_0,"BLE Peripheral",14);
#endif // FEATURE_OAD

#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

  // Register callback with SimpleGATTprofile
  VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );

#if defined ( DC_DC_P0_7 )

  // Enable stack to toggle bypass control on TPS62730 (DC/DC converter)
  HCI_EXT_MapPmIoPortCmd( HCI_EXT_PM_IO_PORT_P0, HCI_EXT_PM_IO_PORT_PIN7 );

#endif // defined ( DC_DC_P0_7 )

  // Setup a delayed profile startup
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );
  osal_set_event( simpleBLEPeripheral_TaskID, SBP_DHT11_EVT );
  //HalLedSet( HAL_LED_2, HAL_LED_MODE_ON );
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
    {
      simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & SBP_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

    // Start Bond Manager
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

    // Set timer for first periodic event
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_DHT11_EVT, SBP_DHT11_PERIOD );
    
    return ( events ^ SBP_START_DEVICE_EVT );
  }

  if ( events & SBP_PERIODIC_EVT )
  {
//    HalLedSet((HAL_LED_1),HAL_LED_MODE_ON);
//    {
//      uint16 adc;
//      HalAdcSetReference(HAL_ADC_REF_AVDD);
//      adc=HalAdcRead(HAL_ADC_CHN_AIN7,HAL_ADC_RESOLUTION_12);
//      HalLcdWriteStringValue("ADC",adc,10,HAL_LCD_LINE_6);
//      {
//        char str[32]="hello";
//        float volt=adc*3.3/2048;
//        sprintf(str,"volt=%.2fv",volt);
//        HalLcdWriteString(str,HAL_LCD_LINE_5);
//      }
//    }
    // Restart timer
    if ( SBP_PERIODIC_EVT_PERIOD )
    {
      osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );
    }

    // Perform periodic application task
    performPeriodicTask();

    return (events ^ SBP_PERIODIC_EVT);
  }
  if(events & SBP_DHT11_EVT)
  {
       // memset(temp,0,2);
       // memset(humidity,0,2);
        //将温湿度的转换成字符串 
        //获得的温湿度通过串口输出到电脑显示
        //HalUARTWrite(0,"strTemp",12);

        //HalUARTWrite(0,strHumidity,9); 


    return (events ^ SBP_DHT11_EVT);
  }

#if defined ( PLUS_BROADCASTER )
  if ( events & SBP_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ SBP_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER

  // Discard unknown events
  return 0;
}
static void simpleBLEPeripheral_HandleSerial(mtOSALSerialData_t *cmdMsg)
  {
    uint8 i,len,*str=NULL;   //len 有用数据长度
    str=cmdMsg->msg;  //指向数据开头
    len=*str;  //msg 里的第 1 个字节代表后面的数据长度
    for(i=1;i<=len;i++)
    
    HalUARTWrite(0,str+i,1); 
 }
/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 *********************************************************************/


static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
 // #if defined( CC2540_MINIDK )
    case KEY_CHANGE:
      simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;
 // #endif // #if defined( CC2540_MINIDK )
    case SERIAL_MSG:
    simpleBLEPeripheral_HandleSerial((mtOSALSerialData_t *)pMsg);
      break; 

  default:
    // do nothing
    break;
  }
}

//#if defined( CC2540_MINIDK )
/*********************************************************************
 * @fn      simpleBLEPeripheral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void simpleBLEPeripheral_HandleKeys( uint8 shift, uint8 keys )
{
  uint8 HAL_Keys = 0;

  VOID shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_UP )
  {
    if( FUNC_VALUE == 0 )
    {
    if( LCD_TABLE == LCD_TABLE2 )
    {
    HAL_Keys |= HAL_KEY_UP;
    if( LCD_FLAG == LCD_DHT11 )
    {
      sign_show( LCD_RED );
    }
    else
    {
      sign_show( LCD_FLAG-1 );
    }
    }
    }
   // osal_set_event(SimpleBLEFunc_TaskID,FNC_LCD_EVENT);

  }

  if ( keys & HAL_KEY_LEFT )
  {

    HAL_Keys |= HAL_KEY_LEFT;
    // if device is not in a connection, pressing the right key should toggle
    // advertising on and off
    if( gapProfileState != GAPROLE_CONNECTED )
    {
      uint8 current_adv_enabled_status;
      uint8 new_adv_enabled_status;

      //Find the current GAP advertisement status
      GAPRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );

      if( current_adv_enabled_status == FALSE )
      {
        new_adv_enabled_status = TRUE;
      }
      else
      {
        new_adv_enabled_status = FALSE;
      }

      //change the GAP advertisement status to opposite of current status
      GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
    }

  }
  
  if( keys & HAL_KEY_RIGHT )
  {
    HAL_Keys |= HAL_KEY_RIGHT;
   
    if( FUNC_VALUE == 0 )
    {
    if( LCD_TABLE == LCD_TABLE1 )
    {
      Str_show();
    }
    else if( LCD_TABLE == LCD_TABLE2 )
    {
    //  HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);
      blank_show();
      Func_show();
      switch( LCD_FLAG )
      {
      case LCD_DHT11 :
        DHT11_show();
        FUNC_VALUE = LCD_DHT11;
//        uint16 HalUARTWrite(uint8 port, uint8 *buf, uint16 len)
        HalUARTWrite( HAL_UART_PORT_0, "LCD_DHT11\n" ,9);
        osal_set_event( SimpleBLEFunc_TaskID, FNC_DHT_EVENT );
        break;
        
      case LCD_MQ2 :
        MQ2_show();
        FUNC_VALUE = LCD_MQ2;
        HalUARTWrite( HAL_UART_PORT_0, "LCD_MQ\n" ,8);
        osal_set_event( SimpleBLEFunc_TaskID, FNC_MQ_EVENT );
        break;
        
      case LCD_RED :
        FUNC_VALUE = LCD_RED;
        RED_show();
        HalUARTWrite( HAL_UART_PORT_0, "LCD_RED\n" ,8);
        osal_set_event( SimpleBLEFunc_TaskID, FNC_RED_EVENT );
        break;
        
      default:
        break;
      }
    }
  }
  }
  if( keys & HAL_KEY_DOWN )
  {
    HAL_Keys |= HAL_KEY_DOWN;
   
    if( FUNC_VALUE == 0 )
    {
    if( LCD_TABLE == LCD_TABLE2 )
    {
    if( LCD_FLAG == LCD_RED )
    {
      sign_show( LCD_DHT11 );
    }
    else
    {
      sign_show( LCD_FLAG+1 );
    }
  }
  }
  }
  
  if( keys & HAL_KEY_CENTER )
  {
    HAL_Keys |= HAL_KEY_CENTER;

   
    if ( gapProfileState == GAPROLE_CONNECTED )// 已连接上
    {
        char str[32] = {0};
        uint32 time = osal_GetSystemClock();
        
        static attHandleValueNoti_t pReport;
        
        sprintf(str, "time=%06ld", time);
        
        HalLcdWriteString( "Notify:", HAL_LCD_LINE_7 );
        HalLcdWriteString( str, HAL_LCD_LINE_8 );

        pReport.len = osal_strlen(str);
        pReport.handle = 0x0035;        
        osal_memcpy(pReport.value, str, pReport.len);
    
        GATT_Notification( 0, &pReport, FALSE );      

        HalLedBlink (HAL_LED_1 | HAL_LED_2 | HAL_LED_3, 1, 50, 100);//这个的意思是， 100ms内，以50%的占空比闪烁1次, 实际就是点亮50ms 
        //HalLedSet( HAL_LED_3, HAL_LED_MODE_ON );
         osal_set_event( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT );
    }
  }
  
      if ( keys & HAL_KEY_SW_6 )  // Switch 6
    {
  
    }
  // Set the value of the keys state to the Simple Keys Profile;
  // This will send out a notification of the keys state if enabled
  //SK_SetParameter( HAL_KEY_ATTR, sizeof ( uint8 ), &SK_Keys );
}
//#endif // #if defined( CC2540_MINIDK )

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {
        uint8 ownAddress[B_ADDR_LEN];
        uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          // Display device address
         // HalLcdWriteString( bdAddr2Str( ownAddress ),  HAL_LCD_LINE_2 );
         // HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
         // HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLedBlink (HAL_LED_4, 0, 10, 100);//这个的意思是， 以10%的占空比连续闪烁        
      }
      break;

    case GAPROLE_CONNECTED:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
         // HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_WAITING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
      //    HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        //  HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
       //   HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

    default:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
      //    HalLcdWriteString( "",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;

  }

  gapProfileState = newState;

#if !defined( CC2540_MINIDK )
  VOID gapProfileState;     // added to prevent compiler warning with
                            // "CC2540 Slave" configurations
#endif


}

/*********************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
uint8 char7value[15];
uint8 buf[6];
uint8 charvalue1;
static void performPeriodicTask( void )
{
//  uint8 valueToCopy;
//  uint8 stat;
//  char7value[0]++;
//  if(char7value[0]==200)
//  {
//   char7value[0]=0;
//  }
//  char7value[0]=(uint8)rand()/100;
//   SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR6, &buf );
//    HalUARTWrite( HAL_UART_PORT_0, buf, sizeof(buf) );
//  SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &charvalue1 );
//    HalUARTWrite( HAL_UART_PORT_0, &charvalue1, 1 );
//   // Call to retrieve the value of the third characteristic in the profile
//   // SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, 1, "A" );
//   // SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR6, 5, "AAAAA" );
//   //SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR6, &valueToCopy);
//  stat = SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &valueToCopy);
//  SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR7, sizeof(uint8), &char7value);
//  HalUARTWrite( HAL_UART_PORT_0 ,buf, 5 );
//  if( stat == SUCCESS )
//  {
//    /*
//     * Call to set that value of the fourth characteristic in the profile. Note
//     * that if notifications of the fourth characteristic have been enabled by
//     * a GATT client device, then a notification will be sent every time this
//     * function is called.
//     */
//    SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof(uint8), &valueToCopy);
//    
//  }
    //CHAR6通知功能
    if ( gapProfileState == GAPROLE_CONNECTED )// 已连接上
    {
        char str[32] = {0};
        uint32 time = osal_GetSystemClock();
        
        static attHandleValueNoti_t pReport;
        
//        sprintf(str, "time=%06ld", time);
        sprintf((char *)str, "T=%d.%d%%\r\n H=%d.%d%%\r\n", 
        sensorValue[2], sensorValue[3], 
                sensorValue[0], sensorValue[1]);
        HalLcdWriteString( "Notify:", HAL_LCD_LINE_7 );
        HalLcdWriteString( str, HAL_LCD_LINE_8 );

        pReport.len = osal_strlen(str);
        pReport.handle = 0x0035;        
        osal_memcpy(pReport.value, str, pReport.len);
    
        GATT_Notification( 0, &pReport, FALSE );      

        HalLedBlink (HAL_LED_1 | HAL_LED_2 | HAL_LED_3, 1, 50, 100);//这个的意思是， 100ms内，以50%的占空比闪烁1次, 实际就是点亮50ms 
        //HalLedSet( HAL_LED_3, HAL_LED_MODE_ON );
    }
}
/*********************************************************************
* @fn     UART_CB
*
* @brief  Callback from Uart
*
* @param  channel and halUARTCfg_t
*
* @return none
*********************************************************************/
//static void UART_CB(uint8 port ,uint8 event)
//{
//    HalUARTRead(0,uartbuf,sizeof(uartbuf));
//      HalUARTWrite(0,uartbuf,9);
//      osal_memset(uartbuf,0,10);
//   
//    uint8 rxbuf[100],txbuf[100];
//    uint16 NumByte,flag;
//    NumByte=Hal_UART_RxBufLen(0);
//    HalUARTRead(0,rxbuf,NumByte);
//    if(osal_memcmp(rxbuf,"AT",2)==0)
//    {
//      flag=0;
//      osal_memset(rxbuf,0,sizeof(rxbuf));
//    }
//    if(osal_memcmp(rxbuf,"AT+LED1",7)==0)
//    {
//      flag=1;
//      osal_memset(rxbuf,0,sizeof(rxbuf));
//    }
//    switch(flag)
//    {
//      case 0:
//      strcpy(txbuf,"hello AT");
//      HalUARTWrite(0,txbuf,strlen(txbuf)+1);
//      osal_memset(txbuf,0,sizeof(txbuf));
//      break;
//      case 1:
//       HalLedSet(HAL_LED_3,HAL_LED_MODE_ON);
//      break;
//    default:
//      break;
//    }
 // HalUARTWrite(0,"Hello World\n",12);   
//}
/**********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 **********************************************************************/
//从机用于接收数据的函数
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue;
  uint8 buf[15];
  uint8 strbuf[15];
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR1:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, &newValue );
      
      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
       // HalLcdWriteStringValue( "Char 1:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      _ltoa(newValue,strbuf,10);
      HalUARTWrite(HAL_UART_PORT_0,strbuf,osal_strlen((char *)strbuf)+1);
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

    case SIMPLEPROFILE_CHAR3:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &newValue );

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
       // HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );

      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;

      #if (defined HAL_LCD) && (HAL_LCD == TRUE)
        //HalLcdWriteStringValue( "Char 3:", (uint16)(newValue), 10,  HAL_LCD_LINE_3 );
      #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

      break;
       case SIMPLEPROFILE_CHAR6:     
       SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR6, &buf );
        HalLedSet (HAL_LED_3,HAL_LED_MODE_ON);
     if(buf[0]>=15)
     {
      HalUARTWrite(0,&buf[1],14);
      HalUARTWrite(0,"...\n",4);
     }
     else
     {
      HalUARTWrite(0,&buf[1],buf[0]);
     }
      break;
    default:
      // should not reach here!
      break;
  }
}

#if (defined HAL_LCD) && (HAL_LCD == TRUE)
/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string. Only needed when
 *          LCD display is used.
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;

  *pStr++ = '0';
  *pStr++ = 'x';

  // Start from end of addr
  pAddr += B_ADDR_LEN;

  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }

  *pStr = 0;

  return str;
}
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)

/*********************************************************************
*********************************************************************/
