#include"DHT11.h"

/**************************************************
  接口定义，移植此程序只需修改下列宏定义和延时函数
**************************************************/
#define IN_DQ		P0_0  //设置数据端口
#define DQ_PIN		0     //这里需与IN_DQ一致
#define DQ_PORT		P0DIR


#define SET_OUT DQ_PORT|=BV(DQ_PIN);asm("NOP");asm("NOP")
#define SET_IN  DQ_PORT&=~(BV(DQ_PIN));asm("NOP");asm("NOP")

#define CL_DQ  IN_DQ=0;asm("NOP");asm("NOP")
#define SET_DQ IN_DQ=1;asm("NOP");asm("NOP") 


uint8  CheckSum;
uint8  tmp8BitValue;

/*函数声明*/
void Read8Bit(void);
static void Delay_nus(uint16 s);


static void Delay_nus(uint16 s) 
{
  while (s--)
  {
#if 0
    asm("NOP");
    asm("NOP");
    asm("NOP");
#else
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
	asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");asm("NOP");
#endif
  }
}

void Read8Bit(void)
{
  static uint8  OverTimeCnt = 0;
  uint8 i,tmpBit;
  
  for(i=0;i<8;i++)
  {
    OverTimeCnt = 2;
    while((IN_DQ == 0)&&OverTimeCnt++);
    //while(IN_DQ == 0);
    Delay_nus(19);//12
    if(IN_DQ == 1)
      tmpBit = 1;
    else
      tmpBit = 0;
    OverTimeCnt = 2;
    while((IN_DQ == 1)&&OverTimeCnt++);
    //while(IN_DQ == 1);
    //超时则跳出for循环		  
    if(OverTimeCnt==1)
      break;
    
    tmp8BitValue<<=1;
    tmp8BitValue|=tmpBit;        //0
  
  }

}

/*
//how to use ReadValue
uint8 Sensor[4];
ReadValue(Sensor);

Sensor[0]
-->湿度整数值
Sensor[1]
-->湿度小数值

Sensor[2]
-->温度整数值
Sensor[3]
-->温度小数值
*/
int ReadValue(uint8 *sv)
{
  static uint8  OverTimeCnt = 0;
  SET_OUT; 
  CL_DQ; 
  Delay_nus(20000);//主机拉低至少18ms  
  SET_DQ;
  Delay_nus(20);//总线由上拉电阻拉高 主机延时20us-40us
  SET_IN;
  if(IN_DQ == 0)
  {
    OverTimeCnt = 2;
    while((IN_DQ == 0)&&OverTimeCnt++);

    while(IN_DQ == 1);
    //数据接收状态	
    Read8Bit();
    sv[0]=tmp8BitValue;
    Read8Bit();
    sv[1]=tmp8BitValue;
    Read8Bit();
    sv[2]=tmp8BitValue;
    Read8Bit();
    sv[3]=tmp8BitValue;
	
    Read8Bit();
    CheckSum = tmp8BitValue;
    
    if(CheckSum == sv[0]+sv[1]+sv[2]+sv[3])
    {
      CheckSum = 0xff;
    
    }

    return 1;
  }

  return 0;
}

