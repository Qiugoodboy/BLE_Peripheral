#ifndef __OLED_SHOW_H_
#define __OLED_SHOW_H_


extern void LCD_P16x16Ch(unsigned char x, unsigned char y, unsigned char N);
/*欢迎界面*/
extern void table_show();

/*功能设定模块*/
extern void Str_show(); 

/*相应功能界面*/
extern void Func_show();

/*对应功能显示*/
extern void sign_show( uint8 i );

/*显示空白页*/
extern void blank_show();    
extern void DHT11_show();
extern void MQ2_show();
extern void RED_show();


#endif