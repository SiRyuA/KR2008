#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <math.h>
 
#ifndef _AVR_SYSTEM__H_
#define _AVR_SYSTEM__H_
 
#define F_CPU 7372800
 
#define BV_(x) _BV(x)
#define sbi_(port,bit) port |= _BV(bit)
#define cbi_(port,bit) port &= (~_BV(bit))
 
#define __hc165_clk() sbi_(PORTB,1);asm volatile("nop"::);cbi_(PORTB,1);asm volatile("nop"::)
#define __hc165_data_load() cbi_(PORTC,7);asm volatile("nop"::);sbi_(PORTC,7)
 
#define SW0   (PINC&0x40)==0
#define SW1   (PING&0x01)==0
#define SW2   (PING&0x02)==0
#define SW3   (PING&0x04)==0
 
volatile unsigned char  Complement_Flag=0;
volatile unsigned char  IR_Operation_Flag=0;
volatile unsigned char  flag=0;
volatile unsigned char  flag_check=0;
volatile unsigned char  PSD_Move_Flag=0;
volatile unsigned char  AD_IR_Save[5]={0,0,0,0,0};
volatile unsigned char  IR_Digital[5] = {0,0,0,0,0};
 
volatile unsigned int  PSD_AD_Save[3]={0,0,0};
volatile unsigned int  PSD_Voltage[3] = {0,0,0};
volatile unsigned int  TovfCnt = 0;
 
volatile long    Encoder[3]={0,0,0};
 
unsigned char     lcd_maxcols;
unsigned char     lcd_maxrows;
 
unsigned int    PSDL[10]={0,0,0,0,0,0,0,0,0,0};
unsigned int    PSDC[10]={0,0,0,0,0,0,0,0,0,0};
unsigned int    PSDR[10]={0,0,0,0,0,0,0,0,0,0};
unsigned int    count=0,
       psd_flag=0;
unsigned int    Complement_PSDC=0;
unsigned int    Complement_PSDL=0;
unsigned int    Complement_PSDR=0;
 
unsigned int    IR_ADConverter=0;
unsigned int    Motor_Break=0;
unsigned int    Motor_Calibration=0;
 
float      Voltage=0,
       PSD_cm[3],
       PSD_CM[3],
       PSD_CMR[3];
 
void PSD(char channel);
void Read_Time();
void Shake();
 
long MCtrl_REncoder(unsigned char ch);
void MCtrl_PwmDuty(unsigned char ch, int pwm);
void omni(int x, int y, int r);
 
void uSecDelay(unsigned int uSec);
void SecDelay(unsigned int Sec);
void msDelay(unsigned int ms);
 
void lcd_write_command(unsigned char val);
void lcd_write_data(unsigned char val);
void lcd_init(unsigned char maxrows, unsigned char maxcols);
void lcd_move_cursor(unsigned char row, unsigned char col);
void lcd_display_char(unsigned char row, unsigned char col, int8_t ch);
void lcd_display_str(unsigned char row, unsigned char col, int8_t* str);
 
void Settings(int IR_Read_Time,int IR_ADConverter_Center,int Motor_Break_Type,int Motor_Calibration_Data);
 
void mcu_init(void);
 
void lcd_clear_screen(void);
void LCD_num(char x, char y, int num);
void LCD_sensor(char x, char y, int num);
void LCD_count(char x, char y, int num);
 
void Encoder_Reset(void);
void Start(void);
void Stop(void);
 
void PSD_IR_Check(void);
void IR_ADConverter_Save(int Delay, int Value);
 
void Move_XYR_Encoder(int X, int Y, int R, int encoder_value, int stop_cm);
void Move_XYR_IR(int X, int Y, int R, int search_type, int IR_num);
void Move_XYR_PSD(int X, int Y, int R, int search_type, int PSD_ch, int stop_distance);
void Move_XYR_Time(int X, int Y, int R, int stop_time);
 
void Move_Line(int X, int R, int stop_check, int LINE);
 
void Move_PSD_Encoder(int X, int PSD_ch, int Distance, int stop_cm);
void Move_PSD_IR(int X, int PSD_ch, int Distance);
void Move_PSD_PSD(int X, int PSD_ch, int Distance, int stop_Distance);
void Move_PSD_Time(int X, int PSD_ch, int Distance, int stop_time);
 
#endif
