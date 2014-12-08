#include "avr_system.h"
 
void Settings(int IR_Read_Time,int IR_ADConverter_Center,int Motor_Break_Type,int Motor_Calibration_Data)
{
 mcu_init();
 IR_ADConverter_Save(IR_Read_Time,IR_ADConverter_Center);
 Motor_Break=Motor_Break_Type;
 Motor_Calibration=Motor_Calibration_Data;
}
 
 
void mcu_init(void)
{
 DDRA = 0xff;
 DDRB = (~BV_(0));
 DDRC = 0xbf;
 DDRD = 0xff;
 DDRE = 0xfe;
 DDRF = 0x00;
 DDRG = 0xf8;
 PORTA= 0x00;
 PORTB= (BV_(0));
 PORTC= 0x40;
 PORTD= 0x00;
 PORTE= 0X01;
 PORTF= 0x00;
 PORTG= 0x07;
 
 sbi_(PORTB,2);
 OCR1A = 0;
 sbi_(PORTB,3);
 OCR1B = 0;
 sbi_(PORTB,4);
 OCR1C = 0;
 MCtrl_REncoder(0);
 MCtrl_REncoder(1);
 MCtrl_REncoder(2);
 
 TCNT0 = 0x00;
 TCCR0 = 0x01;
 ASSR = 0;
 TIMSK |= _BV(TOIE0);
 TIMSK |= _BV(TOIE1);
 TCCR1B = _BV(CS10);
 TCCR1A = 0xA9;
 TCNT1 = 0x00;
 OCR1A = 0x00;
 OCR1B = 0x00;
 OCR1C = 0x00;
 TCNT2 = 0x00;
 TCCR2 = 0x00;
 TIMSK |= _BV(TOIE2);
 TCCR2 = 0x03;
 
 ADMUX = 0x00;
 ADCSRA = 0xce;
 
 sei();
 lcd_init(2, 20);
 lcd_clear_screen();
 
}
 
void lcd_clear_screen(void)
{
 uSecDelay(30);
 lcd_write_command(0x01);
 uSecDelay(2000);
}
 
void LCD_num(char x, char y, int num)
{
 unsigned char N1000,N100,N10,N1;
 N1000 = num/1000 + '0';
 N100 = (num%1000)/100 + '0';
 N10 = (num%100)/10 + '0';
 N1 = num%10 + '0';
 lcd_display_char(y,x,N100);
 lcd_display_char(y,x+1,N10);
 lcd_display_char(y,x+2,N1);
}
 
void LCD_sensor(char x, char y, int num)
{
 unsigned char N1000,N100,N10,N1;
 N1000 = num/1000 + '0';
 N100 = (num%1000)/100 + '0';
 N10 = (num%100)/10 + '0';
 N1 = num%10 + '0';
 lcd_display_char(y,x,N10);
 lcd_display_char(y,x+1,N1);
}
 
void LCD_count(char x, char y, int num)
{
 unsigned char N1000,N100,N10,N1;
 N1000 = num/1000 + '0';
 N100 = (num%1000)/100 + '0';
 N10 = (num%100)/10 + '0';
 N1 = num%10 + '0';
 lcd_display_char(y,x,N1);
}
 
void Encoder_Reset(void)
{
 if(Motor_Break==1) omni(0,0,0);
 MCtrl_REncoder(0); Encoder[0]=0;
 MCtrl_REncoder(1); Encoder[1]=0;
 MCtrl_REncoder(2); Encoder[2]=0;
}
 
void Start(void)
{
 lcd_clear_screen();
 PORTD=0x00;
 Encoder_Reset();
}
 
void Stop(void)
{
 lcd_clear_screen();
 PORTD=0xff;
 Motor_Break=1;
 Encoder_Reset();
}
 
void PSD_IR_Check(void)
{
 PSD(0);PSD(1);PSD(2);
 LCD_sensor(0,0,PSD_CMR[2]);
 LCD_sensor(3,0,PSD_CMR[1]);
 LCD_sensor(6,0,PSD_CMR[0]);
 LCD_count(0,1,IR_Digital[4]);
 LCD_count(2,1,IR_Digital[3]);
 LCD_count(4,1,IR_Digital[2]);
 LCD_count(6,1,IR_Digital[1]);
 LCD_count(8,1,IR_Digital[0]);
}
 
void IR_ADConverter_Save(int Delay, int Value)
{
 int BLine=0,i=0, Count=0, Out=0;
 int BFirst[5]={0,0,0,0,0},BSave[5]={255,255,255,255,255};
 for(;;)
 {
  PSD(2);LCD_sensor(1,1,PSD_CMR[2]);
  PSD(1);LCD_sensor(4,1,PSD_CMR[1]);
  PSD(0);LCD_sensor(7,1,PSD_CMR[0]);
  if(SW0)
  {
   lcd_clear_screen();
   for(;;)
   {
    BFirst[i] = AD_IR_Save[i];
    if(BSave[i]>BFirst[i]) BSave[i]=AD_IR_Save[i];
    
    omni(100,0,0);
    
    i++;
    if(i>4) i=0;
    
    Count++;
    if(Count>Delay) break;
   }
   Out=1;
  }
  else if(SW1)
  {
   lcd_clear_screen();
   for(char j=0;j<5;j++) BSave[j] = 25;
   msDelay(300);
   Out=1;
  }
  else if(SW3)
  {
   msDelay(300);
   IR_ADConverter=Value;
   Out=1;
  }
  if(Out==1) break;
 }
 BLine=(BSave[0]+BSave[1]+BSave[2]+BSave[3]+BSave[4])/5;
 IR_ADConverter=BLine+Value;
 lcd_clear_screen();
 omni(0,0,0);
}
