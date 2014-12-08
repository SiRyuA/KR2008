#include "avr_system.h"
// Sensor System
ISR(ADC_vect)
{
 static unsigned char Channel_cnt=0;
 unsigned char ADC_Low=0,ADC_High=0;
 unsigned char Change8=0;
 unsigned int AD_Val=0;
 Channel_cnt++;
 AD_Val=ADCW;
 if(Channel_cnt<6)
 {
  ADC_High=(unsigned char)((AD_Val>>8)&0xff);
  ADC_Low=(unsigned char)(AD_Val&0xff);
  Change8=((ADC_Low>>2)|(ADC_High<<6));
  AD_IR_Save[Channel_cnt-1]=Change8;
 }
 else
 {
  PSD_AD_Save[Channel_cnt-6]=AD_Val;
 }
 if(Channel_cnt>7)
 {
  Channel_cnt=0;
  Complement_Flag=1;
 }
 for(char i=0;i<5;i++)
 {
  if(AD_IR_Save[i]> IR_ADConverter) IR_Digital[i]=0;
  else IR_Digital[i]=1;
 }
 ADMUX = Channel_cnt;
 ADCSRA |= _BV(ADSC);
}
 
void PSD(char channel)
{
 count++;
 Voltage = (((float) PSD_Voltage[channel]*5)/1024);
 if(Voltage<2.4 && Voltage>0.4)
 {
  if(Voltage>0.94 && Voltage<2.4) PSD_cm[channel] = (9.9477*Voltage*Voltage) - (46.445*Voltage)+64.327;
  else if(Voltage>0.4 && Voltage<0.94) PSD_cm[channel] = (151.28*Voltage*Voltage) - (298.15*Voltage)+177.46;
 }
 else
 {
  if(Voltage>2.4) PSD_cm[channel]=10;
  else if(Voltage<0.4) PSD_cm[channel]=80;
 }
 PSD_CM[channel] += PSD_cm[channel];
 if(count >= 5) {
 PSD_CMR[channel] = (PSD_CM[channel]/5);
 count=0;
 PSD_CM[channel]=0;
 }
}
 
 
// Sensor Data Shake
void Read_Time()
{
 static unsigned char cnt=0;
 if(Complement_Flag==1)
 {
  Complement_Flag=0;
  cnt++;
  PSDL[cnt] = PSD_AD_Save[0];
  PSDC[cnt] = PSD_AD_Save[1];
  PSDR[cnt] = PSD_AD_Save[2];
  if(cnt==10)
  {
   Shake();
   Complement_PSDL=(PSDL[3]+PSDL[4]+PSDL[5]+PSDL[6]);
   Complement_PSDC=(PSDC[3]+PSDC[4]+PSDC[5]+PSDC[6]);
   Complement_PSDR=(PSDR[3]+PSDR[4]+PSDR[5]+PSDR[6]);
   Complement_PSDL=(Complement_PSDL>>2);
   Complement_PSDC=(Complement_PSDC>>2);
   Complement_PSDR=(Complement_PSDR>>2);
   cnt=0;
   PSD_Voltage[0]=Complement_PSDL;
   PSD_Voltage[1]=Complement_PSDC;
   PSD_Voltage[2]=Complement_PSDR;
  }
 }
}
 
void Shake()
{
 unsigned char i=0, j=0;
 unsigned int temp_L=0, temp_C=0, temp_R=0;
 for(i=0; i<5; i++)
 {
  for(j=i+1; j<10-i; j++)
  {
   if(PSDL[j-1] > PSDL[j])
   {
    temp_L = PSDL[j-1];
    PSDL[j-1] = PSDL[j];
    PSDL[j] = temp_L;
   }
   if(PSDC[j-1] > PSDC[j])
   {
    temp_C = PSDC[j-1];
    PSDC[j-1] = PSDC[j];
    PSDC[j] = temp_C;
   }
   if(PSDR[j-1] > PSDR[j])
   {
    temp_R = PSDR[j-1];
    PSDR[j-1] = PSDR[j];
    PSDR[j] = temp_R;
   }
  }
  for(j=10-i-2; j>i; j--)
  {
   if(PSDL[j-1] > PSDL[j])
   {
    temp_L = PSDL[j-1];
    PSDL[j-1] = PSDL[j];
    PSDL[j] = temp_L;
   }
   if(PSDC[j-1] > PSDC[j])
   {
    temp_C = PSDC[j-1];
    PSDC[j-1] = PSDC[j];
    PSDC[j] = temp_C;
   }
   if(PSDR[j-1] > PSDR[j])
   {
    temp_R = PSDR[j-1];
    PSDR[j-1] = PSDR[j];
    PSDR[j] = temp_R;
   }
  }
 }
}
 
 
// Motor System
long MCtrl_REncoder(unsigned char ch)
{
 unsigned char FCurCnt[3] = {0,0,0}, BCurCnt[3] = {0,0,0};
 static unsigned char FPreCnt[3] = {0,0,0}, BPreCnt[3] = {0,0,0};
 unsigned char fcnt = 0, bcnt = 0;
 unsigned char i = 0;
 if(ch == 0){
 cbi_(PORTC,5); cbi_(PORTC,4);
 }else if(ch == 1){
 cbi_(PORTC,5); sbi_(PORTC,4);
 }else if(ch == 2){
 sbi_(PORTC,5); cbi_(PORTC,4);
 }
 cbi_(PORTC,3);
 FCurCnt[ch] = 0;
 __hc165_data_load();
 for(i=0; i<8; i++){
 if(PINB& BV_(0)) FCurCnt[ch] |= (1<<(8-i));
 __hc165_clk();
 }
 sbi_(PORTC,3);
 BCurCnt[ch] = 0;
 __hc165_data_load();
 for(i=0; i<8; i++){
 if(PINB& BV_(0)) BCurCnt[ch] |= (1<<(8-i));
 __hc165_clk();
 }
 fcnt = FCurCnt[ch] - FPreCnt[ch];
 bcnt = BCurCnt[ch] - BPreCnt[ch];
 Encoder[ch] += (long)fcnt + (long)bcnt;
 FPreCnt[ch] = FCurCnt[ch];
 BPreCnt[ch] = BCurCnt[ch];
 return Encoder[ch];
}
 
void MCtrl_PwmDuty(unsigned char ch,int pwm)
{
 unsigned char dir=1, pwm_value=0;
 if(pwm == 0)
 {
  pwm_value = (int)(0);
 }
 else if(pwm > 0)
 {
  dir = 1;
  if(pwm > 255) pwm_value = 255;
  else pwm_value = (int)(pwm);
 }
 else if(pwm < 0)
 {
  dir = 0;
  if(pwm < (-255)) pwm_value = 255;
  else pwm_value = (int)(-pwm);
 }
 if(ch == 0){
  if(dir) sbi_(PORTB,2);
  else cbi_(PORTB,2);
  OCR1A = pwm_value;
 }else if(ch == 1){
  if(dir) sbi_(PORTB,3);
  else cbi_(PORTB,3);
  OCR1B = pwm_value;
 }else if(ch == 2){
  if(dir) sbi_(PORTB,4);
  else cbi_(PORTB,4);
  OCR1C = pwm_value;
 }
}
 
void omni(int x, int y, int r)
{
 MCtrl_PwmDuty(0, (-x*0.866) + (y*0.5) + r);
 MCtrl_PwmDuty(1, (-y + r));
 MCtrl_PwmDuty(2, ( x*0.866) + (y*0.5) + r);
}
 
 
// Timer & Encoder System
void uSecDelay(unsigned int uSec)
{
 unsigned int t = 0;
 t = (unsigned int)(uSec/ 16);
 TovfCnt = 0;
 while(!(TovfCnt>(t)));
}
 
void SecDelay(unsigned int Sec)
{
 unsigned int i=0, j=0;
 for(j=0; j<Sec; j++){
  for(i=0; i<20; i++){
   uSecDelay(50000);
  }
 }
}
 
void msDelay(unsigned int ms)
{
 unsigned int i=0, j=0;
 for(j=0; j<ms; j++){
  for(i=0; i<20; i++){
   uSecDelay(50);
  }
 }
}
 
ISR(TIMER2_OVF_vect)
{
 static unsigned char ir_cnt=0, cnt=0, cnt2=0, psd_move_cnt=0;
 TCNT2 = 5;
 psd_move_cnt++;
 ir_cnt++;
 Read_Time();
 if(psd_move_cnt==2)
 {
  psd_move_cnt=0;
  PSD_Move_Flag=1;
 }
 if(flag==1)
 {
  cnt++;
  if(cnt>=100)
  {
   cnt=0;
   cnt2++;
   if(cnt2>=flag_check)
   {
    cnt2=0;
    flag=0;
   }
  }
 }
 if(ir_cnt==5)
 {
  IR_Operation_Flag=1;
  ir_cnt=0;
 }
}
 
ISR(TIMER0_OVF_vect)
{
 TovfCnt++;
}
 
ISR(TIMER1_OVF_vect)
{
 asm volatile("nop"::);
}
 
 
// LCD System
void lcd_write_command(unsigned char val)
{
 asm volatile("nop"::);asm volatile("nop"::);
 PORTC &= ~_BV(0);
 PORTC &= ~(_BV(1));
 PORTC &= ~(_BV(2));
 asm volatile("nop"::);asm volatile("nop"::);
 PORTC |= _BV(0);
 PORTA = val;
 asm volatile("nop"::);asm volatile("nop"::);
 PORTC &= ~_BV(0);
 asm volatile("nop"::);asm volatile("nop"::);
 PORTC |= _BV(0);
 PORTC |= _BV(2);
 asm volatile("nop"::);asm volatile("nop"::);
}
 
void lcd_write_data(unsigned char val)
{
 asm volatile("nop"::);asm volatile("nop"::);
 PORTC &= ~_BV(0);
 PORTC |= _BV(1);
 PORTC &= ~(_BV(2));
 asm volatile("nop"::);asm volatile("nop"::);
 PORTC |= _BV(0);
 PORTA = val;
 asm volatile("nop"::);asm volatile("nop"::);
 PORTC &= ~(_BV(0));
 asm volatile("nop"::);asm volatile("nop"::);
 PORTC &= ~_BV(1);
 PORTC |= _BV(0);
 PORTC |= _BV(2);
 asm volatile("nop"::);asm volatile("nop"::);
}
 
void lcd_init(unsigned char maxrows, unsigned char maxcols)
{
 int i=0,j=0;
 
 lcd_maxrows = maxrows;
 lcd_maxcols = maxcols;
 uSecDelay(45000);
 lcd_write_command(0x38);
 uSecDelay(1000);
 lcd_write_command(0x0C);
 uSecDelay(1000);
 lcd_write_command(0x06);
 uSecDelay(1000);
 lcd_write_command(0x1c);
 
 for(i=20;i>0;i--)
 {
  lcd_display_str(i,j,"-★- KMC KR2008 -★-");
  msDelay(5);
  j++;
  if(j>1) j=0;
 }
  lcd_display_str(0,1," -VER.HGBSRA120525- ");
 msDelay(50);
 
 lcd_clear_screen();
 msDelay(50);
}
 
void lcd_move_cursor(unsigned char row, unsigned char col)
{
 uSecDelay(30);
 switch(row) {
 case 0: 
  lcd_write_command(0x80 + col);
  break;
 case 1:
  lcd_write_command((0xc0 + col));
  break;
 }
}
 
void lcd_display_char(unsigned char row, unsigned char col, int8_t ch)
{
 if(row < lcd_maxrows && col < lcd_maxcols) {
  uSecDelay(30);
  lcd_move_cursor(row, col);
  uSecDelay(30);
  lcd_write_data(ch);
  uSecDelay(30);
 }
}
 
void lcd_display_str(unsigned char row, unsigned char col, int8_t* str)
{
 unsigned char i;
 if(row < lcd_maxrows && col < lcd_maxcols) {
  lcd_move_cursor(row, col);
  i = col;
  while(i++ < lcd_maxcols && *str) {
   uSecDelay(30);
   lcd_write_data(*str++);
  }
 }
}