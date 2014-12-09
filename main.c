#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <math.h>
 
#define F_CPU 7372800
 
#define BV_(x) _BV(x)
#define sbi_(port,bit) port |= _BV(bit)
#define cbi_(port,bit) port &= (~_BV(bit))
 
#define SW0   (PINC&0x40)==0        // Switch 0
#define SW1   (PING&0x01)==0        // Switch 1
#define SW2   (PING&0x02)==0        // Switch 2
#define SW3   (PING&0x04)==0        // Switch 3
 
volatile unsigned char  Complement_Flag=0;
volatile unsigned char  IR_Operation_Flag=0;
volatile unsigned char  flag=0;
volatile unsigned char  flag_check=0;
volatile unsigned char  PSD_Move_Flag=0;
volatile unsigned char  AD_IR_Save[5]={0,0,0,0,0};   // IR Sensor Analoge Data Save
volatile unsigned char  IR_Digital[5] = {0,0,0,0,0};  // IR Sensor Digital Data Save
 
volatile unsigned int  PSD_AD_Save[3]={0,0,0};   // PSD Sensor Analoge Data Save
volatile unsigned int  PSD_Voltage[3] = {0,0,0};   // PSD Sensor Voltage Data Save
volatile unsigned int  TovfCnt = 0;
 
volatile long    Encoder[3]={0,0,0};    // Motor Encoder Data Save
 
unsigned char     lcd_maxcols;      // LCD Width
unsigned char     lcd_maxrows;      // LCD height
 
unsigned int    PSDL[10]={0,0,0,0,0,0,0,0,0,0}; // PSD 0 Analoge Data 10EX Save 
unsigned int    PSDC[10]={0,0,0,0,0,0,0,0,0,0}; // PSD 1 Analoge Data 10EX Save 
unsigned int    PSDR[10]={0,0,0,0,0,0,0,0,0,0}; // PSD 2 Analoge Data 10EX Save 
unsigned int    count=0,
       psd_flag=0;
unsigned int    Complement_PSDC=0;     // PSD 0 Analoge Data Mean Save
unsigned int    Complement_PSDL=0;     // PSD 1 Analoge Data Mean Save
unsigned int    Complement_PSDR=0;     // PSD 2 Analoge Data Mean Save
 
unsigned int    IR_ADConverter=0;     // IR Sensor AD Converter Criticality Value
unsigned int    Motor_Break=0;      // Moving Motor Break?
unsigned int    Motor_Calibration=0;    // Moving Motor revise
 
float      Voltage=0,
       PSD_cm[3],
       PSD_CM[3],
       PSD_CMR[3];
 
/*------------------------------------------------------------------------
 Interrupt vector notice
 
 ADC_vect  :: SIG_ADC
 
 INT0_vect  :: SIG_INTERRUPT0
 INT1_vect  :: SIG_INTERRUPT1
 INT2_vect  :: SIG_INTERRUPT2
 INT3_vect  :: SIG_INTERRUPT3
 INT4_vect  :: SIG_INTERRUPT4
 INT5_vect  :: SIG_INTERRUPT5
 INT6_vect  :: SIG_INTERRUPT6
 INT7_vect  :: SIG_INTERRUPT7
 
 TIMER0_OVF_vect :: SIG_OVERFLOW0
 TIMER1_OVF_vect :: SIG_OVERFLOW1
 TIMER2_OVF_vect :: SIG_OVERFLOW2
 TIMER3_OVF_vect :: SIG_OVERFLOW3
 
 USART0TX_vect :: SIG_UART0_TRANS
 USART1TX_vect :: SIG_UART1_TRANS
 USART0RX_vect :: SIG_UART0_RECV
 USART1RX_vect :: SIG_UART1_RECV
 
  ------------------------------------------------------------------------*/
 
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
void lcd_display_char(unsigned char row, unsigned char col, signed char ch);
void lcd_display_str(unsigned char row, unsigned char col, signed char* str);
 
void Settings(int IR_Read_Time,int IR_ADConverter_Center,int Motor_Break_Type,int Motor_Calibration_Data);
 
void mcu_init(void);
 
void lcd_clear_line(unsigned char line);
void lcd_clear_screen(void);
void LCD_num_3(char x, char y, int num);
void LCD_num_2(char x, char y, int num);
void LCD_num_1(char x, char y, int num);
 
void Encoder_Reset(void);
void Start(void);
void Stop(void);
 
void PSD_IR_Check(void);
void IR_ADConverter_Save(int Delay, int Value);
 
void Move_XYR_Encoder(int X, int Y, int R, int encoder_value, int stop_cm);
void Move_XYR_IR(int X, int Y, int R, int search_type, int IR_num);
void Move_XYR_PSD(int X, int Y, int R, int PSD_ch, int stop_distance);
void Move_XYR_Time(int X, int Y, int R, int stop_time);
 
void Move_Line(int X, int R, int stop_check, int LINE);
 
void Move_PSD_Encoder(int X, int PSD_ch, int Distance, int stop_cm);
void Move_PSD_IR(int X, int PSD_ch, int Distance);
void Move_PSD_PSD(int X, int PSD_ch, int Distance, int stop_Distance);
void Move_PSD_Time(int X, int PSD_ch, int Distance, int stop_time);
 
ISR(ADC_vect)
{
 static unsigned char Channel_cnt=0;       // ADC Port의 Channel 변경 카운트 변수
 unsigned char ADC_Low=0,ADC_High=0;       // 변환 완료된 AD값을 저장하는 변수
 unsigned char Change8=0;         // 8bit의 AD 값을 저장하는 임시 변수
 unsigned int AD_Val=0;          // 10bit의 AD 값을 저장하는 변수
 Channel_cnt++;            // PSD 10bit , IR_Sensor 8bit
 AD_Val=ADCW;
 if(Channel_cnt<6)           // IR_Sensor 8비트 저장
 {
  ADC_High=(unsigned char)((AD_Val>>8)&0xff);
  ADC_Low=(unsigned char)(AD_Val&0xff);
  Change8=((ADC_Low>>2)|(ADC_High<<6));
  AD_IR_Save[Channel_cnt-1]=Change8;
 }
 else
 {
  PSD_AD_Save[Channel_cnt-6]=AD_Val;      // PSD Sensor 1,2,3 값을 저장 10비트
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
 
long MCtrl_REncoder(unsigned char ch)
{
 unsigned char FCurCnt[3] = {0,0,0}, BCurCnt[3] = {0,0,0};
 static unsigned char FPreCnt[3] = {0,0,0}, BPreCnt[3] = {0,0,0};
 unsigned char fcnt = 0, bcnt = 0;
 unsigned char i = 0;
 if(ch == 0){         //엔코더 선택
 cbi_(PORTC,5); cbi_(PORTC,4);
 }else if(ch == 1){
 cbi_(PORTC,5); sbi_(PORTC,4);
 }else if(ch == 2){
 sbi_(PORTC,5); cbi_(PORTC,4);
 }
 cbi_(PORTC,3);         //정방향 읽기
 FCurCnt[ch] = 0;
  cbi_(PORTC,7); sbi_(PORTC,7);     //hc165 data load
 for(i=0; i<8; i++){
 if(PINB& BV_(0)) FCurCnt[ch] |= (1<<(8-i));
 sbi_(PORTB,1); cbi_(PORTB,1);     //hc165 clk
 }
 sbi_(PORTC,3);         //역방향 읽기
 BCurCnt[ch] = 0;
 cbi_(PORTC,7); sbi_(PORTC,7);     //hc165 data load
 for(i=0; i<8; i++){
 if(PINB& BV_(0)) BCurCnt[ch] |= (1<<(8-i));
 sbi_(PORTB,1); cbi_(PORTB,1);     //hc165 clk
 }
 fcnt = FCurCnt[ch] - FPreCnt[ch];
 bcnt = BCurCnt[ch] - BPreCnt[ch];
 Encoder[ch] += (long)fcnt + (long)bcnt;
 FPreCnt[ch] = FCurCnt[ch];
 BPreCnt[ch] = BCurCnt[ch];
 return Encoder[ch];
}
 
void MCtrl_PwmDuty(unsigned char ch,int pwm) // 모터선택, 모터속도 0~255
{
 unsigned char dir=1, pwm_value=0;
 if(pwm == 0)        //모터 정지
 {
  pwm_value = (int)(0);
 }
 else if(pwm > 0)       //정방향 설정
 {
  dir = 1;
  if(pwm > 255) pwm_value = 255;
  else pwm_value = (int)(pwm);
 }
 else if(pwm < 0)       //역방향 설정
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
 
ISR(TIMER2_OVF_vect)            //-// 1ms마다 진입 //-//
{
 static unsigned char ir_cnt=0, cnt=0, cnt2=0, psd_move_cnt=0;
 TCNT2 = 5;              // 1mSec 진입 TCNT 설정 250 * 4us//
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
 
ISR(TIMER0_OVF_vect) //16uSec
{
 TovfCnt++;
}
 
ISR(TIMER1_OVF_vect)
{
 asm volatile("nop"::);
}
 
/*------------------------------------------------------------------------
 LCD controller의 instruction register에 값을 기록한다
 설명     : LCD controller에 명령을 내린다
 Note     : LCD panel의 인터페이스 방법에 따라 compile 내용이 달라진다
  ------------------------------------------------------------------------*/
void lcd_write_command(unsigned char val)
{
 PORTC &= ~(_BV(1));
 PORTC &= ~(_BV(2));
 PORTA = val;
 PORTC |= _BV(0);
 PORTC &= ~_BV(0);
}
 
/*------------------------------------------------------------------------
 LCD controller의 ddata register에 값을 기록한다
 설명     : LCD controller에 data를 기록한다
 Note     : LCD panel의 인터페이스 방법에 따라 compile 내용이 달라진다
  ------------------------------------------------------------------------*/
void lcd_write_data(unsigned char val)
{
 PORTC |= _BV(1);
 PORTC &= ~(_BV(2));
 PORTA = val;
 PORTC |= _BV(0);
 PORTC &= ~_BV(0);
}
 
/*------------------------------------------------------------------------
 디스플레이 드라이버 초기화
 설명      : 디스플레이 드라이버를 초기화한다
 전달인자 : maxrows  LCD 행의 수
      maxcols  LCD 각 행당 열의 수
 리턴값  : 없음
 Note  : - init_LCD()는 커널 서비스(시간지연)를 필요로 하므로 멀티태스킹 커널이 시작된 후에
     호출해야 한다
    - init_LCD()는 시스템 초기화 시 한번만 호출해야 한다
  ------------------------------------------------------------------------*/
void lcd_init(unsigned char maxrows, unsigned char maxcols)
{
 int i=0,j=0;
 
 lcd_maxrows = maxrows;
 lcd_maxcols = maxcols;
 // 디스플레이 모듈 초기화   
 uSecDelay(45000);
 lcd_write_command(0x38);
 uSecDelay(1000);
 lcd_write_command(0x0C);
 uSecDelay(1000);
 lcd_write_command(0x06);
 uSecDelay(1000);
 lcd_write_command(0x1c);
 // 디스플레이 모듈 테스트
 for(i=20;i>0;i--)
 {
  lcd_display_str(i,j,"-★- KMC KR2008 -★-");
  msDelay(5);
  j++;
  if(j>1) j=0;
 }
  lcd_display_str(0,1," -VER.HGBSRA120525- ");
 msDelay(50);
 // 디스플레이 모듈 공백화
 lcd_clear_screen();
 msDelay(50);
}
 
/*------------------------------------------------------------------------
  커서 임의 위치로 보내기 : 커서를 LCD버퍼의 임의 위치로 보내다
        'row' 는 LCD에서 커서의 행 위치
        'row'는 0에서 'lcd_maxrows - 1'까지의 값을 갖는다
        'col' 는 LCD에서 커서의 열 위치
        'col'은 0에서 'lcd_maxcols - 1'까지의 값을 갖는다
  ------------------------------------------------------------------------*/
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
 
/*------------------------------------------------------------------------
   문자 표시 : 표시장치에 ASCII 문자열을 표시한다
     row' 는 LCD에서 커서의 행 위치 'row'는 0에서 'lcd_maxrows - 1'까지의 값을 갖는다
               'col' 는 LCD에서 커서의 열 위치 'col'은 0에서 'lcd_maxcols - 1'까지의 값을 갖는다
               'ch'  row/col에 디스플레이 하고자 하는 문자?
  ------------------------------------------------------------------------*/
void lcd_display_char(unsigned char row, unsigned char col, signed char ch)
{
 if(row < lcd_maxrows && col < lcd_maxcols) {
  uSecDelay(30);
  lcd_move_cursor(row, col);  // 커서를 ROW/COL에 위치시킨다
  uSecDelay(30);
  lcd_write_data(ch);    // 표시장치에 문자를 쓴다 
  uSecDelay(30);
 }
}
 
/*------------------------------------------------------------------------
 ASCII문자열 표시 : 표시장치에 ASCII 문자열을 표시한다
      row' 는 LCD에서 커서의 행 위치 'row'는 0에서 'lcd_maxrows - 1'까지의 값을 갖는다
      'col' 는 LCD에서 커서의 열 위치 'col'은 0에서 'lcd_maxcols - 1'까지의 값을 갖는다
      'str'  row/col에 디스플레이 하고자 하는 ASCII문자열
  ------------------------------------------------------------------------*/
void lcd_display_str(unsigned char row, unsigned char col, signed char* str)
{
 unsigned char i;
 if(row < lcd_maxrows && col < lcd_maxcols) {
  lcd_move_cursor(row, col);     // 커서를 ROW/COL에 위치시킨다
  i = col;         // 최대 허용가능한 값으로 리미트 카운터를 세트한다
  while(i++ < lcd_maxcols && *str) {   // (str+limit)와 lcd_maxcols 사이에 문자를 쓴다
   uSecDelay(30);
   lcd_write_data(*str++);    // 표시장치에 문자를 쓴다
  }
 }
}
 
void LED(unsigned char pin, unsigned char on_off)  //LED on/off  ->  pin : 0~3   on/off : on=1, off=0 
{
 if(pin>7) return;
 
 if(on_off) sbi_(PORTD,pin);
 else  cbi_(PORTD,pin);
}
 
void mcu_init(void)
{
 //PORT 초기화
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
 
 // TIMER 초기화
 TCNT0 = 0x00;      // timer0 ovf 설정 
 TCCR0 = 0x01;        // 1 prescale
 ASSR = 0;
 TIMSK |= _BV(TOIE0);    // timer0 enable
 TIMSK |= _BV(TOIE1);    // timer1 enable
 TCCR1B = _BV(CS10);     // 내부클럭을 사용 (CK/1) - /*fast pwm*/
    TCCR1A = 0xA9;                      // 8 bit PWM
    TCNT1 = 0x00;                       // set counter1 to zero
 TCCR3B = _BV(CS10);    
    TCCR3A = 0xA9;
 TCCR3C = 0x00;
 
 OCR1A = 0x00;                       // set output compare register A to zero
 OCR1B = 0x00;      // set output compare register B to zero
 OCR1C = 0x00;      // set output compare register C to zero
 OCR3A = 0x00;                       // set output compare register A to zero
 OCR3B = 0x00;      // set output compare register B to zero
 OCR3C = 0x00;      // set output compare register C to zero
 
 TCNT2 = 0x00;      
 TCCR2 = 0x03;
 TIMSK |= _BV(TOIE2);               // timer2 enable 
 
 //ADC초기화
 ADMUX = 0x00;             // ADC 초기화  Now AD_Port Out Because Start Channel 1 //
 ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADIE)|_BV(ADPS2)|_BV(ADPS1); // Division 32 & ADCSRA = 0xce; 
 
 sei();        // +global int enable
    lcd_init(2, 20);     // initialize lcd display
 lcd_clear_screen();
 
}
 
 /*------------------------------------------------------------------------
 라인 지우기 : 한 라인을 지우고 커서를 라인의 처음 위치로 보낸다
     'line' 은 지울 라인 넘버 0에서 'lcd_maxrows - 1'까지의 값을 갖는다
  ------------------------------------------------------------------------*/
void lcd_clear_line(unsigned char line)
{
    unsigned char i;
 
    if(line < lcd_maxrows) {        
        lcd_move_cursor(line, 0);    // 지울 라인의 처음 위치로 커서를 보낸다       
        for(i=0; i<lcd_maxcols; i++) {   // 현재 라인의 위치로 부터 모든 행에 ' '을 쓴다
            uSecDelay(30);                
            lcd_write_data(' ');    // 현재 위치에 ASCII space 값을 쓴다           
        }
        lcd_move_cursor(line, 0);    // 커서를 라인의 처음 위치로 보낸다              
    }
}
 
 /*------------------------------------------------------------------------
 스크린 지우기 : 디스플레이 전체를 클리어한다
  ------------------------------------------------------------------------*/
void lcd_clear_screen(void)
{
 uSecDelay(30);
 lcd_write_command(0x01);     // 클리어 명령을 쓴다
 uSecDelay(2000);       // 2msec이상 기다린다(두개의 틱이면 충분함)
}
 
// 3자리수 단위로 데이터 출력
// LCD 가로 위치 / LCD 세로 위치 / 출력물
void LCD_num_3(char y, char x, int num)
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
 
// 2자리수 단위로 데이터 출력
// LCD 가로 위치 / LCD 세로 위치 / 출력물
void LCD_num_2(char y, char x, int num)
{
 unsigned char N1000,N100,N10,N1;
 N1000 = num/1000 + '0';
 N100 = (num%1000)/100 + '0';
 N10 = (num%100)/10 + '0';
 N1 = num%10 + '0';
 lcd_display_char(y,x,N10);
 lcd_display_char(y,x+1,N1);
}
 
// 1자리수 단위로 데이터 출력
// LCD 가로 위치 / LCD 세로 위치 / 출력물
void LCD_num_1(char y, char x, int num)
{
 unsigned char N1000,N100,N10,N1;
 N1000 = num/1000 + '0';
 N100 = (num%1000)/100 + '0';
 N10 = (num%100)/10 + '0';
 N1 = num%10 + '0';
 lcd_display_char(y,x,N1);
}
 
// 엔코더 초기화
void Encoder_Reset(void)
{
 if(Motor_Break==1) omni(0,0,0);
 MCtrl_REncoder(0); Encoder[0]=0;
 MCtrl_REncoder(1); Encoder[1]=0;
 MCtrl_REncoder(2); Encoder[2]=0;
}
 
// 로봇 시작 선언
void Start(void)
{
 lcd_clear_screen();
 PORTD=0x00;
 Encoder_Reset();
}
 
// 로봇 정지 선언
void Stop(void)
{
 lcd_clear_screen();
 PORTD=0xff;
 Motor_Break=1;
 Encoder_Reset();
}
 
// PSD & IR 센서 LCD 출력
void PSD_IR_Check(void)
{
 PSD(0);PSD(1);PSD(2);
 LCD_num_2(0,0,PSD_CMR[2]);
 LCD_num_2(3,0,PSD_CMR[1]);
 LCD_num_2(6,0,PSD_CMR[0]);
 LCD_num_1(0,1,IR_Digital[4]);
 LCD_num_1(2,1,IR_Digital[3]);
 LCD_num_1(4,1,IR_Digital[2]);
 LCD_num_1(6,1,IR_Digital[1]);
 LCD_num_1(8,1,IR_Digital[0]);
}
 
// IR센서 값 구하기
// 라인 검색 시간 / 라인 기준값
void IR_ADConverter_Save(int Delay, int Value)
{
 int BLine=0,i=0, Count=0, Out=0;
 int BFirst[5]={0,0,0,0,0},BSave[5]={255,255,255,255,255};
 for(;;)
 {
  PSD(2);LCD_num_2(1,1,PSD_CMR[2]);
  PSD(1);LCD_num_2(4,1,PSD_CMR[1]);
  PSD(0);LCD_num_2(7,1,PSD_CMR[0]);
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
 
//초기셋팅
//라인 검색 시간 / 라인 기준값 / 모터 정지 방식 / 모터 보정 값
void Settings(int IR_Read_Time,int IR_ADConverter_Center,int Motor_Break_Type,int Motor_Calibration_Data)
{
 mcu_init();
 IR_ADConverter_Save(IR_Read_Time,IR_ADConverter_Center);
 Motor_Break=Motor_Break_Type;
 Motor_Calibration=Motor_Calibration_Data;
}
 
// 강제이동 - 엔코더정지
// 전후진속도 / 좌우 이동 속도 / 회전 속도 / 엔코더 데이터 값 / 정지할 CM
void Move_XYR_Encoder(int X, int Y, int R, int encoder_data, int stop_cm)
{
 int encoder_value = encoder_data;
 
 if(encoder_data == 'F'||encoder_data == 'B'||encoder_data == 'f'||encoder_data == 'b') encoder_value = 131.20;
 else if(encoder_data == 'R'||encoder_data == 'L'||encoder_data == 'r'||encoder_data == 'l') encoder_value = 75.75;
 else if(encoder_data == 'DEG' || encoder_data == 'Deg' || encoder_data == 'deg') encoder_value = 21.5;
 
 double stop_encoder = stop_cm*encoder_value;
 double Value, MA=0;
 int Speed_X=0, Speed_Y=0, Speed_R=0, Delay=0;
 
 Encoder_Reset();
 
 for(;;)
 {
  MCtrl_REncoder(0);
  MCtrl_REncoder(1);
  MCtrl_REncoder(2);
  
  if(Encoder[0]/100>Encoder[2]/100) Value = Motor_Calibration;
  else if(Encoder[2]/100>Encoder[0]/100) Value = -Motor_Calibration;
  
  if((MA==0)&&(Encoder[0] > (stop_cm/0.25))&&(Encoder[2] > (stop_cm/0.25)))
  {
   Delay++;
   if(Delay > 5)
   {
    if(X>0) Speed_X++;
    else if(X<0) Speed_X--;
    else Speed_X=0;
    
    if(Y>0) Speed_Y++;
    else if(Y<0) Speed_Y--;
    else Speed_Y=0;
    
    if(R>0) Speed_R++;
    else if(R<0) Speed_R--;
    else Speed_R=0;
    
    Delay = 0;
   }
  }
  
  else if((MA==1)&&(Encoder[0] > (stop_cm/0.75))&&(Encoder[2] > (stop_cm/0.75)))
  {
   Delay++;
   if(Delay > 5)
   {
    if(X>0) Speed_X--;
    else if(X<0) Speed_X++;
    else Speed_X=0;
    
    if(Y>0) Speed_Y--;
    else if(Y<0) Speed_Y++;
    else Speed_Y=0;
    
    if(R>0) Speed_R--;
    else if(R<0) Speed_R++;
    else Speed_R=0;
    
    Delay = 0;
   }
  }
  
  if((MA==0)&&(Speed_X > X)&&(Speed_Y > Y)&&(Speed_R > R))
  {
   MA=1;
   Speed_X = X;
   Speed_Y = Y;
   Speed_R = R;
  }
  
  MCtrl_PwmDuty(0,(-Speed_X)+(Speed_Y/2)+(Speed_R)+Value);
  MCtrl_PwmDuty(1,(-Speed_Y)+Value);
  MCtrl_PwmDuty(2,( Speed_X)+(Speed_Y/2)+(Speed_R)+Value);
  
  if((Encoder[0] > stop_cm)&&(Encoder[2] > stop_cm))
  {
   Encoder_Reset();
   break;
  }
 }
}
 
// 강제이동 - IR정지
// 전후진속도 / 좌우이동속도 / 좌우회전속도 / 0=선택한 1개만 잡힐시 정지, 1=5개중 1개라도 잡히면 정지 / 선택할 센서 
void Move_XYR_IR(int X, int Y, int R, int search_type, int IR_num)
{
 double Value;
 
 Encoder_Reset();
 
 for(;;)
 {
  MCtrl_REncoder(0);
  MCtrl_REncoder(1);
  MCtrl_REncoder(2);
  
  if(Encoder[0]/100>Encoder[2]/100) Value = Motor_Calibration;
  else if(Encoder[2]/100>Encoder[0]/100) Value = -Motor_Calibration;
  
  MCtrl_PwmDuty(0,(-X)+(Y/2)+(R)+Value);
  MCtrl_PwmDuty(1,(-Y)+Value);
  MCtrl_PwmDuty(2,( X)+(Y/2)+(R)+Value);
  
  if(search_type>0) IR_num++;
  if(IR_num>4) IR_num=0;
  
  if(IR_Digital[IR_num] == 1)
  {
   Encoder_Reset();
   break;
  }
 }
}
 
// 강제이동 - PSD정지
// 전후진속도 / 좌우이동속도 / 좌우회전속도 / 정지시 사용할 PSD 채널 / 정지거리
void Move_XYR_PSD(int X, int Y, int R, int PSD_ch, int stop_distance)
{
 double Value;
 
 Encoder_Reset();
 
 for(;;)
 {
  PSD(0);
  PSD(1);
  PSD(2);
  
  MCtrl_REncoder(0);
  MCtrl_REncoder(1);
  MCtrl_REncoder(2);
  
  if(Encoder[0]/100>Encoder[2]/100) Value = Motor_Calibration;
  else if(Encoder[2]/100>Encoder[0]/100) Value = -Motor_Calibration;
  
  MCtrl_PwmDuty(0,(-X)+(Y/2)+(R)+Value);
  MCtrl_PwmDuty(1,(-Y)+Value);
  MCtrl_PwmDuty(2,( X)+(Y/2)+(R)+Value);
  
  if(PSD_CMR[PSD_ch] > stop_distance)
  {
   Encoder_Reset();
   break;
  }
 }
}
 
// 강제이동 - 시간정지
// 전후진속도 / 좌우이동속도 / 좌우회전속도 / 정지시간
void Move_XYR_Time(int X, int Y, int R, int stop_time)
{
 double Value, MA=0;
 int Speed_X=0, Speed_Y=0, Speed_R=0, Delay=0, time_delay=0;
 
 Encoder_Reset();
 
 for(;;)
 {
  time_delay++;
  
  MCtrl_REncoder(0);
  MCtrl_REncoder(1);
  MCtrl_REncoder(2);
  
  if(Encoder[0]/100>Encoder[2]/100) Value = Motor_Calibration;
  else if(Encoder[2]/100>Encoder[0]/100) Value = -Motor_Calibration;
  
  if((MA==0)&&(time_delay > (stop_time/0.25)))
  {
   Delay++;
   if(Delay > 5)
   {
    if(X>0) Speed_X++;
    else if(X<0) Speed_X--;
    else Speed_X=0;
    
    if(Y>0) Speed_Y++;
    else if(Y<0) Speed_Y--;
    else Speed_Y=0;
    
    if(R>0) Speed_R++;
    else if(R<0) Speed_R--;
    else Speed_R=0;
    
    Delay = 0;
   }
  }
  
  else if((MA==1)&&(time_delay > (stop_time/0.75)))
  {
   Delay++;
   if(Delay > 5)
   {
    if(X>0) Speed_X--;
    else if(X<0) Speed_X++;
    else Speed_X=0;
    
    if(Y>0) Speed_Y--;
    else if(Y<0) Speed_Y++;
    else Speed_Y=0;
    
    if(R>0) Speed_R--;
    else if(R<0) Speed_R++;
    else Speed_R=0;
    
    Delay = 0;
   }
  }
  
  if((MA==0)&&(Speed_X > X)&&(Speed_Y > Y)&&(Speed_R > R))
  {
   MA=1;
   Speed_X = X;
   Speed_Y = Y;
   Speed_R = R;
  }
  
  MCtrl_PwmDuty(0,(-Speed_X)+(Speed_Y/2)+(Speed_R)+Value);
  MCtrl_PwmDuty(1,(-Speed_Y)+Value);
  MCtrl_PwmDuty(2,( Speed_X)+(Speed_Y/2)+(Speed_R)+Value);
  
  if(time_delay>stop_time)
  {
   Encoder_Reset();
   break;
  }
 }
}
 
// IR센서로 바닥 검은 라인 타고 이동
// 전진속도 / 위치보정속도 / 간선체크 / 인식방식
void Move_Line(int X, int R, int stop_check, int LINE)
{
 flag=0;
 flag_check=5;
 signed int IR_position=0, line_count=0;
 
 for(;;)
 {
  if(IR_Operation_Flag==1)
  {
   IR_Operation_Flag=0;
   
   //All Black
   if(LINE=='B'&&IR_Digital[0]==1&&IR_Digital[1]==1&&IR_Digital[2]==1&&IR_Digital[3]==1&&IR_Digital[4]==1)
   {
    if(flag==0)
    {
     flag=1;
     line_count++;
    }
   }
   //All White
   else if(LINE=='W'&&IR_Digital[0]==0&&IR_Digital[1]==0&&IR_Digital[2]==0&&IR_Digital[3]==0&&IR_Digital[4]==0)
   {
    if(flag==0)
    {
     flag=1;
     line_count++;
    }
   }
   //Left
   else if(LINE=='L'&&IR_Digital[0]==1&&IR_Digital[1]==1&&IR_Digital[2]==1&&IR_Digital[3]==0&&IR_Digital[4]==0)
   {
    if(flag==0)
    {
     flag=1;
     line_count++;
    }
   }
   else if(LINE=='L2'&&IR_Digital[0]==1&&IR_Digital[1]==1&&IR_Digital[2]==0&&IR_Digital[3]==0&&IR_Digital[4]==0)
   {
    if(flag==0)
    {
     flag=1;
     line_count++;
    }
   }
   else if(LINE=='L1'&&IR_Digital[0]==1&&IR_Digital[1]==0&&IR_Digital[2]==0&&IR_Digital[3]==0&&IR_Digital[4]==0)
   {
    if(flag==0)
    {
     flag=1;
     line_count++;
    }
   }
   //Right
   else if(LINE=='R'&&IR_Digital[0]==0&&IR_Digital[1]==0&&IR_Digital[2]==1&&IR_Digital[3]==1&&IR_Digital[4]==1)
   {
    if(flag==0)
    {
     flag=1;
     line_count++;
    }
   }
   else if(LINE=='R2'&&IR_Digital[0]==0&&IR_Digital[1]==0&&IR_Digital[2]==0&&IR_Digital[3]==1&&IR_Digital[4]==1)
   {
    if(flag==0)
    {
     flag=1;
     line_count++;
    }
   }
   else if(LINE=='R1'&&IR_Digital[0]==0&&IR_Digital[1]==0&&IR_Digital[2]==0&&IR_Digital[3]==0&&IR_Digital[4]==1)
   {
    if(flag==0)
    {
     flag=1;
     line_count++;
    }
   }
   else if(LINE=='PSD')
   {
    PSD(0);PSD(1);PSD(2);
    
    if(PSD_CMR[1] >= stop_check)
    {
     Encoder_Reset();
     break;
    }
   }
   else
   {
    if(IR_Digital[0]==0&&IR_Digital[1]==0&&IR_Digital[2]==1&&IR_Digital[3]==0&&IR_Digital[4]==0) IR_position=0;
    else if(IR_Digital[4]==1) IR_position=IR_position-5;
    else if(IR_Digital[3]==1) IR_position--;
    else if(IR_Digital[0]==1) IR_position=IR_position+5;
    else if(IR_Digital[1]==1) IR_position++;
    if    (IR_position>R)  IR_position=R;
    else if(IR_position<(-R)) IR_position=(-R);
   }
   
   if(stop_check == line_count)
   {
    Encoder_Reset();
    break;
   }
   
   omni(X,IR_position,IR_position);
  }
 }
}
 
// 벽타고이동 - 엔코더정지
// 전후진속도 / 벽 타고 이동시 사용할 PSD 센서번호 / 유지거리 / 정지 CM
void Move_PSD_Encoder(int X, int PSD_ch, int Distance, int stop_cm)
{
 flag=1;
 flag_check=5;
 
 double stop_encoder = stop_cm*131.20;
 int psd_cmd=0, Value=0;
 
 if((X>0 && PSD_ch==2)||(X<0 && PSD_ch==0)) psd_cmd=-1;
 else psd_cmd=1;
 
 for(;;)
 {
  PSD(0);PSD(1);PSD(2);
  
  if(PSD_CMR[PSD_ch] < 70)  Value=60*psd_cmd;
  else if(PSD_CMR[PSD_ch] < 50) Value=30*psd_cmd;
  else if(PSD_CMR[PSD_ch] < 30) Value=10*psd_cmd;
  else if(PSD_CMR[PSD_ch] < 15) Value= 5*psd_cmd;
  
  if(PSD_CMR[PSD_ch] < 70)
  {
   MCtrl_PwmDuty(0,-X-Value);
   MCtrl_PwmDuty(1,  -Value);
   MCtrl_PwmDuty(2, X-Value);
  }
  else if(PSD_CMR[PSD_ch] > 70)
  {
   if(PSD_ch==0){
    MCtrl_PwmDuty(0,-150);
    MCtrl_PwmDuty(1, 150);
    MCtrl_PwmDuty(2, 120);
   }
   else if(PSD_ch==2){
    MCtrl_PwmDuty(0,-120);
    MCtrl_PwmDuty(1,-150);
    MCtrl_PwmDuty(2, 150);
   }
  }
  
  if((Encoder[0] > stop_cm)&&(Encoder[2] > stop_cm))
  {
   Encoder_Reset();
   break;
  }
 }
}
 
// 벽타고이동 - IR정지
// 전후진속도 / 벽 타고 이동시 사용할 PSD 센서번호 / 유지거리
void Move_PSD_IR(int X, int PSD_ch, int Distance)
{
 flag=1;
 flag_check=5;
 
 int psd_cmd=0, Value=0, IR_num=0;
 
 if((X>0 && PSD_ch==2)||(X<0 && PSD_ch==0)) psd_cmd=-1;
 else psd_cmd=1;
 
 for(;;)
 {
  PSD(0);PSD(1);PSD(2);
  
  if(PSD_CMR[PSD_ch] < 70)  Value=60*psd_cmd;
  else if(PSD_CMR[PSD_ch] < 50) Value=30*psd_cmd;
  else if(PSD_CMR[PSD_ch] < 30) Value=10*psd_cmd;
  else if(PSD_CMR[PSD_ch] < 15) Value= 5*psd_cmd;
  
  if(PSD_CMR[PSD_ch] < 70)
  {
   MCtrl_PwmDuty(0,-X-Value);
   MCtrl_PwmDuty(1,  -Value);
   MCtrl_PwmDuty(2, X-Value);
  }
  else if(PSD_CMR[PSD_ch] > 70)
  {
   if(PSD_ch==0){
    MCtrl_PwmDuty(0,-150);
    MCtrl_PwmDuty(1, 150);
    MCtrl_PwmDuty(2, 120);
   }
   else if(PSD_ch==2){
    MCtrl_PwmDuty(0,-120);
    MCtrl_PwmDuty(1,-150);
    MCtrl_PwmDuty(2, 150);
   }
  }
  
  IR_num++;
  if(IR_num>4) IR_num=0;
  
  if(IR_Digital[IR_num] == 1)
  {
   Encoder_Reset();
   break;
  }
  
 }
}
 
// 벽타고이동 - PSD정지
// 전후진속도 / 벽 타고 이동시 사용할 PSD 센서번호 / 유지거리 / 정지거리
void Move_PSD_PSD(int X, int PSD_ch, int Distance, int stop_Distance)
{
 flag=1;
 flag_check=5;
 
 int psd_cmd=0, Value=0;
 
 if((X>0 && PSD_ch==2)||(X<0 && PSD_ch==0)) psd_cmd=-1;
 else psd_cmd=1;
 
 for(;;)
 {
  PSD(0);PSD(1);PSD(2);
  
  if(PSD_CMR[PSD_ch] < 70)  Value=60*psd_cmd;
  else if(PSD_CMR[PSD_ch] < 50) Value=30*psd_cmd;
  else if(PSD_CMR[PSD_ch] < 30) Value=10*psd_cmd;
  else if(PSD_CMR[PSD_ch] < 15) Value= 5*psd_cmd;
  
  if(PSD_CMR[PSD_ch] < 70)
  {
   MCtrl_PwmDuty(0,-X-Value);
   MCtrl_PwmDuty(1,  -Value);
   MCtrl_PwmDuty(2, X-Value);
  }
  else if(PSD_CMR[PSD_ch] > 70)
  {
   if(PSD_ch==0){
    MCtrl_PwmDuty(0,-150);
    MCtrl_PwmDuty(1, 150);
    MCtrl_PwmDuty(2, 120);
   }
   else if(PSD_ch==2){
    MCtrl_PwmDuty(0,-120);
    MCtrl_PwmDuty(1,-150);
    MCtrl_PwmDuty(2, 150);
   }
  }
  
  if(PSD_CMR[1] <= stop_Distance)
  {
   Encoder_Reset();
   break;
  }
 }
}
 
// 벽타고이동 - 시간정지
// 전후진속도 / 벽 타고 이동시 사용할 PSD 센서번호 / 유지거리 / 정지시간
void Move_PSD_Time(int X, int PSD_ch, int Distance, int stop_time)
{
 flag=1;
 flag_check=5;
 
 int psd_cmd=0, Value=0, time_delay=0;
 
 if((X>0 && PSD_ch==2)||(X<0 && PSD_ch==0)) psd_cmd=-1;
 else psd_cmd=1;
 
 for(;;)
 {
  time_delay++;
  PSD(0);PSD(1);PSD(2);
  
  if(PSD_CMR[PSD_ch] < 70)  Value=60*psd_cmd;
  else if(PSD_CMR[PSD_ch] < 50) Value=30*psd_cmd;
  else if(PSD_CMR[PSD_ch] < 30) Value=10*psd_cmd;
  else if(PSD_CMR[PSD_ch] < 15) Value= 5*psd_cmd;
  
  if(PSD_CMR[PSD_ch] < 70)
  {
   MCtrl_PwmDuty(0,-X-Value);
   MCtrl_PwmDuty(1,  -Value);
   MCtrl_PwmDuty(2, X-Value);
  }
  else if(PSD_CMR[PSD_ch] > 70)
  {
   if(PSD_ch==0){
    MCtrl_PwmDuty(0,-150);
    MCtrl_PwmDuty(1, 150);
    MCtrl_PwmDuty(2, 120);
   }
   else if(PSD_ch==2){
    MCtrl_PwmDuty(0,-120);
    MCtrl_PwmDuty(1,-150);
    MCtrl_PwmDuty(2, 150);
   }
  }
  
  if(stop_time > time_delay)
  {
   Encoder_Reset();
   break;
  }
 }
}
 
///// ///// ///// ///// ///// ///// ///// ///// ///// /////
 
///// ///// ///// ///// ///// ///// ///// ///// ///// /////
 
int main()
{
 Settings(5000,100,1,1); // 초기셋팅
 
 for(;;)
 {
  PSD_IR_Check();   // PSD & IR 센서 LCD 출력
  
  if(SW0){    // 0번 스위치
   Start();   // 시작
   
   
   Stop();    // 정지
  }
  
  else if(SW1){   // 1번 스위치
   Start();   // 시작
   
   
   Stop();    // 정지
  }
  
  else if(SW2){   // 2번 스위치
   Start();   // 시작
   
   
   Stop();    // 정지
  }
  
  else if(SW3){   // 3번 스위치
   Start();   // 시작
   
   
   Stop();    // 정지
  }
 }
}
 
///// ///// ///// ///// ///// ///// ///// ///// ///// /////
 
///// ///// ///// ///// ///// ///// ///// ///// ///// /////