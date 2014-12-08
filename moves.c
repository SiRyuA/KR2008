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
 
void Move_XYR_PSD(int X, int Y, int R, int search_type, int PSD_ch, int stop_distance)
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
  else if(PSD_CMR[PSD_ch] > 70)  //회전
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
  else if(PSD_CMR[PSD_ch] > 70)  //회전
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
  else if(PSD_CMR[PSD_ch] > 70)  //회전
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
  else if(PSD_CMR[PSD_ch] > 70)  //회전
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