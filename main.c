#include "avr_system.h"
#include "system.c"
#include "settings.c"
#include "moves.c"
 
int main()
{
 Settings(5000,100,1,1);
 
 for(;;)
 {
  PSD_IR_Check();
  
  if(SW0){
   Start();
   
   
   Stop();
  }
  
  else if(SW1){
   Start();
   
   
   Stop();
  }
  
  else if(SW2){
   Start();
   
   
   Stop();
  }
  
  else if(SW3){
   Start();
   
   
   Stop();
  }
 }
}