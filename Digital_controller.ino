#include <Wire.h>
#include <TimerOne.h>
#include <MCP4725.h>

volatile int adc_value, flag=0;
double u,e,r,y;
int32_t value=0;
char in_data;
long int i=0;
static double u1, u2, ee1, ee2;
MCP4725 MCP(0x60);

void setup()
{
   Wire.begin();
   Serial.begin(115200);
   Timer1.initialize(2000); // Sampling period: 2000us(2ms) 
   Timer1.attachInterrupt(timerIsr);
   MCP.begin();
   Wire.setClock(800000);

   r=1;                     //Referenc input : 1
   MCP.setValue(2053);      //Initial Control input U : 0V
   u1=0; u2=0; ee1=0; ee2=0;
}

void loop()
{
 while(Serial.available())
 {
  in_data=Serial.read();
  if(in_data=='l'){               //lead-lag compensator
     while(i<1500){
       if(flag==1){
         y=adc_value*5.0/1023.;   //convert output from digit to volt
         e=r-y;                   //error 
         u = difference_eq_Leg(e); //control input
         
         if(u<=-10)u=-10;
         if(u>=10) u=10;
         
         value=map(u,-10,10,0,4095);
         MCP.setValue(value);
         
         Serial.println(y,4);
         flag=0;
            i++;
      }
    }
  }
    if(in_data=='p'){             //PID controller
     while(i<1500){
       if(flag==1){
         y=adc_value*5.0/1023.;   //convert output from digit to volt
         e=r-y;                   //error 
         u = difference_eq_PID(e); //control input
         
         if(u<=-10)u=-10;
         if(u>=10) u=10;
         
         value=map(u,-10,10,0,4095);
         MCP.setValue(value);
         
         Serial.println(y,4);
         flag=0;
            i++;
      }
    }
  }
    if(in_data=='o'){             //open-loop
     while(i<1500){
       if(flag==1){
         y=adc_value*5.0/1023.;   //convert output from digit to volt
         u = 1;
         value=map(u,-10,10,0,4095);
         MCP.setValue(value);
         
         Serial.println(y,4);
         flag=0;
            i++;
      }
    }
  }
 }
}
double difference_eq_Leg(double ee)        //Lead-lag controller
{
    u = 1.36*u1-0.36*u2+85.5*ee-168.2*ee1+82.8*ee2;
    u2 = u1;
    u1 = u;
    ee2 = ee1;
    ee1 = ee;
    return u;
}
double difference_eq_PID(double ee)       //PID with LPF controller
{
    u = 1.057*u1-0.05743*u2+75.97*ee-147.5*ee1+71.68*ee2;    u2 = u1;
    u1 = u;
    ee2 = ee1;
    ee1 = ee;
    return u;
}
void timerIsr()
{
  adc_value=analogRead(A0);
  flag=1;
}
