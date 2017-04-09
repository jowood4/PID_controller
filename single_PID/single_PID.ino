#include "PID_v1.h"
#include "MAX31855.h"
 
#define SPI_transfer 1
 
#if defined(SPI_transfer)
#include "SPI.h"
#endif
 
//Arduino Pins
uint8_t DHTPIN = 2;
uint8_t MAX31855_DATA = 12;
uint8_t MAX31855_CLK = 13;
uint8_t MAX31855_LAT0 = 7;
uint8_t SSR_PIN = 4;
 
//Temperature Cycle Settings
uint8_t state = 0;
uint16_t max_temp = 95; //in degrees F
uint8_t soak_time = 10; //in minutes
uint8_t ramp_rate = 10; //in degrees F/min
uint8_t cool_down = 10; //in degrees F/min
 
//PID parameters
double init_temp;
double Input, Output, Setpoint;
uint8_t init_read = 1; //flag to prevent 2 temp reads on first PID pass
uint32_t PID_interval = 5000; //time in ms to run PID interval
 
//Object Instantiation
MAX31855 thermo;
PID myPID(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT);
 
void setup()
{
   pinMode(SSR_PIN, OUTPUT);
 
   thermo.setup(MAX31855_LAT0);
   read_temps();
}
 
void loop()
{
   switch (state) {
      case 0: //Ramp Up
         run_cycle_time();
         state = state + 1;
         break;
      case 1: //Soak Time
         run_cycle_time();
         state = state + 1;
         break;
      case 2: //Cool Down
         run_cycle_time();
         state = state + 1;
         break;
      case 3: //Finished
         while(1);
    }
}
 
void run_PID(double Kp, double Ki, double Kd, uint16_t WindowSize, uint32_t time_interval)
{
   double ratio;
   uint32_t windowStartTime;
   uint8_t buttons;
 
   //Specify the links and initial tuning parameters
   myPID.SetOutputLimits(0, WindowSize);
   myPID.SetTunings(Kp, Ki, Kd);
   myPID.SetMode(AUTOMATIC);
 
   //This prevents the system from initially hanging up
   if(init_read){init_read = 0;Setpoint = Setpoint + 1;}
   else{read_temps();}
 
   windowStartTime = millis();
 
   Input = CtoF(thermo.thermocouple_temp);
   myPID.Compute();
 
   ratio = Output / WindowSize;
 
   digitalWrite(SSR_PIN, 1);
   while(millis() - windowStartTime < time_interval * ratio);
 
   digitalWrite(SSR_PIN, 0);
   while(millis() - windowStartTime < time_interval);
}
 
void run_cycle_time(void)
{
   uint32_t initial_time = millis();
   uint32_t elapsed_time;
   double diff_time_min;
   double cycle_time;
 
   //This set of statements calculates time of cycle phase
   if(state == 0) //rising ramp, increasing temperature
   {
      //Calculate time remaining in rise phase based on temperature and rate
      read_temps();
      init_temp = CtoF(thermo.thermocouple_temp);
      cycle_time = (max_temp - init_temp)/ramp_rate;
   }
   else if(state == 1) //soak time
   {
      //Soak time is already determined
      cycle_time = soak_time;
   }
   else if(state == 2) //falling ramp, decreasing temperature
   {
      //Calculate time remaining in fall phase based on temperature and rate each cycle
      read_temps();
      cycle_time = (CtoF(thermo.thermocouple_temp) - init_temp)/cool_down;
   }
 
   //Determine time left in current phase
   elapsed_time = millis();
   diff_time_min = float(elapsed_time - initial_time) / 60000;
 
   while(diff_time_min < cycle_time)
   {
      if(state == 0) //rising ramp, increasing temperature
      {
          //While increasing, Setpoint increases based on elapsed time
          Setpoint = (diff_time_min * ramp_rate) + init_temp;
      }
      else if(state == 1) //soak time
      {
          Setpoint = max_temp;
      }
      else if(state == 2) //falling ramp, decreasing temperature
      {
          //While decreasing, Setpoint increases based on elapsed time
          Setpoint = max_temp - (diff_time_min * cool_down);
      }
 
      //Determine current temp
      read_temps();
 
      //Determine PID response based on current temp
      run_PID(2, 5, 1, 500, PID_interval);
 
      //Determine time left in current phase
      elapsed_time = millis();
      diff_time_min = float(elapsed_time - initial_time) / 60000;
   }
}
 
double CtoF(double temp_C)
{
   double temp_F = (temp_C * 1.8) + 32;
   return temp_F;
}
 
void read_temps(void)
{
   thermo.read_temp();
}
