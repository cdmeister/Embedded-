#include "pid.h"

uint16_t Compute(float Input) {

  /*How long since we last calculated*/
  uint32_t now = millis();
  float timeChange = (float)(now - lastTime);

  /*Compute all the working error variables*/
  float error = Setpoint - Input;
  errSum += (error * timeChange);
  float dErr = (error - lastErr) / timeChange;

  /*Compute PID Output*/
  float Output = kp * error + ki * errSum + kd * dErr;

  /*Remember some variables for next time*/
  lastErr = error;
  lastTime = now;

  return (uint16_t)Output;


}

void SetTunings(float Kp, float Ki, float Kd)
{
   kp = Kp;
   ki = Ki;
   kd = Kd;
}

void SetSetpoint(uint32_t setpoint){

  Setpoint = setpoint;

}
