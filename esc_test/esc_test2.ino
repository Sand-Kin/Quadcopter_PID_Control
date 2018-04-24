#include <Servo.h>
#include <PID_v1.h>

#define NUM_ESCS 4

Servo esc[NUM_ESCS];
int throttlePin = 0;
 
void setup()
{
  esc[0].attach(3);
  esc[1].attach(9);
  esc[2].attach(10);
  esc[3].attach(11);
  initializeMotor(esc);
}
 
void loop()
{
}

void initializeMotor(Servo esc[]){
  for (int i = 0; i < NUM_ESCS; i++){
    esc[i].write(0);
  }
  delay(3000);
  for (int i = 0; i < NUM_ESCS; i++){
    esc[i].write(50);
  }
  delay(3000);
  for (int i = 0; i < NUM_ESCS; i++){
    esc[i].write(0);
  }
  delay(3000);
  for (int i = 0; i < NUM_ESCS; i++){
    esc[i].write(50);
  }
  delay(3000);
  for (int i = 0; i < NUM_ESCS; i++){
    esc[i].write(75);
  }
  delay(5000);
  for (int i = 0; i < NUM_ESCS; i++){
    esc[i].write(0);
  }
}
