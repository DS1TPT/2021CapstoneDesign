#include <HCMotor.h>
#include <Stepper.h>

#define NEMA_DIR 8
#define NEMA_CLK 9 
#define BYJ1 34
#define BYJ2 35
#define BYJ3 36
#define BYJ4 37

#define STEPS 2047

const int byjSteps = 2047;
const int spd = 50; /* 10 to 1024, lower value is faster than higher value */

HCMotor nema;
Stepper byj(STEPS, BYJ1, BYJ3, BYJ2, BYJ4);

void setup() {
  nema.Init();
  nema.attach(0, STEPPER, NEMA_CLK, NEMA_DIR);
  nema.Steps(0, CONTINUOUS);
  nema.DutyCycle(0, spd); 
  byj.setSpeed(15);
}

void loop() {
  nema.DutyCycle(0, spd);
  nema.Direction(0, REVERSE);
  delay(2000);
  nema.DutyCycle(0, 0);
  delay(2000);
  nema.DutyCycle(0, spd);
  nema.Direction(0, FORWARD);
  delay(2000);
  nema.DutyCycle(0, 0);
  delay(2000);
  byj.step(-byjSteps);
  delay(1000);
  byj.step(byjSteps);
}
