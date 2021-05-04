#include <HCMotor.h>

#define NEMA_DIR 8
#define NEMA_CLK 9 

HCMotor nema;

const int spd = 10;

void setup() {
  // put your setup code here, to run once:
  nema.Init();
  nema.attach(0, STEPPER, NEMA_CLK, NEMA_DIR);
  nema.Steps(0, CONTINUOUS);
  nema.DutyCycle(0, spd); /* 10 to 1024 */
}

void loop() {
  // put your main code here, to run repeatedly:
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
}
