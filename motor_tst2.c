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
const int spd = 25, nemaMainSpd = 25, nemaMainSlowSpd = 70;
const int nemaAccDelay = 25, nemaDeccDelay = 25;

HCMotor nema;
Stepper byj(STEPS, BYJ1, BYJ3, BYJ2, BYJ4);

void setup() {
  // put your setup code here, to run once:
  nema.Init();
  nema.attach(0, STEPPER, NEMA_CLK, NEMA_DIR);
  nema.Steps(0, CONTINUOUS);
  nema.DutyCycle(0, spd); /* 10 to 1024 */
  byj.setSpeed(15);
}

void loop() {
  // put your main code here, to run repeatedly:
  int spdTmpAcc = nemaMainSlowSpd;
  nema.Direction(0, REVERSE);
  while (spdTmpAcc > nemaMainSpd) {
      nema.DutyCycle(0, spdTmpAcc--);
      delay(nemaAccDelay);
  }        
  nema.DutyCycle(0, spd);  
  nema.Direction(0, REVERSE);
  delay(2000);
  int spdTmpDecc = nemaMainSpd;
  nema.Direction(0, REVERSE);
  while (spdTmpDecc < nemaMainSlowSpd) {
      nema.DutyCycle(0, spdTmpDecc++);
      delay(nemaDeccDelay);
  }
  delay(1000);      
  nema.DutyCycle(0, 0);
  delay(1000);
  spdTmpAcc = nemaMainSlowSpd;
  nema.Direction(0, FORWARD);
  while (spdTmpAcc > nemaMainSpd) {
      nema.DutyCycle(0, spdTmpAcc--);
      delay(nemaAccDelay);
  }        
  nema.DutyCycle(0, spd);
  nema.Direction(0, FORWARD);
  delay(2000);
  spdTmpDecc = nemaMainSpd;
  nema.Direction(0, FORWARD);
  while (spdTmpDecc < nemaMainSlowSpd) {
      nema.DutyCycle(0, spdTmpDecc++);
      delay(nemaDeccDelay);
  }
  delay(1000);
  nema.DutyCycle(0, 0);
  delay(1000);
  byj.step(-byjSteps);
  delay(1000);
  byj.step(byjSteps);
}
