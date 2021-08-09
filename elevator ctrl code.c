/* Atmega2560 마이크로 컨트롤러는 8비트이므로 기본 데이터 타입은 최적화를 위해 바이트(unsigned char)로 함. */

/* 
현재 모든 주석과 코드는 실제 하드웨어에서 테스트를 하지 않았음. 유의 바람
문의 열림과 닫힘을 감지할 센서를 부착하기 어려운 경우 문짝 모터는 일정 스텝만큼 구동하도록 함.
네마모터는 시계방향 회전이 역방향 회전인 것으로 보임. 결선에 따라 달라질 수 있으나, 카 모터 시험 코드의 방향은 그렇게 되어 있었음.
네마모터는 1 스텝이 1.8 ± 5% 이므로, 200스텝 정도 돌리면 한 바퀴 돌아감. 문을 개폐할 때 센서를 사용할 수 없는 경우 스텝각 150~250 사이의 적정값을 찾도록 함.
*/

/*
아두이노 메가 핀 번호
인터럽트 핀(순서대로): 2, 3, 21, 20, 19, 18
PWM 핀: 2~13, 44~46
디지털 핀: 0 ~ 53
아날로그 입력: A0~A15
I2C 통신 핀: 20(SCA), 21(SCL)
직렬통신 핀: 0, 1, 14~19
*/

/*
• 버튼 아날로그 검출용으로 저항을 쓸 경우 저항값은 10kOhm으로 한다.
• 호출 버튼과 목적지 버튼은 인터럽트 구현을 위해 모두 인터럽트 핀에 연결한다. 다이오드를 사용한다(오류 방지)
*/

/* 코드 시작 부분 */

/* include */
#include <HCMotor.h> /* TB6560 제어 http://scipia.co.kr/cms/blog/225 */

#define FALSE 0
#define TRUE 1

/* cmd */
#define STOP 0
#define UP 1
#define UP_SLOW 2
#define UP_ACCEL 3
#define DN 4
#define DN_SLOW 5
#define DN_ACCEL 6
#define CLOSE 0
#define OPEN 1

/* pinouts(digital) */
#define BTN_CALL_INTR 2 /* 호출 버튼 인터럽트 */
#define BTN_DEST_INTR 3 /* 목적지 버튼 인터럽트 */

#define NEMA_DOOR_DIR 5 
#define NEMA_DOOR_STEP 6

/* TB6560 드라이버의 CW-, CLK- 단자는 모두 아두이노의 GND에 연결한다. */
#define NEMA_MAIN_DIR 8 /* CW+ */
#define NEMA_MAIN_STEP 9 /* CLK+ */

#define LEDF1U 22
#define LEDF2U 23
#define LEDF2D 24
#define LEDF3U 25
#define LEDF3D 26
#define LEDF4D 27
#define LEDB1 28
#define LEDB2 29
#define LEDB3 30
#define LEDB4 31
#define LEDUP 32
#define LEDDN 33

#define IR_SNSR_1 34
#define IR_SNSR_2 35
#define IR_SNSR_3 36
#define IR_SNSR_4 37
#define BTN1U 38
#define BTN2U 39
#define BTN2D 40
#define BTN3U 41
#define BTN3D 42
#define BTN4D 43
#define BTN1 44
#define BTN2 45
#define BTN3 46
#define BTN4 47
#define IR_SNSR_DOOR_OPEN 48
#define IR_SNSR_DOOR_CLOSE 49
#define BTN_EMERGENCY 50
#define LIGHT_EMERGENCY 51

/* pinouts (analog) */

/*
#define BTN_CALL "A0"
#define BTN_DEST "A1"
*/

/* 디지털 구현이 어려운 경우 적외선 센서는 아날로그로 구현할 수 있도록 한다.  */

typedef unsigned char BYTE; /* 바이트는 부호 없는 char */
typedef BYTE BOOL; /* 부울 자료형의 형식을 바이트로 지정 */

/*
volatile int irDist1 = 0, irDist2 = 0, irDist3 = 0, irDist4 = 0;
volatile int irDoorClose = 0, irDoorOpen = 0;
*/
volatile BOOL arrDest = { FALSE, FALSE, FALSE, FALSE }; /* 목적지 입력 상태 저장용 배열 */
volatile BOOL arrCall = { FALSE, FALSE, FALSE, FALSE, FALSE, FALSE }; 
/* 호출 버튼 상태 저장용 배열, F1U, F2U, F3U, F2D, F3D, F4D 순 */
/*
volatile BOOL arrLEDF = { FALSE, FALSE, FALSE, FALSE, FALSE, FALSE };
volatile BOOL arrLEDB = { FALSE, FALSE, FALSE, FALSE };
*/
volatile BOOL isDoorOpen = FALSE; /* 문 상태 저장 변수 */
volatile BOOL isMoving = FALSE;
volatile BYTE carStat = STOP;
volatile BOOL isEmergency = FALSE;
volatile BYTE currFloor = 1;
volatile BOOL doorStat = CLOSE;
volatile BYTE irStage = 0;

const BYTE fndDigits[] = { 0x03, 0x9F, 0x25, 0x0D, 0x99 }; /* 0~4 */
const int nemaMainSpd = 40; /* 10~1024, 낮을 수록 빠름 */
const int nemaMainSlowSpd = 500;
const int nemaDoorSpd = 500;
/* const int nemaDoorSteps = 200  */

HCMotor nemaMain;
HCMotor nemaDoor;

/* 인터럽트 함수 삽입 */
void isrBtnCall(void) {
    /* int analogVal = analogRead(BTN_CALL); */
    if (digitalRead(BTN1U)) {
        arrCall[0] = TRUE;
        digitalWrite(LEDF1U, HIGH);
    }
    if (digitalRead(BTN2U)) {
        arrCall[1] = TRUE;
        digitalWrite(LEDF2U, HIGH);
    }
    if (digitalRead(BTN2D)) {
        arrCall[2] = TRUE;
        digitalWrite(LEDF2D, HIGH);
    }
    if (digitalRead(BTN3U)) {
        arrCall[3] = TRUE;
        digitalWrite(LEDF3U, HIGH);
    }
    if (digitalRead(BTN3D)) {
        arrCall[4] = TRUE;
        digitalWrite(LEDF3D, HIGH);
    }
    if (digitalRead(BTN4D)) {
        arrCall[5] = TRUE;
        digitalWrite(LEDF4D, HIGH);
    }
}

void isrBtnDest(void) {
    /* int analogVal = analogRead(BTN_DEST); */
    if (digitalRead(BTN1)) {
        arrDest[0] = TRUE;
        digitalWrite(LEDB1, HIGH);
    }
    if (digitalRead(BTN2)) {
        arrDest[1] = TRUE;
        digitalWrite(LEDB2, HIGH);
    }
    if (digitalRead(BTN3)) {
        arrDest[2] = TRUE;
        digitalWrite(LEDB3, HIGH);
    }
    if (digitalRead(BTN4)) {
        arrDest[3] = TRUE;
        digitalWrite(LEDB4, HIGH);
    }
}

void setup() {
    /* 핀 배열에 맞게 pinout 정의 */
    /* 인터럽트 모두 실행 */
    pinMode(BTN_CALL_INTR, INPUT);
    pinMode(BTN_DEST_INTR, INPUT);
    pinMode(HC595_DATA, OUTPUT);
    pinMode(HC595_CLOCK, OUTPUT);
    pinMode(HC595_LATCH, OUTPUT);
    pinMode(LEDB1, OUTPUT);
    pinMode(LEDB2, OUTPUT);
    pinMode(LEDB3, OUTPUT);
    pinMode(LEDB4, OUTPUT);
    pinMode(LEDF1U, OUTPUT);
    pinMode(LEDF2U, OUTPUT);
    pinMode(LEDF2D, OUTPUT);
    pinMode(LEDF3U, OUTPUT);
    pinMode(LEDF3D, OUTPUT);
    pinMode(LEDF4D, OUTPUT);
    pinMode(LEDUP, OUTPUT);
    pinMode(LEDDN, OUTPUT);
    pinMode(BTN1U, INPUT);
    pinMode(BTN2U, INPUT);
    pinMode(BTN2D, INPUT);
    pinMode(BTN3U, INPUT);
    pinMode(BTN3D, INPUT);
    pinMode(BTN4D, INPUT);
    pinMode(BTN1, INPUT);
    pinMode(BTN2, INPUT);
    pinMode(BTN3, INPUT);
    pinMode(BTN4, INPUT);
    pinMode(IR_SNSR_1, INPUT);
    pinMode(IR_SNSR_2, INPUT);
    pinMode(IR_SNSR_3, INPUT);
    pinMode(IR_SNSR_4, INPUT);
    pinMode(IR_SNSR_DOOR_OPEN, INPUT);
    pinMode(IR_SNSR_DOOR_CLOSE, INPUT);
    pinMode(BTN_EMERGENCY, INPUT);
    pinMode(irDoorClose, INPUT);
    pinMode(irDoorOpen, INPUT);
    attachInterrupt(0, isrBtnCall, RISING);
    attachInterrupt(1, isrBtnDest, RISING);
    nemaMain.Init();
    nemaMain.attach(0, STEPPER, NEMA_MAIN_STEP, NEMA_MAIN_DIR);
    nemaMain.Steps(0, CONTINUOUS);
    nemaMain.DutyCycle(0, nemaMainSpd);
    nemaDoor.Init();
    nemaDoor.attach(1, STEPPER, NEMA_DOOR_STEP, NEMA_DOOR_DIR);
    nemaDoor.Steps(1, CONTINUOUS);
    nemaDoor.DutyCycle(1, nemaDoorSpd);
    getFloor();
}

void getFloor(void) { /* 층수 구하고 FND로 표시하는 함수 */
    if (digitalRead(IR_SNSR_1) == 1) {
        currFloor = 1;
        fndDrv(1);
    }
    else if (digitalRead(IR_SNSR_2) == 1) {
        currFloor = 2;
        fndDrv(2);
    }
    else if (digitalRead(IR_SNSR_3) == 1) {
        currFloor = 3;
        fndDrv(3);
    }
    else if (digitalRead(IR_SNSR_4) == 1) {
        currFloor = 4;
        fndDrv(4);
    }
    /* 구현이 가능한 경우 센서 고장 운전을 구현한다.  */
    /* 층수 표시 부분 구현하기 */
    return;
}

void motorDrv(BYTE drvMode) { /* 모터 구동 */
    switch(drvMode) {
        case STOP:
        nemaMain.DutyCycle(0, 0);
        carStat = STOP;
        isMoving = FALSE;
        break;

        case UP:
        nemaMain.Direction(0, REVERSE);
        nemaMain.DutyCycle(0, nemaMainSpd);
        carStat = UP;
        isMoving = TRUE;
        break;

        case DN:
        nemaMain.Direction(0, FORWARD);
        nemaMain.DutyCycle(0, nemaMainSpd);
        carStat = DN;
        isMoving = TRUE;
        break;

        /* 이 아래부턴 실험해야함  */
        case UP_SLOW:
        int spdTmp = nemaMainSpd;
        nemaMain.Direction(0, FORWARD);
        while (spdTmp == nemaMainSlowSpd) {
            nemaMain.DutyCycle(0, spdTmp++);
            delay(1);
        }        
        break;

        case UP_ACCEL:
        int spdTmp = 1000;
        nemaMain.Direction(0, FORWARD);
        while (spdTmp == nemaMainSpd) {
            nemaMain.DutyCycle(0, spdTmp--);
            delay(1);
        }        
        break;

        case DN_SLOW:
        int spdTmp = nemaMainSpd;
        nemaMain.Direction(0, REVERSE);
        while (spdTmp == nemaMainSlowSpd) {
            nemaMain.DutyCycle(0, spdTmp++);
            delay(1);
        }        
        break;

        case DN_ACCEL:
        int spdTmp = 1000;
        nemaMain.Direction(0, REVERSE);
        while (spdTmp == nemaMainSpd) {
            nemaMain.DutyCycle(0, spdTmp--);
            delay(1);
        }        
        break;
    }
}

void chkDoor(void) { /* 문을 확인한다. */
    /* 실물이 제작되면 작성 */
}

void doorDrv(BOOL op) { /* 문 구동용 함수 */
    
    if (doorStat == OPEN && op == CLOSE) {
        /* 문을 닫는다  */
        while (digitalRead(irDoorClose) != 1)
            nemaDoor.Direction(1, REVERSE);
            nemaDoor.DutyCycle(1, nemaDoorSpd);
        doorStat = CLOSE;
        nemaDoor.DutyCycle(1, 0);
        return;
    }
    else if (doorStat == CLOSE && op == OPEN) {
        /* 문을 연다 */
        while (digitalRead(irDoorOPEN) != 1)
            nemaDoor.Direction(1, FORWARD);
            nemaDoor.DutyCycle(1, nemaDoorSpd);
        doorStat = OPEN;
        nemaDoor.DutyCycle(1, 0);
        return;
    }
    else if (doorStat == CLOSE && op == CLOSE) return;
    else if (doorStat == OPEN && op == OPEN) return;
}

BOOL chkDest(void) { /* 현재 층수가 호출된/목적지 층수인지를 확인한다. */
    if (digitalRead(irDist1) == 1 && (arrDest[0] == TRUE || arrCall[0] == TRUE)) {
        preciseMotorCtrl(1);
        arrCall[0] = FALSE;
        arrDest[0] = FALSE;
        return TRUE;
    }
    else if (digitalRead(irDist2) == 1 && (arrDest[1] == TRUE || arrCall[1] == TRUE && carStat == UP || arrCall[3] && carStat == DN)) {
        preciseMotorCtrl(2);
        if (carStat == UP) arrCall[1] = FALSE;
        else if (carStat == DN) arrCall[3] = FALSE;
        arrDest[1] = FALSE;
        return TRUE;
    }
    else if (digitalRead(irDist3) == 1 && (arrDest[2] == TRUE || arrCall[2] && carStat == UP || arrCall[4] && carStat == DN)) {
        preciseMotorCtrl(3);
        if (carStat == UP) arrCall[2] = FALSE;
        else if (carStat == DN) arrCall[4] = FALSE;
        arrDest[2] = FALSE;
        return TRUE;
    }
    else if (digitalRead(irDist4) == 1 && (arrDest[3] == TRUE || arrCall[3] == TRUE)) {
        preciseMotorCtrl(4);
        arrCall[5] = FALSE;
        arrDest[3] = FALSE;
        return TRUE;
    }
    return FALSE;
}

BYTE chkUpDn(void) { /* 상승/하강 결정 함수 */ /* 디버깅 필요 */
    /* 상승? */
    if (carStat != DN) {
        for (int i = (currFloor - 1), i < 4, i++) {
            if (arrDest[i] && i > currFloor) return UP; 
        }
        for (int i = (currFloor - 1), i < 4, i++) {
            if (arrCall[i] && i > currFloor) return UP;
        }
    }
    /* 하강? */
    if (carStat != UP) {
        for (int i = (currFloor - 1), i >= 0, i--) {
            if (arrDest[i] && i < currFloor) return DN;
        }
        for (int i = (currFloor - 1), i > 2, i--) {
            if (arrCall[i] && i < currFloor) return DN;
        }
    }
    /* 이동할 필요 없음 */
    return STOP;
}

void preciseMotorCtrl(BYTE floor) { /* 모터 정밀제어 함수 */
    if (carStat == UP) motorDrv(UP_SLOW);
    else if (carStat == DN) motorDrv(DN_SLOW);

    switch(floor) {
        case 1:
        while (!digitalRead(IR_SNSR_1)) (void) "aoeu"
        break;

        case 2:
        while (!digitalRead(IR_SNSR_2)) (void) "aoeu"
        break;

        case 3:
        while (!digitalRead(IR_SNSR_3)) (void) "aoeu"
        break;

        case 4:
        while (!digitalRead(IR_SNSR_4)) (void) "aoeu"
        break;
    }
    motorDrv(STOP);
    return;
}

void fndDrv(BYTE floorNum) { /* 7seg 구동 */
    
    /* 구동부 */
    digitalWrite(HC595_LATCH, LOW);
    shiftOut(HC595_DATA, HC595_CLOCK, LSBFIRST, fndDigits[floorNum]);
    digitalWrite(HC595_LATCH, HIGH);

} 

void ctrlMove(BYTE mode) { /* mode 0은 정지(버튼만 눌림), 1은 움직이는 중에 제어 */
    if (!mode) goto drivedoor;
    chkDest();
    if (isMoving) }
        getFloor();
        return;
	}
    switch (currFloor) {
        case 1:
        digitalWrite(LEDB1, LOW);
        digitalWrite(LEDF1U, LOW);
        break;

        case 2:
        digitalWrite(LEDB2, LOW);
        if (carStat == UP) digitalWrite(LEDF2U, LOW);
        else if (carStat == DN) digitalWrite(LEDF2D, LOW);
        break;

        case 3:
        digitalWrite(LEDB3, LOW);
        if (carStat == UP) digitalWrite(LEDF3U, LOW);
        else if (carStat == DN) digitalWrite(LEDF3D, LOW);
        break;

        case 4:
        digitalWrite(LEDB4, LOW);
        digitalWrite(LEDF4D, LOW);
        break;
    }
    drivedoor:
    if (doorStat == CLOSE) { 
        doorDrv(OPEN);
        delay(5000);
    }
    doorDrv(CLOSE);
    return;
}

void strtr(void) {
    for (int i = 0; i < 4; i++) { /* 움직여야 하는가?, 목적지 입력 확인 */
        if (arrDest[i]) goto start;
    }
    for (int i = 0; i < 6, i++) { /* 움직여야 하는가?, 호출 입력 확인 */
        if (arrCall[i]) goto start;
    }
    /* 대기한다 */
    digitalWrite(LEDUP, LOW);
    digitalWrite(LEDDN, LOW);
    return;
    
    /* 시동한다  */
    start:
    BYTE updn = chkUpDn();
    if (updn == UP) {
        digitalWrite(LEDUP, HIGH);
        digitalWrite(LEDDN, LOW);
        motorDrv(UP_ACCEL);
        motorDrv(UP);
    } else if (updn == DN) {
        digitalWrite(LEDUP, LOW);
        digitalWrite(LEDDN, HIGH);
        motorDrv(DN_ACCEL);
        motorDrv(DN);
    } else if (updn == STOP)
        ctrlMove(0);
    return;
}

void loop(void) { 
    if (digitalRead(BTN_EMERGENCY)) {
        for (int i = 0; i < 4; i++) {
            arrDest[i] = FALSE;
        }
        for (int i = 0; i < 6; i++) {
            arrCall[i] = FALSE;
        }
        digitalWrite(LEDF1U, LOW);
        digitalWrite(LEDF2U, LOW);
        digitalWrite(LEDF2D, LOW);
        digitalWrite(LEDF3U, LOW);
        digitalWrite(LEDF3D, LOW);
        digitalWrite(LEDF4D, LOW);
        digitalWrite(LEDB1, LOW);
        digitalWrite(LEDB2, LOW);
        digitalWrite(LEDB3, LOW);
        digitalWrite(LEDB4, LOW);
        digitalWrite(LIGHT_EMERGENCY, HIGH);
        arrDest[0] = TRUE;
        detachInterrupt(0);
        while (1) {
            if (isMoving) {
                ctrlMove(1);
            } else {
                strtr();
            }
        }
        attachInterrupt(0, isrBtnCall, RISING);
        digitalWrite(LIGHT_EMERGENCY, LOW);
        return;
    }
    if (isMoving) {
        ctrlMove(1);
        return;
    } else {
        strtr();
        return;
    }
}
