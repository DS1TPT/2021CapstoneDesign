/* Atmega2560 마이크로 컨트롤러는 8비트이므로 기본 데이터 타입은 최적화를 위해 바이트(unsigned char)로 함. */

/* 
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

/* 헤더 불러오기 */
#include <HCMotor.h>
/* 라이브러리 설명: https://forum.hobbycomponents.com/viewtopic.php?f=58&t=1870 */
/* TB6560 제어 http://scipia.co.kr/cms/blog/225 */

/* 참과 거짓 전처리 */
#define FALSE 0
#define TRUE 1

/* 명령어 정의(전처리) */
#define STOP 0
#define UP 1
#define UP_SLOW 2
#define UP_ACCEL 3
#define DN 4
#define DN_SLOW 5
#define DN_ACCEL 6
#define UP_RECY 8
#define DN_RECY 9
#define CLOSE 0
#define OPEN 1

/* 디지털 핀 번호 정의 */
#define BTN_CALL_INTR 2 /* 호출 버튼 인터럽트 */
#define BTN_DEST_INTR 3 /* 목적지 버튼 인터럽트 */

/* TB6560 드라이버의 CW-, CLK- 단자는 모두 아두이노의 GND에 연결한다. */
#define NEMA_DOOR_DIR 5 /* CW+ */
#define NEMA_DOOR_STEP 6 /* CLK+ */

#define NEMA_MAIN_DIR 9 /* CW+ */
#define NEMA_MAIN_STEP 8 /* CLK+ */

#define HC595_DATA 12 /* FND 구동용 쉬프트 레지스터 */
#define HC595_LATCH 11
#define HC595_CLOCK 10

#define LEDF1U 22 /* 각 층의 호출버튼 LED 핀번호 정의 */
#define LEDF2U 23
#define LEDF2D 24
#define LEDF3U 25
#define LEDF3D 26
#define LEDF4D 27
#define LEDUP 32 /* 운전 방향 표시등 정의 */
#define LEDDN 33

#define IR_SNSR_1 34 /* 카 위치 감지 거리 센서 정의 */
#define IR_SNSR_2 35
#define IR_SNSR_3 36
#define IR_SNSR_4 37

#define BTN1U 40 /* 층별 호출 버튼 정의 */
#define BTN2U 41
#define BTN2D 42
#define BTN3U 43
#define BTN3D 44
#define BTN4D 45
#define BTN1 46 /* 카 목적지 버튼 정의 */
#define BTN2 47
#define BTN3 48
#define BTN4 49
/*
#define IR_SNSR_DOOR_OPEN 48
#define IR_SNSR_DOOR_CLOSE 49
*/
#define BTN_EMERGENCY 50 /* 비상버튼 */
#define LIGHT_EMERGENCY 51 /* 비상운전 표시등 */

/* pinouts (analog) */

/*
#define BTN_CALL "A0"
#define BTN_DEST "A1"
*/

/* 디지털 구현이 어려운 경우 적외선 센서는 아날로그로 구현할 수 있도록 한다.  */

typedef unsigned char BYTE; /* 바이트는 부호 없는 char */
typedef BYTE BOOL; /* 부울 자료형의 형식을 바이트로 지정 */

volatile BOOL arrDest[] = { FALSE, FALSE, FALSE, FALSE }; /* 목적지 입력 상태 저장용 배열 */
volatile BOOL arrCall[] = { FALSE, FALSE, FALSE, FALSE, FALSE, FALSE }; 
/* 호출 버튼 상태 저장용 배열, F1U, F2U, F3U, F2D, F3D, F4D 순 */
volatile BYTE spdStat = STOP; /* 정속/감속/가속/저속복귀 상태 기록 */
volatile BOOL isMoving = FALSE; /* 움직이는가? */
volatile BYTE carStat = STOP; /* 카 운전 상태(방향) */
volatile BYTE currFloor = 1; /* 현재 위치 */
volatile BOOL doorStat = CLOSE; /* 문 상태 변수(필요 없을 시 제거) */
volatile BYTE originalDir = STOP; /* 직전 이동 방향 기록용 변수 */

const BYTE fndDigits[] = { 0x03, 0x9F, 0x25, 0x0D, 0x99 }; /* 0~4 */
const int nemaMainSpd = 40; /* 10~1024, 낮을 수록 빠름 */
const int nemaMainSlowSpd = 500;
const int nemaDoorSpd = 500;
const int nemaDoorSteps = 190; /* 문 모터는 한 바퀴보다 살짝 덜 돌아가게 설정함 */

HCMotor nemaMain;
HCMotor nemaDoor;

void getFloor(void);
void motorDrv(BYTE drvMode);
void doorDrv(BOOL op);
BOOL chkDest(void);
BYTE chkUpDn(void);
void preciseMotorCtrl(BYTE floor);
void fndDrv(BYTE floorNum);

/* 인터럽트 함수 */
void isrBtnCall(void) { /* 호출 버튼 인터럽트 처리 */
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

void isrBtnDest(void) { /* 목적층 버튼 인터럽트 처리 */
    /* int analogVal = analogRead(BTN_DEST); */
    if (digitalRead(BTN1)) {
        arrDest[0] = TRUE;
    }
    if (digitalRead(BTN2)) {
        arrDest[1] = TRUE;
    }
    if (digitalRead(BTN3)) {
        arrDest[2] = TRUE;
    }
    if (digitalRead(BTN4)) {
        arrDest[3] = TRUE;
    }
}

void setup() {
    Serial.begin(9600);
    Serial.println("Serial Comm initialized");
    /* 핀 배열에 맞게 pinout 정의 */
    /* 인터럽트 모두 실행 */
    pinMode(BTN_CALL_INTR, INPUT);
    pinMode(BTN_DEST_INTR, INPUT);
    pinMode(HC595_DATA, OUTPUT);
    pinMode(HC595_CLOCK, OUTPUT);
    pinMode(HC595_LATCH, OUTPUT);
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
    pinMode(BTN_EMERGENCY, INPUT);
    pinMode(LIGHT_EMERGENCY, OUTPUT);
    attachInterrupt(0, isrBtnCall, RISING);
    attachInterrupt(1, isrBtnDest, RISING);
    nemaMain.Init();
    nemaMain.attach(0, STEPPER, NEMA_MAIN_STEP, NEMA_MAIN_DIR);
    nemaMain.Steps(0, CONTINUOUS);
    nemaMain.DutyCycle(0, 0);
    nemaDoor.Init();
    nemaDoor.attach(1, STEPPER, NEMA_DOOR_STEP, NEMA_DOOR_DIR);
    nemaDoor.Steps(1, nemaDoorSteps);
    nemaDoor.DutyCycle(1, 0);
    getFloor();
    Serial.println("Setup complete");
    
    /* 등화류 초기화 */
    digitalWrite(LIGHT_EMERGENCY, HIGH);
    digitalWrite(LEDUP, HIGH);
    digitalWrite(LEDDN, HIGH);
}

void loop(void) { 
    BYTE updn = 0;
    BOOL isDest = FALSE;
    getFloor();
    if (digitalRead(BTN_EMERGENCY)) { /* 비상정지 버튼 입력 */
        Serial.println("[loop] lock btn input");
        nemaMain.DutyCycle(0, 0);
        digitalWrite(LIGHT_EMERGENCY, LOW);
        Serial.println("[loop] In infinite loop(locked)");
        detachInterrupt(0); /* 버튼 잠금 */
        detachInterrupt(1);
        while(digitalRead(BTN_EMERGENCY)) /* 버튼 릴리스 전까지 무한 빈 루프 */
            (void) "aoeu";
        digitalWrite(LIGHT_EMERGENCY, HIGH); /* 버튼 릴리스 뒤 소등 및 모터 재구동 */
        attachInterrupt(0, isrBtnCall, RISING); /* 버튼 잠금 해제 */
        attachInterrupt(1, isrBtnDest, RISING);
        Serial.println("[loop] Infinite loop(lock) ended");
        if (carStat == UP && spdStat == UP) motorDrv(UP);
        else if (carStat == UP && spdStat == UP_ACCEL) motorDrv(UP);
        else if (carStat == UP && spdStat == UP_SLOW) motorDrv(UP_RECY);
        else if (carStat == UP && spdStat == UP_RECY) motorDrv(UP_RECY);
        else if (carStat == DN && spdStat == DN) motorDrv(DN);
        else if (carStat == DN && spdStat == DN_ACCEL) motorDrv(DN);
        else if (carStat == DN && spdStat == DN_SLOW) motorDrv(DN_RECY);
        else if (carStat == DN && spdStat == DN_RECY) motorDrv(DN_RECY);
        return;
    }
    
    if (isMoving) {
        Serial.println("[loop] isMoving is TRUE");
        getFloor();
        if (chkDest()) goto arrival;
        return;

        arrival:
        Serial.println("[loop] Arrival");
        switch (currFloor) {
            case 1:
            digitalWrite(LEDF1U, LOW);
            break;

            case 2:
            if (carStat == UP) digitalWrite(LEDF2U, LOW);
            else if (carStat == DN) digitalWrite(LEDF2D, LOW);
            break;

            case 3:
            if (carStat == UP) digitalWrite(LEDF3U, LOW);
            else if (carStat == DN) digitalWrite(LEDF3D, LOW);
            break;

            case 4:
            digitalWrite(LEDF4D, LOW);
            break;
        }

        driveDoor:
        Serial.println("[loop] Driving door...");
        if (doorStat == CLOSE) { 
            Serial.println("[loop] Opening door...");
            doorDrv(OPEN);
            delay(5000);
            doorDrv(CLOSE);
            delay(5000);
            Serial.println("[loop] Closed door");
        }
        return;

    } else {
        for (int i = 0; i < 4; i++) { /* 움직여야 하는가?, 목적지 입력 확인 */
            if (arrDest[i]) goto start;
        }
        for (int i = 0; i < 6; i++) { /* 움직여야 하는가?, 호출 입력 확인 */
            if (arrCall[i]) goto start;
        }
        Serial.println("[loop] Staying");
        originalDir = STOP;
        digitalWrite(LEDUP, LOW);
        digitalWrite(LEDDN, LOW);
        return;
        
        start:
        Serial.println("[loop] Starting");
        updn = chkUpDn();
        if (updn == UP) {
            Serial.println("[loop] updn is UP");
            digitalWrite(LEDUP, HIGH);
            digitalWrite(LEDDN, LOW);
            motorDrv(UP_ACCEL);
            motorDrv(UP);
        } else if (updn == DN) {
            Serial.println("[loop] updn is DN");
            digitalWrite(LEDUP, LOW);
            digitalWrite(LEDDN, HIGH);
            motorDrv(DN_ACCEL);
            motorDrv(DN);
        } else if (updn == STOP) {
            Serial.println("[loop] updn is STOP");
            if ((arrDest[0] || arrCall[0]) && currFloor == 1) {
                arrDest[0] = FALSE;
                arrCall[0] = FALSE;
            } else if ((arrDest[1] || arrCall[1] || arrCall[3]) && currFloor == 2) {
                arrDest[1] = FALSE;
                arrCall[1] = FALSE;
                arrCall[3] = FALSE;
            } else if ((arrDest[2] || arrCall[2] || arrCall[4]) && currFloor == 3) {
                arrDest[2] = FALSE;
                arrCall[2] = FALSE;
                arrCall[4] = FALSE;
            } else if ((arrDest[3] || arrCall[5]) && currFloor == 4) {
                arrDest[3] = FALSE;
                arrCall[5] = FALSE;
            }
            goto driveDoor;
        }
    }
}

void getFloor(void) { /* 층수 구하고 FND로 표시하는 함수 */
    Serial.println("[getFloor] Execution");
    if (digitalRead(IR_SNSR_1)) {
        Serial.println("[getFloor] Floor is 1");
        currFloor = 1;
        fndDrv(1);
    } else if (digitalRead(IR_SNSR_2)) {
        Serial.println("[getFloor] Floor is 2");
        currFloor = 2;
        fndDrv(2);
    } else if (digitalRead(IR_SNSR_3)) {
        Serial.println("[getFloor] Floor is 3");
        currFloor = 3;
        fndDrv(3);
    } else if (digitalRead(IR_SNSR_4)) {
        Serial.println("[getFloor] Floor is 4");
        currFloor = 4;
        fndDrv(4);
    }
    /* 센서 고장 시의 운전은 구현되지 않았음 */
}

void motorDrv(BYTE drvMode) { /* 모터 구동 */
    Serial.println("[motorDrv] Execution");
    int spdTmp = 0;
    switch(drvMode) {
        case STOP:
        Serial.println("[motorDrv] drvMode is STOP");
        nemaMain.DutyCycle(0, 0);
        carStat = STOP;
        spdStat = STOP;
        isMoving = FALSE;
        break;

        case UP:
        Serial.println("[motorDrv] drvMode is UP");
        nemaMain.Direction(0, REVERSE);
        nemaMain.DutyCycle(0, nemaMainSpd);
        carStat = UP;
        spdStat = UP;
        originalDir = UP;
        isMoving = TRUE;
        break;

        case DN:
        Serial.println("[motorDrv] drvMode is DN");
        nemaMain.Direction(0, FORWARD);
        nemaMain.DutyCycle(0, nemaMainSpd);
        carStat = DN;
        spdStat = DN;
        originalDir = DN;
        isMoving = TRUE;
        break;

        /* 목적층/호출위치에 가까워졌을 때 속도를 줄이고 정밀하게 운전 */
        case UP_SLOW:
        Serial.println("[motorDrv] drvMode is UP_SLOW");
        spdTmp = nemaMainSpd;
        nemaMain.Direction(0, FORWARD);
        while (spdTmp != nemaMainSlowSpd) {
            nemaMain.DutyCycle(0, spdTmp++);
            delay(1);
        }
        spdStat = UP_SLOW;
        break;

        case UP_ACCEL:
        Serial.println("[motorDrv] drvMode is UP_ACCEL");
        spdTmp = 1000;
        nemaMain.Direction(0, FORWARD);
        while (spdTmp != nemaMainSpd) {
            nemaMain.DutyCycle(0, spdTmp--);
            delay(1);
        }        
        spdStat = UP_ACCEL;
        break;

        case DN_SLOW:
        Serial.println("[motorDrv] drvMode is DN_SLOW");
        spdTmp = nemaMainSpd;
        nemaMain.Direction(0, REVERSE);
        while (spdTmp != nemaMainSlowSpd) {
            nemaMain.DutyCycle(0, spdTmp++);
            delay(1);
        }        
        spdStat = DN_SLOW;
        break;

        case DN_ACCEL:
        Serial.println("[motorDrv] drvMode is DN_ACCEL");
        spdTmp = 1000;
        nemaMain.Direction(0, REVERSE);
        while (spdTmp != nemaMainSpd) {
            nemaMain.DutyCycle(0, spdTmp--);
            delay(1);
        }        
        spdStat = DN_ACCEL;
        break;

        /* 저속 복귀 운전 명령 */
        case UP_RECY:
        Serial.println("[motorDrv] drvMode is UP_RECY");
        nemaMain.Direction(0, FORWARD);
        nemaMain.DutyCycle(0, nemaMainSlowSpd);
        spdStat = UP_RECY;
        break;

        case DN_RECY:
        Serial.println("[motorDrv] drvMode is DN_RECY");
        nemaMain.Direction(0, REVERSE);
        nemaMain.DutyCycle(0, nemaMainSlowSpd);
        spdStat = DN_RECY;
        break;
    }
}

void doorDrv(BOOL op) { /* 문 구동용 함수 */
    Serial.println("[doorDrv] Execution");
    if (doorStat == OPEN && op == CLOSE) {
        Serial.println("[doorDrv] Closing");
        /* 문을 닫는다  */
        nemaDoor.Direction(1, REVERSE);
        nemaDoor.Steps(1, nemaDoorSteps);
        nemaDoor.DutyCycle(1, nemaDoorSpd);
        doorStat = CLOSE;
        delay(1000);
        nemaDoor.DutyCycle(1, 0);
        return;
    } else if (doorStat == CLOSE && op == OPEN) {
        Serial.println("[doorDrv] Opening");
        nemaDoor.Direction(1, FORWARD);
        nemaDoor.Steps(1, nemaDoorSteps);
        nemaDoor.DutyCycle(1, nemaDoorSpd);
        doorStat = OPEN;
        delay(1000);
        nemaDoor.DutyCycle(1, 0);
        return;
    }
    Serial.println("[doorDrv] ?Problem occured(doorStat does not match with op)");
    if (doorStat == CLOSE && op == CLOSE) return;
    else if (doorStat == OPEN && op == OPEN) return;
}

BOOL chkDest(void) { /* 현재 층수가 호출된/목적지 층수인지를 확인한다. */
    Serial.println("[chkDest] Execution");
    if (digitalRead(IR_SNSR_1) == 1 && (arrDest[0] == TRUE || arrCall[0] == TRUE)) {
        Serial.println("[chkDest] SNSR 1 INPUT");
        preciseMotorCtrl(1);
        arrCall[0] = FALSE;
        arrDest[0] = FALSE;
        originalDir = STOP;
        return TRUE;
    } else if (digitalRead(IR_SNSR_2) == 1 && (arrDest[1] == TRUE || arrCall[1] == TRUE && carStat == UP || arrCall[3] && carStat == DN)) {
        Serial.println("[chkDest] SNSR 2 INPUT");
        preciseMotorCtrl(2);
        if (originalDir == UP) arrCall[1] = FALSE;
        else if (originalDir == DN) arrCall[3] = FALSE;
        arrDest[1] = FALSE;
        return TRUE;
    } else if (digitalRead(IR_SNSR_3) == 1 && (arrDest[2] == TRUE || arrCall[2] && carStat == UP || arrCall[4] && carStat == DN)) {
        Serial.println("[chkDest] SNSR 3 INPUT");
        preciseMotorCtrl(3);
        if (originalDir == UP) arrCall[2] = FALSE;
        else if (originalDir == DN) arrCall[4] = FALSE;
        arrDest[2] = FALSE;
        return TRUE;
    } else if (digitalRead(IR_SNSR_4) == 1 && (arrDest[3] == TRUE || arrCall[3] == TRUE)) {
        Serial.println("[chkDest] SNSR 4 INPUT");
        preciseMotorCtrl(4);
        arrCall[5] = FALSE;
        arrDest[3] = FALSE;
        originalDir = STOP;
        return TRUE;
    }
    Serial.println("[chkDest] NO SNSR INPUT");
    return FALSE;
}

BYTE chkUpDn(void) { /* 상승/하강 결정 함수 */ /* 디버깅 필요 */
    Serial.println("[chkUpDn] Execution");
    /* 상승? */
    if (carStat != DN && originalDir != DN) {
        for (int i = (currFloor - 1); i < 4; i++) {
            if (arrDest[i] && i > currFloor) {
                Serial.println("[chkUpDn] UP");
                return UP; 
            }
        }
        for (int i = (currFloor - 1); i < 4; i++) {
            if (arrCall[i] && i > currFloor) {
                Serial.println("[chkUpDn] UP");
                return UP;
            }
        }
    }
    /* 하강? */
    if (carStat != UP && originalDir != UP) {
        for (int i = (currFloor - 1); i >= 0; i--) {
            if (arrDest[i] && i < currFloor) {
                Serial.println("[chkUpDn] UP");
                return DN;
            }
        }
        for (int i = (currFloor - 1); i > 2; i--) {
            if (arrCall[i] && i < currFloor) {
                Serial.println("[chkUpDn] UP");
                return DN;
            }
        }
    }
    /* 이동할 필요 없음 */
    return STOP;
}

void preciseMotorCtrl(BYTE floor) { /* 모터 정밀제어 함수 */
    Serial.println("[preciseMotorCtrl] Execution");
    if (carStat == UP) motorDrv(UP_SLOW);
    else if (carStat == DN) motorDrv(DN_SLOW);

    Serial.println("[preciseMotorCtrl] Motor slowed down");
    switch(floor) { /* 센서 입력이 들어올 때까지 무한 빈 루프 */
        case 1:
        while (!digitalRead(IR_SNSR_1)) (void) "aoeu";
        break;

        case 2:
        while (!digitalRead(IR_SNSR_2)) (void) "aoeu";
        break;

        case 3:
        while (!digitalRead(IR_SNSR_3)) (void) "aoeu";
        break;

        case 4:
        while (!digitalRead(IR_SNSR_4)) (void) "aoeu";
        break;
    }
    Serial.println("[preciseMotorCtrl] Stopping motor");
    motorDrv(STOP);
    return;
}

void fndDrv(BYTE floorNum) { /* 7seg 구동 */
    Serial.println("[fndDrv] Execution");
    digitalWrite(HC595_LATCH, LOW);
    shiftOut(HC595_DATA, HC595_CLOCK, LSBFIRST, fndDigits[floorNum]);
    digitalWrite(HC595_LATCH, HIGH);
} 

/*
        if (carStat == DN && (arrDest[0] || arrCall[0]) && currFloor == 1) {
            preciseMotorCtrl(1);
            arrDest[0] = FALSE;
            arrCall[0] = FALSE;
            originalDir = STOP;
            goto arrival;
        } else if (carStat == UP && (arrDest[1] || arrCall[1]) && currFloor == 2) {
            preciseMotorCtrl(2);
            arrDest[1] = FALSE;
            arrCall[1] = FALSE;
            goto arrival;
        } else if (carStat == DN && (arrDest[1] || arrCall[3]) && currFloor == 2) {
            preciseMotorCtrl(2);
            arrDest[1] = FALSE;
            arrCall[3] = FALSE;
            goto arrival;
        } else if (carStat == UP && (arrDest[2] || arrCall[2]) && currFloor == 3) {
            preciseMotorCtrl(3);
            arrDest[2] = FALSE;
            arrCall[2] = FALSE;
            goto arrival;
        } else if (carStat == DN && (arrDest[2] || arrCall[4]) && currFloor == 3) {
            preciseMotorCtrl(3);
            arrDest[2] = FALSE;
            arrCall[4] = FALSE;
            goto arrival;
        } else if (carStat == UP && (arrDest[3] || arrCall[5]) && currFloor == 4) {
            preciseMotorCtrl(4);
            arrDest[3] = FALSE;
            arrCall[5] = FALSE;
            originalDir = STOP;
            goto arrival;
        }
*/
