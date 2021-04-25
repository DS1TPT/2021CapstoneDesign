/* Atmega2560 마이크로 컨트롤러는 8비트이므로 기본 데이터 타입은 최적화를 위해 바이트(unsigned char)로 함. */

/* 현재 모든 주석과 코드는 모두 실제 하드웨어가 없는 상태에서 짜여 있으므로 참고용으로만 사용할 것 */

/*
아두이노 메가 핀 번호
인터럽트 핀(순서대로): 2, 3, 21, 20, 19, 18
PWM 핀: 2~13, 44~46
디지털 핀: 0 ~ 53
아날로그 입력: A0~A15
I2C 통신 핀: 20(SCA), 21(SCL)
직렬통신 핀: 0, 1, 14~19
*/

/* 코드 시작 부분 */

/* include */
#include <Stepper.h> /* 28BYJ-48, ULN2003 제어 */
#include <HCMotor.h> /* TB6560 제어 http://scipia.co.kr/cms/blog/225 */

#define FALSE 0
#define TRUE 1

/* cmd */
#define STOP 0
#define UP_SLOWDN 1
#define UP_ACCEL 2
#define DN_START 3
#define DN_SLOWDN 4

/* start */
#define UP 10
#define DN 11

/* pinouts(digital) */
#define BTN_CALL_INTR 2 /* 호출 버튼 인터럽트 */
#define BTN_DEST_INTR 3 /* 목적지 버튼 인터럽트 */

#define HC595_DATA 5 /* TB6560 */
#define HC595_LATCH 6
#define HC595_CLOCK 7

#define NEMA_DIR 8 /* CW+ */
#define NEMA_STEP 9 /* CLK+ */
#define NEMA_ENABLE 10

#define LEDF1 22
#define LEDF2 23
#define LEDF3 24
#define LEDF4 25
#define LEDB1 26
#define LEDB2 27
#define LEDB3 28
#define LEDB4 29
#define LEDUP 30
#define LEDDN 31

#define BYJ11 32
#define BYJ12 33
#define BYJ13 34
#define BYJ14 35
#define BYJ21 36
#define BYJ22 37
#define BYJ23 38
#define BYJ24 39
#define BYJ31 40
#define BYJ32 41
#define BYJ33 42
#define BYJ34 43
#define BYJ41 44
#define BYJ42 45
#define BYJ43 46
#define BYJ44 47

/* pinouts (analog) */

#define BTN_CALL 0
#define BTN_DEST 1
#define IR_SNSR_1 2
#define IR_SNSR_2 3
#define IR_SNSR_3 4
#define IR_SNSR_4 5

/*
#define BTN1U 32
#define BTN2U 33
#define BTN2D 34
#define BTN3U 35
#define BTN3D 36
#define BTN4D 37
#define BTN1 38
#define BTN2 39
#define BTN3 40
#define BTN4 41
*/

typedef unsigned char BYTE; /* 바이트는 부호 없는 char */
typedef BYTE BOOL; /* 부울 자료형의 형식을 바이트로 지정 */

volatile int irDist1 = 0, irDist2 = 0, irDist3 = 0, irDist4 = 0; /* 적외선 근접센서 거리 저장용 변수*/
volatile BYTE arrDest = { 0, 0, 0, 0 }; /* 목적지 입력 상태 저장용 배열 */
volatile BYTE arrButton = { 0, 0, 0, 0, 0, 0, 0, 0 }; /* 호출 버튼 상태 저장용 배열 */
volatile BOOL doorStat = FALSE; /* 문 상태 저장 변수 */
volatile BYTE carStat = STOP;
const BYTE fndDidits[] = { 0x03, 0x9F, 0x25, 0x0D, 0x99 }; /* 0~4 */
const int byjSteps = 2048; /* 2048 = 1 바퀴 */

/* 인터럽트 함수 삽입 */

void setup() {
    /* 핀 배열에 맞게 pinout 정의 */
    /* 인터럽트 모두 실행 */
}

void irGetDist(void) { /* 적외선 근접센서에서 거리를 구하여 irDistX 변수에 저장한다.*/
    irDist1 = analogRead(적외선 센서1);
    irDist2 = analogRead(적외선 센서2);
    irDist3 = analogRead(적외선 센서3);
    irDist4 = analogRead(적외선 센서4);
}

void motorDrv(byte drvMode) { /* 모터 구동 */
    /* 모터 드라이버가 구해지면 작성하도록 한다. */
}

void chkDoor(void) { /* 문을 확인한다. */

}

void doorDrv(void) { /* 문 구동용 함수 */

}

void chkStatus(void) {
    /* 모터 드라이버 작동 전에 실행되는 함수이다. */
    /* 문이 열리지 않았는지, 목적 층수인지, 버튼이 눌렸는지, 목적지가 입력되지 않았는지 등을 확인한다. */

}

BYTE chkDest(void) { /* 현재 층수가 목적지 층수인지를 확인한다. */

}

void fndDrv(void) { /* 7seg 구동 */
    
} 

void loop(void) { 
    /* 여기에 작동 순서를 기록한다 */
    if(mv) {
        irGetDist();
        BOOL isDest = chkDest();
        if (isDest) { /* 목적지인 경우 */
            switch(carStat) {
                case STOP: /* 멈춘 경우... */
                doorDrv(); /* 문 구동 로직을 제대로 짠 다음 수정한다 */
                break;
                case UP: /* 올라가는 경우 */
                motorDrv(UP_SLOWDN);
                break;
                case DN: /* 내려가는 경우 */
                motorDrv(DN_SLOWDN);
                break;
            }
        } else {
            /* 목적지가 아닐 경우 할 것 */
        }
    }
}
