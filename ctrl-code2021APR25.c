/* Atmega2560 마이크로 컨트롤러는 8비트이므로 기본 데이터 타입은 최적화를 위해 바이트(unsigned char)로 함. */

/* 현재 모든 주석과 코드는 모두 실제 하드웨어가 없는 상태에서 짜여 있으므로 참고용으로만 사용할 것 */

/*
적외선 근접 센서의 값에 들어갈 수의 크기를 고려하여 데이터 타입을 정한다. 기본은 int, 필요시 늘리거나(long) 줄일 수 있다(byte)
처리 순서: 모든 장치를 초기화 → 적외선 센서 동작 → 목적 층수인지 확인 → 모터를 계속 돌릴지 감속할지 결정 → 모터구동 → 무한반복
인터럽트: 1. 정해진 층수의 적외선 센서에 도달하였을 때 인터럽트를 발생시켜 정해진 곳에 멈추게 한다(미확정)
         2. 버튼이 눌린 경우에는 무조건 인터럽트를 발생한다.
         3. 이외의 필요한 경우에만 인터럽트를 만든다. 인터럽트는 최대한 적은 수로 한다.
*/

#define FALSE 0
#define TRUE 1

#define STOP 0
#define UP_SLOWDN 1
#define UP_ACCEL 2
#define DN_START 3
#define DN_SLOWDN 4

#define UP 10
#define DN 11

typedef unsigned char byte; /* 바이트는 부호 없는 char */
typedef byte BOOL; /* 부울 자료형의 형식을 바이트로 지정 */

/* include */

int irDist1 = 0, irDist2 = 0, irDist3 = 0, irDist4 = 0; /* 적외선 근접센서 거리 저장용 변수*/
byte arrDest = { 0, 0, 0, 0 }; /* 목적지 입력 상태 저장용 배열 */
BOOL doorStat = FALSE; /* 문 상태 저장 변수 */
byte carStat = STOP;

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

byte chkDest(void) { /* 현재 층수가 목적지 층수인지를 확인한다. */

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
