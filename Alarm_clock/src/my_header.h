enum SEGMENTS {
    SEG0 = 1 << 0,
    SEG1 = 1 << 1,
    SEG2 = 1 << 2,
    SEG3 = 1 << 3,
    SEG4 = 1 << 4,
    SEG5 = 1 << 5,
    SEG6 = 1 << 6
};

enum DIGITS {
    DIG0 = SEG0 | SEG1 | SEG2 | SEG3 | SEG4 | SEG5,
    DIG1 = SEG1 | SEG2,
    DIG2 = SEG0 | SEG1 | SEG6 | SEG4 | SEG3,
    DIG3 = SEG0 | SEG1 | SEG2 | SEG3 | SEG6,
    DIG4 = SEG1 | SEG2 | SEG5 | SEG6,
    DIG5 = SEG0 | SEG5 | SEG6 | SEG2 | SEG3,
    DIG6 = DIG5 | SEG4,
    DIG7 = DIG1 | SEG0,
    DIG8 = DIG0 | SEG6,
    DIG9 = DIG5 | SEG1
};

int digits_on[] = {
(1 << 8) | (1 << 9) | (1 << 10),                //0
(1 << 7) | (1 << 9) | (1 << 10),                //1
(1 << 7) | (1 << 8) | (1 << 10),                //2
(1 << 7) | (1 << 8) | (1 << 9),                 //3
(1 << 7) | (1 << 8) | (1 << 9) | (1 << 10)      // All off
};

enum { DELAY_STOP = 1000 };
enum { BUZZER_DURATION = 15000 };
enum INC_TIME_MODES {
    TIMER_MODE,
    BUTTON_MODE,
    ALARM_MODE
};

enum { M_SEC             = 48000000 / 1000                  };  // In System Timer ticks
enum { SYS_TICK_PERIOD   = M_SEC    / 10                    };  // 0.1 ms
enum { SEC_DURATION      = M_SEC    * 200 / SYS_TICK_PERIOD };  // 200 ms (2000 ticks)
enum { DUTY_CYCLE_PERIOD = 8 * 60 };  // 0.4 MIN


void SystemClock_Config(void);
static void volatile delay(volatile uint32_t time);
void inc_time(uint32_t mode);
void set_time_by_button();
void init_button(void);
void init_clock(void);
void init_pwm(void);
void buzzer(void);
void double_click(void);

