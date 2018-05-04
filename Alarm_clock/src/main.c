#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "my_header.h"

uint32_t current_digits[4];
uint32_t digits[] = {DIG0, DIG1, DIG2, DIG3, DIG4, DIG5, DIG6, DIG7, DIG8, DIG9};
uint64_t set_alarm_mode;

struct {
    uint32_t tick;
    uint64_t total_tick;
    uint32_t sec;
    uint32_t min;
    uint32_t hours;
} clock;

struct {
    uint32_t min;
    uint32_t hours;
    uint32_t state;
} alarm;

struct {
    uint64_t version;
    uint32_t state;
    uint64_t last_single_click;
} button;

struct {
    uint32_t period;
    uint32_t duty_cycle;
    uint8_t is_on;
    uint8_t is_rising_cycle;
} pwm;
//=================================================================
int main(void) {
    SystemClock_Config();
    init_clock();
    init_button();
    init_pwm();

    while (1) {
  //      if(alarm.state)
 //           buzzer();
    }
}
//=================================================================
void init_button(void) {
    button.state = 0;
    button.version = 0;
    button.last_single_click = 0;

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_NO);

    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableFallingTrig_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_0);

    NVIC_EnableIRQ(EXTI0_1_IRQn);
    NVIC_SetPriority(EXTI0_1_IRQn, 0);
}
//=================================================================
void init_clock(void) {
    clock.tick = 0;
    clock.total_tick = 0;
    clock.sec = 0;
    clock.min = 0;
    clock.hours = 0;
    set_alarm_mode = 0;
    alarm.min = 0;
    alarm.hours = 12;

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);

    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_4, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_9, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_11, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_12, LL_GPIO_MODE_OUTPUT);
}
//=================================================================
void init_pwm(void) {
    pwm.period = pwm.duty_cycle = 50;
    pwm.is_on = pwm.is_rising_cycle = 0;
}
//=================================================================
/**
  * System Clock Configuration
  * The system Clock is configured as follow :
  *    System Clock source            = PLL (HSI/2)
  *    SYSCLK(Hz)                     = 48000000
  *    HCLK(Hz)                       = 48000000
  *    AHB Prescaler                  = 1
  *    APB1 Prescaler                 = 1
  *    HSI Frequency(Hz)              = 8000000
  *    PLLMUL                         = 12
  *    Flash Latency(WS)              = 1
  */

void SystemClock_Config() {
        /* Set FLASH latency */
        LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

        /* Enable HSI and wait for activation*/
        LL_RCC_HSI_Enable();
        while (LL_RCC_HSI_IsReady() != 1);

        /* Main PLL configuration and activation */
        LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2,
                                    LL_RCC_PLL_MUL_12);

        LL_RCC_PLL_Enable();
        while (LL_RCC_PLL_IsReady() != 1);

        /* Sysclk activation on the main PLL */
        LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
        LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
        while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

        /* Set APB1 prescaler */
        LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

        /* Set systick to 0.1 ms */
        SysTick_Config(SYS_TICK_PERIOD);

        /* Update CMSIS variable (which can be updated also
         * through SystemCoreClockUpdate function) */
        SystemCoreClock = 168000000;
}
//=================================================================
void NMI_Handler(void) {}

void HardFault_Handler(void) {
        while (1);
}

void SVC_Handler(void) {}

void PendSV_Handler(void) {}
//=================================================================
void EXTI0_1_IRQHandler(void) {
    if(clock.total_tick - button.version < 30) {                         // Check chattering
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
        return;
    }
    if(button.state == 0) {                                             // Press
        button.version = clock.total_tick;
        button.state = 1;
    }
    else {                                                              // Press out
        if(alarm.state)                                                 // Shut down buzzer
            alarm.state = 0;
        else if(clock.total_tick - button.last_single_click < 2000)      // 2 fast clicks
            double_click();
        else if(clock.total_tick - button.version > 15000) {             // Long press
            if(set_alarm_mode == 0)
                set_alarm_mode = 1;
            else
                set_alarm_mode = 0;
        }
        else {                                                          // Single click
            button.last_single_click = clock.total_tick;
            if(set_alarm_mode)
                inc_time(ALARM_MODE);
            else
                inc_time(BUTTON_MODE);
        }
        button.version = clock.total_tick;
        button.state = 0;
    }
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
}
//=================================================================
void SysTick_Handler(void) {
    clock.tick++;
    clock.total_tick++;
    if (clock.tick == SEC_DURATION)
        inc_time(TIMER_MODE);

    static int cur_dig_num = 0;
    if(set_alarm_mode) {
        current_digits[0] = alarm.hours / 10;
        current_digits[1] = alarm.hours % 10;
        current_digits[2] = alarm.min / 10;
        current_digits[3] = alarm.min % 10;
    }
    else {
        current_digits[0] = clock.hours / 10;
        current_digits[1] = clock.hours % 10;
        current_digits[2] = clock.min / 10;
        current_digits[3] = clock.min % 10;
    }

    LL_GPIO_WriteOutputPort(GPIOC, digits_on[4]);      // Shut down all digits
    if(pwm.is_on) {
        if(clock.total_tick % pwm.period < pwm.duty_cycle)
            LL_GPIO_WriteOutputPort(GPIOC, digits[current_digits[cur_dig_num]] | digits_on[cur_dig_num]);
    }
    else
        LL_GPIO_WriteOutputPort(GPIOC, digits[current_digits[cur_dig_num]] | digits_on[cur_dig_num]);

    if(set_alarm_mode)                                 // Dots on
        LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_11);
    else
        LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_11);
    cur_dig_num++;
    if(cur_dig_num == 4)
        cur_dig_num = 0;
    if(!(pwm.is_on && (clock.total_tick % DUTY_CYCLE_PERIOD == 0)))
        return;

    if(pwm.is_rising_cycle) {
        pwm.duty_cycle++;
        if(pwm.duty_cycle == pwm.period) {
            pwm.is_rising_cycle = 0;
            pwm.is_on = 0;
        }
    } else {
        pwm.duty_cycle--;
        if(pwm.duty_cycle == 0)
            pwm.is_rising_cycle = 1;
    }
}
//=================================================================
static void volatile delay(volatile uint32_t time) {
    volatile uint32_t counter = DELAY_STOP;
    while(time != 0) {
        while(counter) counter--;
        time--;
        counter = DELAY_STOP;
    }
}
//=================================================================
void inc_time(uint32_t mode) {
    switch(mode) {
        case TIMER_MODE:
            clock.tick = 0;
            clock.sec++;
            if(clock.sec == 60) {
                clock.sec = 0;
                clock.min++;
                if(clock.min == 60) {
                    clock.hours++;
                    if(clock.hours == 24)
                        clock.hours = 0;
                    clock.min = 0;
                }
                if(clock.hours == alarm.hours && clock.min == alarm.min && !alarm.state)
                    alarm.state = 1;
            }
            if(clock.sec == 48) {
                pwm.is_on = 1;
            }
            break;
        case BUTTON_MODE:
            clock.tick = 0;
            clock.sec = 0;
            clock.min++;
            if(clock.min == 60)
                clock.min = 0;
            break;
       case ALARM_MODE:
            alarm.min++;
            if(alarm.min == 60)
                alarm.min = 0;
            break;
        default:
            break;
    }
}
//=================================================================
void buzzer(void) {
    while(alarm.state) {
        if(clock.total_tick % 6)
            LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_12);
    }
}
//=================================================================
#define DOUBLE_CLICK(mode)              \
    do {                                \
        mode.hours++;                   \
        if(mode.hours == 24)            \
            mode.hours = 0;             \
        if(mode.min == 0)               \
            mode.min = 59;              \
        else                            \
            mode.min--;                 \
    } while(0)
/* min-- because min was incremented by previous click */
void double_click(void) {
    if(set_alarm_mode) {
        DOUBLE_CLICK(alarm);
    }
    else {
        DOUBLE_CLICK(clock);
    }
}
#undef DOUBLE_CLICK
//=================================================================
void set_time_by_button() {
    if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)) {
        __disable_irq();
        delay(10);
        if(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0))
            inc_time(BUTTON_MODE);
        while(LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0));
        __enable_irq();
    }
}






