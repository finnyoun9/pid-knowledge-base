// STM32F103C8 — PID Motor Speed Control
// PA0=TIM2_CH1 PWM,  PB0/PB1=Direction
// PB6=TIM4_CH1, PB7=TIM4_CH2 (Encoder mode)
// PA9=TX PA10=RX (UART1 115200)

#include <stdint.h>

// ═══ Registers ════════════════════════════════════════════
// RCC
#define RCC_APB1ENR     (*(volatile uint32_t*)(0x4002101C))
#define RCC_APB2ENR     (*(volatile uint32_t*)(0x40021018))

// GPIOA
#define GPIOA_CRL       (*(volatile uint32_t*)(0x40010800))
#define GPIOA_CRH       (*(volatile uint32_t*)(0x40010804))
#define GPIOA_BSRR      (*(volatile uint32_t*)(0x40010810))
#define GPIOA_BRR       (*(volatile uint32_t*)(0x40010814))

// GPIOB
#define GPIOB_CRL       (*(volatile uint32_t*)(0x40010C00))
#define GPIOB_BSRR      (*(volatile uint32_t*)(0x40010C10))
#define GPIOB_BRR       (*(volatile uint32_t*)(0x40010C14))

// TIM2 (PWM)
#define TIM2_CR1        (*(volatile uint32_t*)(0x40000000))
#define TIM2_CCMR1      (*(volatile uint32_t*)(0x40000018))
#define TIM2_CCER       (*(volatile uint32_t*)(0x40000020))
#define TIM2_PSC        (*(volatile uint32_t*)(0x40000028))
#define TIM2_ARR        (*(volatile uint32_t*)(0x4000002C))
#define TIM2_CCR1       (*(volatile uint32_t*)(0x40000034))
#define TIM2_SR         (*(volatile uint32_t*)(0x40000010))
#define TIM2_EGR        (*(volatile uint32_t*)(0x40000014))

// TIM4 (Encoder)
#define TIM4_CR1        (*(volatile uint32_t*)(0x40000800))
#define TIM4_CR2        (*(volatile uint32_t*)(0x40000804))
#define TIM4_SMCR       (*(volatile uint32_t*)(0x40000808))
#define TIM4_CCMR1      (*(volatile uint32_t*)(0x40000818))
#define TIM4_CCER       (*(volatile uint32_t*)(0x40000820))
#define TIM4_PSC        (*(volatile uint32_t*)(0x40000828))
#define TIM4_ARR        (*(volatile uint32_t*)(0x4000082C))
#define TIM4_CNT        (*(volatile uint32_t*)(0x40000824))
#define TIM4_SR         (*(volatile uint32_t*)(0x40000810))
#define TIM4_EGR        (*(volatile uint32_t*)(0x40000814))
#define TIM4_DIER       (*(volatile uint32_t*)(0x4000080C))

// USART1
#define USART1_SR       (*(volatile uint32_t*)(0x40013800))
#define USART1_DR       (*(volatile uint32_t*)(0x40013804))
#define USART1_BRR      (*(volatile uint32_t*)(0x40013808))
#define USART1_CR1      (*(volatile uint32_t*)(0x4001380C))

// NVIC
#define NVIC_ISER0      (*(volatile uint32_t*)(0xE000E100))
#define NVIC_ICER0      (*(volatile uint32_t*)(0xE000E180))

// SysTick
#define STK_CTRL        (*(volatile uint32_t*)(0xE000E010))
#define STK_LOAD        (*(volatile uint32_t*)(0xE000E014))
#define STK_VAL         (*(volatile uint32_t*)(0xE000E018))

// ═══ Vector table ═══════════════════════════════════════════
typedef void (*isr_t)(void);
extern void _stack_top(void);
void reset_handler(void);
void tim4_isr(void);
void fault_handler(void) { while(1){} }

__attribute__((section(".vectors")))
const isr_t vector_table[] = {
    (isr_t)&_stack_top,      // 0: SP
    reset_handler,           // 1: Reset
    fault_handler,           // 2: NMI
    fault_handler,           // 3: HardFault
    0, 0, 0,                 // 4-6
    0, 0, 0, 0,              // 7-10
    0, 0, 0,                 // 11-13
    0,                       // 14: PendSV
    0,                       // 15: SysTick
    0, 0, 0, 0,              // 16-19
    0, 0, 0, 0, 0, 0, 0,     // 20-26
    0,                       // 27: TIM2
    0,                       // 28: TIM3
    tim4_isr,                // 29: TIM4  (IRQ 30)
    0, 0, 0, 0, 0, 0, 0, 0, // 30-37
    0, 0, 0, 0, 0, 0, 0, 0, // 38-45
    0, 0, 0, 0, 0, 0, 0, 0, // 46-53
};

// ═══ UART ═══════════════════════════════════════════════════
static void uart_putc(char c) {
    while(!(USART1_SR & (1<<7))){}  // wait TXE
    USART1_DR = c;
}

static void uart_puts(const char *s) {
    while(*s) uart_putc(*s++);
}

static void uart_putint(int n) {
    char buf[12], *p = buf + 11;
    *p = 0;
    if(n < 0) { uart_putc('-'); n = -n; }
    if(n == 0) { uart_putc('0'); return; }
    while(n > 0) { *--p = '0' + (n % 10); n /= 10; }
    uart_puts(p);
}

// ═══ Delay (busy-wait) ═══════════════════════════════════════
static volatile uint32_t sys_tick = 0;

// 1ms @ 72MHz: 72000 ticks
// We'll use SysTick at 1kHz
// SysTick_Handler is vector 15, but we just poll STK_CTRL

static void delay_ms(uint32_t ms) {
    uint32_t start = sys_tick;
    while((sys_tick - start) < ms);
}

// ═══ PWM Init (TIM2, CH1 on PA0) ════════════════════════════
// 72MHz / 72 prescaler = 1MHz counter
// ARR=10000 => PWM period = 10ms (100Hz), resolution 0.01%
// Actually let's use: PSC=71 → 1MHz, ARR=999 → 1kHz PWM, 0-100% = 0-999
static void pwm_init(void) {
    // Enable TIM2 clock (APB1 bit0)
    RCC_APB1ENR |= (1<<0);
    
    // PA0: alternate function push-pull (AFPP), 50MHz
    // CNF=10 (AFPP), MODE=11 (50MHz) → 0xB
    GPIOA_CRL &= ~(0xF<<0);
    GPIOA_CRL |=  (0xB<<0);

    TIM2_PSC  = 71;      // 72MHz / 72 = 1MHz
    TIM2_ARR  = 999;     // PWM freq = 1MHz / 1000 = 1kHz
    TIM2_CCR1 = 0;       // duty = 0 initially
    TIM2_CCMR1 = (0x6<<4); // CH1 PWM mode 1 (OC1M=110), preload
    TIM2_CCER  |= 1;     // CH1 enable, active high
    TIM2_EGR   |= 1;     // generate update event
    TIM2_CR1   |= 1;     // enable TIM2
}

static void pwm_set(uint16_t duty) {
    if(duty > 999) duty = 999;
    TIM2_CCR1 = duty;
}

// ═══ Motor Direction ═════════════════════════════════════════
// PB0 = IN1, PB1 = IN2
// Forward: PB0=1 PB1=0
// Reverse: PB0=0 PB1=1
// Brake:   PB0=1 PB1=1
// Coast:   PB0=0 PB1=0

static void motor_init(void) {
    // PB0, PB1: push-pull 50MHz
    GPIOB_CRL &= ~(0xFF<<0);
    GPIOB_CRL |=  (0x33<<0);
    // Coast by default
    GPIOB_BRR = (1<<0) | (1<<1);
}

static void motor_forward(void) {
    GPIOB_BSRR = (1<<0);
    GPIOB_BRR  = (1<<1);
}

static void motor_reverse(void) {
    GPIOB_BRR  = (1<<0);
    GPIOB_BSRR = (1<<1);
}

static void motor_brake(void) {
    GPIOB_BSRR = (1<<0) | (1<<1);
}

static void motor_coast(void) {
    GPIOB_BRR = (1<<0) | (1<<1);
}

// ═══ Encoder (TIM4, CH1=PB6, CH2=PB7) ════════════════════════
// TIM4 is a general-purpose timer on APB1
// Encoder mode 3: count on both edges, both channels
// PSC=0 → count every encoder pulse
// We'll read 16-bit counter, reset every period

static volatile int32_t encoder_pulses = 0;
static volatile uint16_t last_cnt = 0;

static void encoder_init(void) {
    // Enable TIM4 clock (APB1 bit2)
    RCC_APB1ENR |= (1<<2);

    // PB6, PB7: input floating (CNF=01)
    GPIOB_CRL &= ~(0xFF<<24);
    GPIOB_CRL |=  (0x44<<24);  // 01 00 01 00

    // Reset TIM4
    TIM4_CR1  = 0;
    TIM4_CNT  = 0;
    TIM4_PSC  = 0;            // no prescaler
    
    // Encoder mode 3: count on both TI1 and TI2 edges
    TIM4_SMCR = (3<<0);       // SMS=011 (encoder mode 3)
    
    // Enable both channels, input capture on both
    TIM4_CCMR1 = (0x3<<0) | (0x3<<8); // IC1F=11, IC2F=11 (max filter)
    TIM4_CCER  = 0;           // CC1P=0, CC2P=0 (normal polarity)

    // Auto-reload: max 16-bit value
    TIM4_ARR = 0xFFFF;
    
    // Enable update interrupt
    TIM4_DIER |= 1;           // UIE (update interrupt enable)

    // Enable TIM4
    TIM4_EGR |= 1;            // generate update
    TIM4_CR1 |= 1;            // enable

    // Enable TIM4 IRQ (IRQ 30 in vector, position 30-16=14 in ISER0)
    NVIC_ISER0 |= (1<<14);
}

// TIM4 overflow ISR: handle counter wrapping
void tim4_isr(void) {
    if(TIM4_SR & 1) {  // update event (overflow/underflow)
        // Read current to determine direction
        int32_t current = (int32_t)(uint16_t)TIM4_CNT;
        int32_t last = (int32_t)(uint16_t)last_cnt;
        int32_t diff = current - last;
        
        // If we overflowed (counter went past 0xFFFF), direction matters
        if(diff > 0x7FFF) diff -= 0x10000;
        if(diff < -0x7FFF) diff += 0x10000;
        
        encoder_pulses += diff;
        last_cnt = (uint16_t)current;
        
        TIM4_SR &= ~1;  // clear update flag
    }
}

// Read encoder speed (pulses per period)
static int32_t encoder_read(void) {
    uint16_t now = (uint16_t)TIM4_CNT;
    int32_t diff = (int32_t)(int16_t)(now - last_cnt);
    
    // Handle wrapping
    if(diff > 0x7FFF) diff -= 0x10000;
    if(diff < -0x7FFF) diff += 0x10000;
    
    // accumulate + reset
    int32_t total = encoder_pulses + diff;
    encoder_pulses = 0;
    last_cnt = now;
    
    return total;
}

// ═══ PID Controller ═══════════════════════════════════════════
typedef struct {
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float last_error;
    float out_min, out_max;   // output limits
} PID;

static void pid_init(PID *pid, float Kp, float Ki, float Kd,
                     float setpoint, float out_min, float out_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0;
    pid->last_error = 0;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

static float pid_update(PID *pid, float measurement, float dt) {
    float error = pid->setpoint - measurement;
    float p_term = pid->Kp * error;
    
    // Integrate (with clamping anti-windup)
    pid->integral += error * dt;
    float i_term = pid->Ki * pid->integral;
    
    // Derivative
    float d_term = pid->Kd * (error - pid->last_error) / dt;
    pid->last_error = error;
    
    float output = p_term + i_term + d_term;
    
    // Clamp output
    if(output > pid->out_max) {
        output = pid->out_max;
        // Anti-windup: back off integral if it's making it worse
        if(i_term > 0) pid->integral -= error * dt;
    }
    if(output < pid->out_min) {
        output = pid->out_min;
        if(i_term < 0) pid->integral -= error * dt;
    }
    
    return output;
}

// ═══ UART Init ══════════════════════════════════════════════
static void uart_init(void) {
    // USART1 on APB2 (bit14)
    RCC_APB2ENR |= (1<<14);
    
    // PA9 TX: AF push-pull 50MHz
    GPIOA_CRH &= ~(0xF<<4);
    GPIOA_CRH |=  (0xB<<4);
    // PA10 RX: input with pull-up
    GPIOA_CRH &= ~(0xF<<8);
    GPIOA_CRH |=  (0x8<<8);
    // Pull-up on PA10
    *(volatile uint32_t*)(0x4001080C) |= (1<<10);
    
    // 115200 @ 72MHz → BRR = 72000000/115200 = 625 = 0x271
    USART1_BRR = 0x271;
    USART1_CR1 = (1<<13)|(1<<3)|(1<<2); // UE, TE, RE
}

// ═══ SysTick Init (1ms) ═══════════════════════════════════
static void systick_init(void) {
    // SysTick at 1ms: 72000000/1000 = 72000
    STK_LOAD = 72000 - 1;
    STK_VAL  = 0;
    STK_CTRL = 7;  // enable, interrupt, use processor clock
}

// ═══ Main ═══════════════════════════════════════════════════
void reset_handler(void) {
    // Enable GPIOA, GPIOB clocks
    RCC_APB2ENR |= (1<<2) | (1<<3);
    
    uart_init();
    systick_init();
    pwm_init();
    motor_init();
    encoder_init();

    // Wait for hardware to stabilize
    delay_ms(100);

    uart_puts("\r\n=== STM32 PID Motor Control ===\r\n");
    uart_puts("Setpoint: 500 pulses/sample period\r\n");
    uart_puts("Kp=2.0 Ki=1.0 Kd=0.0\r\n");
    uart_puts("\r\n");

    // PID setup
    PID pid;
    pid_init(&pid, 2.0f, 1.0f, 0.0f, 500.0f, 0.0f, 999.0f);
    
    uint32_t last_tick = 0;
    uint32_t step = 0;
    
    while(1) {
        uint32_t now = sys_tick;
        float dt = (float)(now - last_tick) / 1000.0f;
        if(dt < 0.02f) continue;  // run at ~50Hz (20ms control loop)
        last_tick = now;
        step++;

        // Change phase every ~5 seconds
        if(step == 250) {  // ~5s at 50Hz
            pid_init(&pid, 2.0f, 3.0f, 0.0f, 500.0f, 0.0f, 999.0f);
            uart_puts("\r\n=== Phase 2: PI Control (Kp=2, Ki=3) ===\r\n");
        }
        if(step == 500) {  // ~10s
            pid_init(&pid, 3.0f, 4.0f, 0.3f, 500.0f, 0.0f, 999.0f);
            uart_puts("\r\n=== Phase 3: PID Fine-tune (Kp=3, Ki=4, Kd=0.3) ===\r\n");
        }
        if(step >= 750) {
            // Reset after ~15s
            step = 0;
            pid_init(&pid, 2.0f, 1.0f, 0.0f, 500.0f, 0.0f, 999.0f);
            uart_puts("\r\n=== Phase 1 (restart): P with Ki=1 ===\r\n");
            delay_ms(500);
        }

        // Read encoder
        int32_t pulses = encoder_read();
        
        // Run PID
        float output = pid_update(&pid, (float)pulses, dt);
        pwm_set((uint16_t)output);
        motor_forward();

        // Print every 10 iterations (5Hz)
        if(step % 10 == 0) {
            uart_puts("SP=");
            uart_putint((int)pid.setpoint);
            uart_puts(" RPM=");
            uart_putint(pulses);
            uart_puts(" PWM=");
            uart_putint((int)output);
            uart_puts(" err=");
            uart_putint((int)(pid.setpoint - pulses));
            uart_puts(" I=");
            uart_putint((int)(pid.integral));
            uart_puts("\r\n");
        }
    }
}
