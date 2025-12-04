#include "fsl_device_registers.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include <stdint.h>

// =====================
// External ASM functions
// =====================
extern uint32_t Ultrasonic_measure_echo_cycles(void);
extern void Button_Handle_Asm(uint32_t flags);

// =====================
// Global state (used by C and ASM)
// =====================
volatile uint8_t system_enabled = 0;  // 0 = OFF, 1 = ON
volatile uint8_t sens_mode      = 0;  // 0 = 30cm, 1 = 50cm, 2 = 80cm
volatile uint8_t btn1_lock      = 0;  // lock for external button (PTA25)
volatile uint8_t btn2_lock      = 0;  // lock for onboard button (PTA10)

// =====================
// Pin mapping
// =====================

// External LED on breadboard: D6 = PTC2
#define LED_GPIO        GPIOC
#define LED_PORT        PORTC
#define LED_PIN         2U    // PTC2

// External button on breadboard: D7 = PTA25
#define EXT_BTN_GPIO    GPIOA
#define EXT_BTN_PORT    PORTA
#define EXT_BTN_PIN     25U   // PTA25

// On-board button SW3: PTA10
#define ONB_BTN_GPIO    GPIOA
#define ONB_BTN_PORT    PORTA
#define ONB_BTN_PIN     10U   // PTA10

// Ultrasonic TRIG: D3 = PTC8
#define TRIG_GPIO       GPIOC
#define TRIG_PORT       PORTC
#define TRIG_PIN        8U    // PTC8

// Ultrasonic ECHO: D4 = PTC12 (through divider)
#define ECHO_GPIO       GPIOC
#define ECHO_PORT       PORTC
#define ECHO_PIN        12U   // PTC12

// Buzzer: A1 = PTB6 (active buzzer)
#define BUZZ_GPIO       GPIOB
#define BUZZ_PORT       PORTB
#define BUZZ_PIN        6U    // PTB6

// =====================
// Small delay helpers
// =====================
static void delay_cycles(volatile uint32_t c)
{
    while (c--) {
        __NOP();
    }
}

static void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_cycles(4000);
    }
}

// =====================
// GPIO helpers
// =====================
// Gen-AI is used to make and explain this 
static inline void LED_on(void)   { LED_GPIO->PSOR = (1U << LED_PIN); }
static inline void LED_off(void)  { LED_GPIO->PCOR = (1U << LED_PIN); }

static inline void Buzzer_on(void)  { BUZZ_GPIO->PSOR = (1U << BUZZ_PIN); }
static inline void Buzzer_off(void) { BUZZ_GPIO->PCOR = (1U << BUZZ_PIN); }

// TRIG control
static inline void Trig_high(void) { TRIG_GPIO->PSOR = (1U << TRIG_PIN); }
static inline void Trig_low(void)  { TRIG_GPIO->PCOR = (1U << TRIG_PIN); }

// =====================
// Ultrasonic trigger pulse (~10 µs)
// =====================
static void Ultrasonic_trigger(void)
{
    Trig_low();
    delay_cycles(100);      // small settle

    Trig_high();
    delay_cycles(400);      // ≈10 µs (rough)
    Trig_low();
}

// =====================
// Convert echo loop count to distance in cm (rough)
// =====================
// Gen-AI is used to validate our math and formula. It does take a few trials until we arrive at this
static float Cycles_to_cm(uint32_t cycles)
{
    // one loop ≈ 0.55 µs (empirical)
    float us = (float)cycles * 0.55f;
    // distance (cm) = (time * speed_of_sound) / 2
    // speed_of_sound ≈ 0.0343 cm/µs
    float dist_cm = (us * 0.0343f) / 2.0f;
    return dist_cm;
}

// =====================
// Board / pin init
// =====================
// Gen-AI is involved in making this method by confirming all the ports and other initalizations
static void Board_init(void)
{
    // Enable clocks to PORTA, PORTB, PORTC
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK
               |  SIM_SCGC5_PORTB_MASK
               |  SIM_SCGC5_PORTC_MASK;

    // LED PTC2
    LED_PORT->PCR[LED_PIN] = PORT_PCR_MUX(1);

    // External button PTA25 (falling AND rising edges)
    EXT_BTN_PORT->PCR[EXT_BTN_PIN] =
        PORT_PCR_MUX(1) |
        PORT_PCR_IRQC(0xB);      // 0xB = interrupt on either edge

    // On-board button PTA10 (internal pull-up + either edge)
    ONB_BTN_PORT->PCR[ONB_BTN_PIN] =
        PORT_PCR_MUX(1)               |
        PORT_PCR_PE_MASK | PORT_PCR_PS_MASK |  // pull enable + pull-up
        PORT_PCR_IRQC(0xB);           // either edge

    // TRIG PTC8
    TRIG_PORT->PCR[TRIG_PIN] = PORT_PCR_MUX(1);

    // ECHO PTC12
    ECHO_PORT->PCR[ECHO_PIN] = PORT_PCR_MUX(1);

    // BUZZER PTB6
    BUZZ_PORT->PCR[BUZZ_PIN] = PORT_PCR_MUX(1);

    // Directions
    LED_GPIO->PDDR   |= (1U << LED_PIN);   // LED out
    TRIG_GPIO->PDDR  |= (1U << TRIG_PIN);  // TRIG out
    BUZZ_GPIO->PDDR  |= (1U << BUZZ_PIN);  // BUZZ out

    EXT_BTN_GPIO->PDDR &= ~(1U << EXT_BTN_PIN); // PTA25 input
    ONB_BTN_GPIO->PDDR &= ~(1U << ONB_BTN_PIN); // PTA10 input
    ECHO_GPIO->PDDR    &= ~(1U << ECHO_PIN);    // ECHO input

    // Safe defaults
    LED_off();
    Buzzer_off();
    Trig_low();
}

// =====================
// NVIC / PORTA interrupt init
// =====================
// Gen-AI creates this
static void Buttons_Int_Init(void)
{
    // Clear any pending flags on PORTA
    PORTA->ISFR = 0xFFFFFFFFU;

    // Enable PORTA interrupt in NVIC
    NVIC_ClearPendingIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTA_IRQn);
}

// =====================
// PORTA IRQ handler
// =====================
// Gen-AI is used to create this
void PORTA_IRQHandler(void)
{
    uint32_t flags = PORTA->ISFR;
    PORTA->ISFR = flags;       // write-1-to-clear
    Button_Handle_Asm(flags);  // do the logic in assembly
}

// =====================
// MAIN
// =====================
int main(void)
{
    Board_init();
    Buttons_Int_Init();

    const float THRESH_CM[3] = {30.0f, 50.0f, 80.0f};

    while (1)
    {
        if (!system_enabled)
        {
            LED_off();
            Buzzer_off();
            continue;
        }

        Ultrasonic_trigger();
        uint32_t echo_cycles = Ultrasonic_measure_echo_cycles();
        float distance_cm = Cycles_to_cm(echo_cycles);

        float threshold = THRESH_CM[sens_mode];

        if (distance_cm > 0.0f && distance_cm < threshold)
        {
            // close: fast LED + buzzer
            LED_on();
            Buzzer_on();
            delay_ms(80);

            LED_off();
            Buzzer_off();
            delay_ms(80);
        }
        else
        {
            // far/timeout: slow LED, buzzer off
            LED_on();
            Buzzer_off();
            delay_ms(200);

            LED_off();
            delay_ms(200);
        }
    }
}
