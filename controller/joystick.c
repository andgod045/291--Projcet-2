#include "joystick.h"
#include "board_config.h"
#include "ir_rc5.h"

#define JOYSTICK_ADC_MAX    1023U
#define JOYSTICK_CENTER_ADC 512U

#define JOYSTICK_MAX_OFFSET 512U

typedef struct {
    uint8_t last_raw;
    uint8_t stable;
    uint8_t debounce_counter;
} Debounce_State;

static Debounce_State sw_debounce = {0};

static void Joystick_DelayMs(const uint16_t ms)
{
    for (uint16_t i = 0; i < ms; i++)
    {
        _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT() < (SYSCLK / 2000UL))
        {
        }
    }
}

static uint8_t DebounceButton(Debounce_State *db, const uint8_t raw)
{
    if (raw == db->last_raw)
    {
        if (db->debounce_counter < 3U)
            db->debounce_counter++;
    }
    else
    {
        db->debounce_counter = 0U;
        db->last_raw = raw;
    }

    if (db->debounce_counter >= 3U)
        db->stable = raw;

    return db->stable;
}

static uint16_t ADC_Read(uint8_t channel)
{
    // Select analog input channel
    AD1CHSbits.CH0SA = channel;

    // Start sampling
    AD1CON1bits.SAMP = 1;

    // Wait for sampling time (about 10 microseconds)
    for (volatile int i = 0; i < 100; i++);

    // Start conversion
    AD1CON1bits.SAMP = 0;

    // Wait for conversion to complete
    while (!AD1CON1bits.DONE);

    // Return 10-bit result
    return (uint16_t)ADC1BUF0;
}

Joystick_Direction Joystick_GetDirection(const uint16_t x, const uint16_t y)
{
    const uint8_t x_high = (x > JOYSTICK_THRESHOLD_HIGH) ? 1U : 0U;
    const uint8_t x_low = (x < JOYSTICK_THRESHOLD_LOW) ? 1U : 0U;
    const uint8_t y_high = (y > JOYSTICK_THRESHOLD_HIGH) ? 1U : 0U;
    const uint8_t y_low = (y < JOYSTICK_THRESHOLD_LOW) ? 1U : 0U;
    const uint16_t x_offset = (x > JOYSTICK_CENTER_ADC) ? (x - JOYSTICK_CENTER_ADC) : (JOYSTICK_CENTER_ADC - x);
    const uint16_t y_offset = (y > JOYSTICK_CENTER_ADC) ? (y - JOYSTICK_CENTER_ADC) : (JOYSTICK_CENTER_ADC - y);

    if (!x_high && !x_low && !y_high && !y_low)
        return JOYSTICK_DIR_CENTER;

    if ((x_high || x_low) && (!y_high && !y_low))
        return x_high ? JOYSTICK_DIR_RIGHT : JOYSTICK_DIR_LEFT;

    if ((y_high || y_low) && (!x_high && !x_low))
        return y_high ? JOYSTICK_DIR_UP : JOYSTICK_DIR_DOWN;

    if (y_offset >= x_offset)
        return y_high ? JOYSTICK_DIR_UP : JOYSTICK_DIR_DOWN;

    return x_high ? JOYSTICK_DIR_RIGHT : JOYSTICK_DIR_LEFT;
}

void Joystick_Init(void)
{
    // Configure VRX pin as analog input (AN0/RA0)
    ANSELA_EnableAnalog(JOYSTICK_VRX_MASK_A);
    GPIOA_SetInput(JOYSTICK_VRX_MASK_A);

    // Configure VRY pin as analog input (AN1/RA1)
    ANSELA_EnableAnalog(JOYSTICK_VRY_MASK_A);
    GPIOA_SetInput(JOYSTICK_VRY_MASK_A);

    // Configure SW pin as digital input with pull-up (RB0)
    ANSELB_DisableAnalog(JOYSTICK_SW_MASK_B);
    GPIOB_SetInput(JOYSTICK_SW_MASK_B);
    CNPUBSET = JOYSTICK_SW_MASK_B;  // Enable pull-up

    // Configure ADC
    AD1CON1 = 0;  // Reset
    AD1CON2 = 0;
    AD1CON3 = 0;

    // Configure ADC1
    // FORM<2:0> = 000 (integer 16-bit output)
    // SSRC<2:0> = 111 (internal counter ends sampling and starts conversion)
    AD1CON1bits.FORM = 0;
    AD1CON1bits.SSRC = 7;

    // VCFG<2:0> = 000 (use AVdd and AVss as reference)
    AD1CON2bits.VCFG = 0;

    // ADCS<7:0> = TPB / (2 * TAD) - 1
    // For TAD = 100ns and PBCLK = 40MHz: ADCS = 40MHz / (2 * 10MHz) - 1 = 1
    AD1CON3bits.ADCS = 1;

    // SAMC<4:0> = sampling time in TAD, set to 15 TAD
    AD1CON3bits.SAMC = 15;

    // Turn on ADC
    AD1CON1bits.ON = 1;

    // Initialize debounce state from the actual pin level to avoid startup false edges.
    const uint8_t sw_raw_init = GPIOB_Read(JOYSTICK_SW_MASK_B) ? 0U : 1U;
    sw_debounce.last_raw = sw_raw_init;
    sw_debounce.stable = sw_raw_init;
    sw_debounce.debounce_counter = 0U;

    Joystick_DelayMs(10);
}

static int8_t Joystick_QuantizeAxis(const uint16_t value)
{
    uint16_t offset;
    uint16_t magnitude;

    if (value >= JOYSTICK_CENTER_ADC)
        offset = value - JOYSTICK_CENTER_ADC;
    else
        offset = JOYSTICK_CENTER_ADC - value;

    // Direct center-to-edge scaling without extra ramp thresholds.
    magnitude = ((offset * IR_MOVEMENT_MAGNITUDE_MAX) + (JOYSTICK_MAX_OFFSET / 2U)) / JOYSTICK_MAX_OFFSET;
    if (magnitude > IR_MOVEMENT_MAGNITUDE_MAX)
        magnitude = IR_MOVEMENT_MAGNITUDE_MAX;

    if (value >= JOYSTICK_CENTER_ADC)
        return (int8_t)magnitude;

    return (int8_t)(-(int8_t)magnitude);
}

void Joystick_Read(Joystick_State *state)
{
    static uint8_t last_sw_stable = 0U;
    static uint8_t sw_initialized = 0U;
    uint16_t max_offset;

    // Read analog values
    state->x = ADC_Read(JOYSTICK_VRX_ADC_CHANNEL);
    state->y = JOYSTICK_ADC_MAX - ADC_Read(JOYSTICK_VRY_ADC_CHANNEL);

    // Get direction
    state->direction = (uint8_t)Joystick_GetDirection(state->x, state->y);

    // Compute magnitude from center distance with direct linear scaling.
    const uint16_t x_offset = (state->x > JOYSTICK_CENTER_ADC) ?
                              (state->x - JOYSTICK_CENTER_ADC) :
                              (JOYSTICK_CENTER_ADC - state->x);
    const uint16_t y_offset = (state->y > JOYSTICK_CENTER_ADC) ?
                              (state->y - JOYSTICK_CENTER_ADC) :
                              (JOYSTICK_CENTER_ADC - state->y);
    max_offset = (x_offset > y_offset) ? x_offset : y_offset;

    state->magnitude = (uint8_t)(((max_offset * IR_MOVEMENT_MAGNITUDE_MAX) + (JOYSTICK_MAX_OFFSET / 2U)) / JOYSTICK_MAX_OFFSET);
    if (state->magnitude > IR_MOVEMENT_MAGNITUDE_MAX)
        state->magnitude = IR_MOVEMENT_MAGNITUDE_MAX;

    state->throttle = Joystick_QuantizeAxis(state->y);
    state->turn = Joystick_QuantizeAxis(state->x);


    // Read and debounce switch button (active low with pull-up)
    uint8_t sw_raw = GPIOB_Read(JOYSTICK_SW_MASK_B) ? 0U : 1U;
    state->sw_button = DebounceButton(&sw_debounce, sw_raw);

    // Detect button events
    if (sw_initialized == 0U)
    {
        state->sw_pressed = 0U;
        state->sw_released = 0U;
        last_sw_stable = state->sw_button;
        sw_initialized = 1U;
    }
    else
    {
        state->sw_pressed = (state->sw_button && !last_sw_stable) ? 1U : 0U;
        state->sw_released = (!state->sw_button && last_sw_stable) ? 1U : 0U;
    }

    // Update hold time
    if (state->sw_button)
    {
        if (state->sw_hold_time < 0xFFFFU)
            state->sw_hold_time++;
    }
    else
    {
        state->sw_hold_time = 0;
    }

    last_sw_stable = state->sw_button;
}
