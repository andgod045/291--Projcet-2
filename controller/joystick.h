#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <xc.h>
#include <stdint.h>

// Direction thresholds for normalized 10-bit ADC values (0-1023).
#define JOYSTICK_THRESHOLD_LOW   300U
#define JOYSTICK_THRESHOLD_HIGH  723U

// Magnitude remap thresholds as percent of max center-to-edge deflection.
// <= MIN maps to 0, >= MAX maps to IR_MAGNITUDE_MAX, values between are linear.
#ifndef JOYSTICK_MAG_MIN_PERCENT
#define JOYSTICK_MAG_MIN_PERCENT 25U
#endif

#ifndef JOYSTICK_MAG_MAX_PERCENT
#define JOYSTICK_MAG_MAX_PERCENT 90U
#endif

typedef enum {
    JOYSTICK_DIR_CENTER = 0,
    JOYSTICK_DIR_UP,
    JOYSTICK_DIR_DOWN,
    JOYSTICK_DIR_LEFT,
    JOYSTICK_DIR_RIGHT
} Joystick_Direction;

typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t direction;
    uint8_t magnitude;      /* 0-7 normalized from joystick distance */

    uint8_t sw_button;
    uint8_t sw_pressed;
    uint8_t sw_released;
    uint16_t sw_hold_time;
} Joystick_State;

void Joystick_Init(void);
void Joystick_Read(Joystick_State *state);
Joystick_Direction Joystick_GetDirection(uint16_t x, uint16_t y);

#endif // JOYSTICK_H
