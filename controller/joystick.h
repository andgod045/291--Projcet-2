#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <xc.h>
#include <stdint.h>

// Direction thresholds for normalized 10-bit ADC values (0-1023).
// Values inside [LOW, HIGH] are treated as centered on that axis.
#define JOYSTICK_THRESHOLD_LOW   384U
#define JOYSTICK_THRESHOLD_HIGH  640U

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
    uint8_t magnitude;      /* 0-7 linearly quantized from joystick distance */
    int8_t throttle;        /* -7..7 from Y axis (forward/back) */
    int8_t turn;            /* -7..7 from X axis (left/right) */

    uint8_t sw_button;
    uint8_t sw_pressed;
    uint8_t sw_released;
    uint16_t sw_hold_time;
} Joystick_State;

void Joystick_Init(void);
void Joystick_Read(Joystick_State *state);
Joystick_Direction Joystick_GetDirection(uint16_t x, uint16_t y);

#endif // JOYSTICK_H
