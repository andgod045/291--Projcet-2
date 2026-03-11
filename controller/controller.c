#include <xc.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "board_config.h"
#include "joystick.h"
#include "lcd.h"
#include "ir_rc5.h"

#pragma config FNOSC = FRCPLL
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLMUL = MUL_20
#pragma config FPLLODIV = DIV_2
#pragma config FWDTEN = ON
#pragma config FPBDIV = DIV_1
#pragma config FSOSCEN = OFF

#define Baud2BRG(desired_baud) ((PBCLK / (16U * (desired_baud))) - 1U)

#define INPUT_SAMPLE_MS               10U
#define SW_SHORT_MIN_SAMPLES          1U
#define SW_SHORT_MAX_SAMPLES          40U
#define SW_LONG_SAMPLES               120U
#define SW_DOUBLE_GAP_MS              500U
#define SW_DOUBLE_GAP_SAMPLES         (SW_DOUBLE_GAP_MS / INPUT_SAMPLE_MS)
#define ROTATE_WINDOW_SAMPLES         10U
#define ROTATE_COOLDOWN_SAMPLES       100U
#define ESTOP_REPEAT_SAMPLES          50U
#define ESTOP_ENTRY_BURST_COUNT       5U
#define IR_TX_REPEATS                 2U
#define IR_CARRIER_HALF_US            13U

#define PREDEFINED_PATH_COUNT         3U
#define CUSTOM_PATH_COUNT             4U
#define TOTAL_PATH_COUNT              (PREDEFINED_PATH_COUNT + CUSTOM_PATH_COUNT)
#define MAX_INTERSECTIONS             16U

typedef enum
{
    MODE_MANUAL = 0,
    MODE_AUTO_SELECT,
    MODE_CUSTOM_EDIT,
    MODE_AUTO_RUN,
    MODE_ESTOP
} ControlMode;

typedef enum
{
    STEP_FORWARD = 0,
    STEP_BACKWARD,
    STEP_LEFT,
    STEP_RIGHT,
    STEP_ROT180,
    STEP_STOP,
    STEP_NA
} PathStep;

typedef struct
{
    PathStep steps[MAX_INTERSECTIONS];
    uint8_t valid;
    uint16_t version;
} CustomPath;

typedef struct
{
    ControlMode mode;
    uint8_t selected_slot;
    uint8_t edit_slot;
    uint8_t edit_index;
    CustomPath custom[CUSTOM_PATH_COUNT];

    Joystick_Direction last_direction;
    uint8_t sw_long_fired;
    uint16_t sw_press_samples;
    uint8_t sw_short_pending;
    uint8_t sw_double_timer;

    int8_t rotate_pending;      // -1 down pending, +1 up pending, 0 none
    uint8_t rotate_age;
    uint8_t rotate_cooldown;
    uint8_t rotate_rearm_center;

    uint8_t estop_repeat_tick;
    uint8_t ir_toggle;
    uint8_t last_magnitude;
    char status_text[CHARS_PER_LINE + 1];
} ControllerContext;

static const PathStep predefined_paths[PREDEFINED_PATH_COUNT][MAX_INTERSECTIONS] = {
    {STEP_FORWARD, STEP_LEFT, STEP_LEFT, STEP_FORWARD, STEP_RIGHT, STEP_LEFT, STEP_RIGHT, STEP_STOP,
     STEP_NA, STEP_NA, STEP_NA, STEP_NA, STEP_NA, STEP_NA, STEP_NA, STEP_NA},
    {STEP_LEFT, STEP_RIGHT, STEP_LEFT, STEP_RIGHT, STEP_FORWARD, STEP_FORWARD, STEP_STOP, STEP_NA,
     STEP_NA, STEP_NA, STEP_NA, STEP_NA, STEP_NA, STEP_NA, STEP_NA, STEP_NA},
    {STEP_RIGHT, STEP_FORWARD, STEP_RIGHT, STEP_LEFT, STEP_RIGHT, STEP_LEFT, STEP_FORWARD, STEP_STOP,
     STEP_NA, STEP_NA, STEP_NA, STEP_NA, STEP_NA, STEP_NA, STEP_NA, STEP_NA}
};

void UART2Configure(int baud_rate);
void wait_1ms(void);
void delayms(int len);

static const char *DirectionString(Joystick_Direction dir);
static const char *ModeString(ControlMode mode);
static char StepChar(PathStep step);
static PathStep SlotStepAt(const ControllerContext *ctx, uint8_t slot, uint8_t index);
static void SetStatus(ControllerContext *ctx, const char *text);
static void UpdateLCD(const ControllerContext *ctx);
static void EmitEvent(const ControllerContext *ctx, const char *event, const Joystick_State *input);
static uint8_t IsCustomSlot(uint8_t slot);
static uint8_t ToCustomIndex(uint8_t slot);
static void CycleSlot(ControllerContext *ctx, int8_t delta);
static void HandleDirectionEvent(ControllerContext *ctx, Joystick_Direction dir, const Joystick_State *input);
static void HandleShortPress(ControllerContext *ctx, Joystick_Direction dir, const Joystick_State *input);
static void HandleDoublePress(ControllerContext *ctx, const Joystick_State *input);
static void HandleLongPress(ControllerContext *ctx, const Joystick_State *input);
static uint8_t DetectRotateGesture(ControllerContext *ctx, Joystick_Direction dir);
static void WaitUs(uint32_t us);
static void IrCarrierBurstUs(uint32_t duration_us);
static void IrMarkUs(uint32_t duration_us);
static void IrSpaceUs(uint32_t duration_us);
static void IrSendCommand(ControllerContext *ctx, IrCommand cmd, uint8_t magnitude);
static IrCommand CommandForSelectedSlot(const ControllerContext *ctx);
static uint8_t ClampProtocolMagnitude(uint8_t magnitude);
static uint8_t MagnitudePercent(uint8_t magnitude);
static void SetManualDirectionStatus(ControllerContext *ctx, const char *label, uint8_t magnitude);

void UART2Configure(const int baud_rate)
{
    U2RXRbits.U2RXR = UART2_RX_PPS_INPUT_SEL;
    RPB9Rbits.RPB9R = UART2_TX_PPS_OUTPUT_SEL;

    U2MODE = 0;
    U2STA = 0x1400;
    U2BRG = Baud2BRG((uint32_t)baud_rate);
    U2MODESET = 0x8000;
}

void wait_1ms(void)
{
    _CP0_SET_COUNT(0);
    while (_CP0_GET_COUNT() < (SYSCLK / (2 * 1000)))
    {
    }
}

void delayms(const int len)
{
    for (int i = 0; i < len; i++)
    {
        wait_1ms();
        WDTCONSET = _WDTCON_WDTCLR_MASK;
    }
}

static void WaitUs(uint32_t us)
{
    const uint32_t target = (SYSCLK / 2000000UL) * us;
    _CP0_SET_COUNT(0);
    while (_CP0_GET_COUNT() < target)
    {
    }
}

static void IrCarrierBurstUs(const uint32_t duration_us)
{
    const uint32_t cycles = (IR_CARRIER_HZ * duration_us) / 1000000UL;
    for (uint32_t i = 0; i < cycles; i++)
    {
        GPIOB_SetHigh(IR_LED_MASK_B);
        WaitUs(IR_CARRIER_HALF_US);
        GPIOB_SetLow(IR_LED_MASK_B);
        WaitUs(IR_CARRIER_HALF_US);
    }
}

static void IrMarkUs(const uint32_t duration_us)
{
    IrCarrierBurstUs(duration_us);
}

static void IrSpaceUs(const uint32_t duration_us)
{
    GPIOB_SetLow(IR_LED_MASK_B);
    WaitUs(duration_us);
}

static void IrSendCommand(ControllerContext *ctx, const IrCommand cmd, uint8_t magnitude)
{
    IrRC5Frame frame;

    if (IrRC5_BuildFrame(cmd, ctx->ir_toggle, magnitude, &frame) == 0)
        return;

    for (uint8_t r = 0; r < IR_TX_REPEATS; r++)
    {
        for (uint8_t i = 0; i < IR_RC5_FRAME_BITS; i++)
        {
            if (frame.bits[i] != 0U)
            {
                IrMarkUs(IR_HALF_BIT_US);
                IrSpaceUs(IR_HALF_BIT_US);
            }
            else
            {
                IrSpaceUs(IR_HALF_BIT_US);
                IrMarkUs(IR_HALF_BIT_US);
            }
        }
        GPIOB_SetLow(IR_LED_MASK_B);
        WaitUs(3000U);
    }

    ctx->ir_toggle ^= 1U;

}

static IrCommand CommandForSelectedSlot(const ControllerContext *ctx)
{
    if (ctx->selected_slot == 0U)
        return IR_CMD_PATH_1;
    if (ctx->selected_slot == 1U)
        return IR_CMD_PATH_2;
    if (ctx->selected_slot == 2U)
        return IR_CMD_PATH_3;

    if (ctx->selected_slot == 3U)
        return IR_CMD_CUSTOM_1;
    if (ctx->selected_slot == 4U)
        return IR_CMD_CUSTOM_2;
    if (ctx->selected_slot == 5U)
        return IR_CMD_CUSTOM_3;
    if (ctx->selected_slot == 6U)
        return IR_CMD_CUSTOM_4;

    return IR_CMD_STOP;
}

static uint8_t ClampProtocolMagnitude(uint8_t magnitude)
{
    if (magnitude > IR_MAGNITUDE_MAX)
        magnitude = IR_MAGNITUDE_MAX;
    return magnitude;
}

static uint8_t MagnitudePercent(uint8_t magnitude)
{
    const uint16_t scaled = ((uint16_t)ClampProtocolMagnitude(magnitude) * 100U) + (IR_MAGNITUDE_MAX / 2U);
    return (uint8_t)(scaled / IR_MAGNITUDE_MAX);
}

static void SetManualDirectionStatus(ControllerContext *ctx, const char *label, uint8_t magnitude)
{
    char msg[CHARS_PER_LINE + 1];
    snprintf(msg, sizeof(msg), "%s %3u%%", label, (unsigned int)MagnitudePercent(magnitude));
    SetStatus(ctx, msg);
}

static const char *DirectionString(const Joystick_Direction dir)
{
    switch (dir)
    {
        case JOYSTICK_DIR_UP: return "UP";
        case JOYSTICK_DIR_DOWN: return "DOWN";
        case JOYSTICK_DIR_LEFT: return "LEFT";
        case JOYSTICK_DIR_RIGHT: return "RIGHT";
        case JOYSTICK_DIR_CENTER: return "CENTER";
        default: return "UNKNOWN";
    }
}

static const char *ModeString(const ControlMode mode)
{
    switch (mode)
    {
        case MODE_MANUAL: return "MANUAL";
        case MODE_AUTO_SELECT: return "AUTO_SEL";
        case MODE_CUSTOM_EDIT: return "CUST_EDIT";
        case MODE_AUTO_RUN: return "AUTO_RUN";
        case MODE_ESTOP: return "E_STOP";
        default: return "UNKNOWN";
    }
}

static char StepChar(const PathStep step)
{
    switch (step)
    {
        case STEP_FORWARD: return 'F';
        case STEP_BACKWARD: return 'B';
        case STEP_LEFT: return 'L';
        case STEP_RIGHT: return 'R';
        case STEP_ROT180: return 'U';
        case STEP_STOP: return 'S';
        default: return '-';
    }
}

static PathStep SlotStepAt(const ControllerContext *ctx, const uint8_t slot, const uint8_t index)
{
    if (index >= MAX_INTERSECTIONS)
        return STEP_NA;

    if (slot < PREDEFINED_PATH_COUNT)
        return predefined_paths[slot][index];

    return ctx->custom[ToCustomIndex(slot)].steps[index];
}

static void SetStatus(ControllerContext *ctx, const char *text)
{
    snprintf(ctx->status_text, sizeof(ctx->status_text), "%s", text == NULL ? "" : text);
}

static void UpdateLCD(const ControllerContext *ctx)
{
    static char last_line1[CHARS_PER_LINE + 1] = "";
    static char last_line2[CHARS_PER_LINE + 1] = "";
    char line1[CHARS_PER_LINE + 1];
    char line2[CHARS_PER_LINE + 1];

    if (ctx->mode == MODE_AUTO_SELECT)
    {
        if (IsCustomSlot(ctx->selected_slot))
            snprintf(line1, sizeof(line1), "AUTO SEL C%u", (unsigned int)(ToCustomIndex(ctx->selected_slot) + 1U));
        else
            snprintf(line1, sizeof(line1), "AUTO SEL P%u", (unsigned int)(ctx->selected_slot + 1U));
    }
    else if (ctx->mode == MODE_CUSTOM_EDIT)
    {
        uint8_t ci = ToCustomIndex(ctx->edit_slot);
        PathStep cur = ctx->custom[ci].steps[ctx->edit_index];
        snprintf(line1, sizeof(line1), "EDIT C%u I%u:%c",
                 (unsigned int)(ci + 1U),
                 (unsigned int)(ctx->edit_index + 1U),
                 StepChar(cur));
    }
    else
    {
        snprintf(line1, sizeof(line1), "Mode:%s", ModeString(ctx->mode));
    }

    snprintf(line2, sizeof(line2), "%s", ctx->status_text);

    if (strncmp(line1, last_line1, CHARS_PER_LINE) != 0 ||
        strncmp(line2, last_line2, CHARS_PER_LINE) != 0)
    {
        LCDprint(line1, 1, 1);
        LCDprint(line2, 2, 1);
        snprintf(last_line1, sizeof(last_line1), "%s", line1);
        snprintf(last_line2, sizeof(last_line2), "%s", line2);
    }
}

static void EmitEvent(const ControllerContext *ctx, const char *event, const Joystick_State *input)
{
    printf("[%s] %s (X:%u Y:%u SW:%u HOLD:%u)\r\n",
           ModeString(ctx->mode),
           event,
           input->x,
           input->y,
           input->sw_button,
           input->sw_hold_time);
}

static uint8_t IsCustomSlot(const uint8_t slot)
{
    return (slot >= PREDEFINED_PATH_COUNT) ? 1U : 0U;
}

static uint8_t ToCustomIndex(const uint8_t slot)
{
    return (uint8_t)(slot - PREDEFINED_PATH_COUNT);
}

static void CycleSlot(ControllerContext *ctx, const int8_t delta)
{
    int16_t idx = (int16_t)ctx->selected_slot + delta;

    if (idx < 0)
        idx = (int16_t)TOTAL_PATH_COUNT - 1;
    else if (idx >= (int16_t)TOTAL_PATH_COUNT)
        idx = 0;

    ctx->selected_slot = (uint8_t)idx;
}

static uint8_t DetectRotateGesture(ControllerContext *ctx, const Joystick_Direction dir)
{
    if (ctx->rotate_cooldown > 0U)
        ctx->rotate_cooldown--;

    // Keep pending UP/DOWN gesture alive across CENTER travel, but still time it out.
    if (ctx->rotate_pending != 0)
    {
        if (ctx->rotate_age < 255U)
            ctx->rotate_age++;

        if (ctx->rotate_age > ROTATE_WINDOW_SAMPLES)
        {
            ctx->rotate_pending = 0;
            ctx->rotate_age = 0U;
        }
    }

    if (dir == JOYSTICK_DIR_CENTER)
    {
        if (ctx->rotate_rearm_center != 0U)
            ctx->rotate_rearm_center = 0U;
        return 0U;
    }

    if (ctx->rotate_rearm_center != 0U || ctx->rotate_cooldown > 0U)
        return 0U;

    if ((ctx->rotate_pending > 0 && dir == JOYSTICK_DIR_DOWN) ||
        (ctx->rotate_pending < 0 && dir == JOYSTICK_DIR_UP))
    {
        ctx->rotate_pending = 0;
        ctx->rotate_age = 0U;
        ctx->rotate_cooldown = ROTATE_COOLDOWN_SAMPLES;
        ctx->rotate_rearm_center = 1U;
        return 1U;
    }

    if (dir == JOYSTICK_DIR_UP)
    {
        ctx->rotate_pending = 1;
        ctx->rotate_age = 0U;
    }
    else if (dir == JOYSTICK_DIR_DOWN)
    {
        ctx->rotate_pending = -1;
        ctx->rotate_age = 0U;
    }

    return 0U;
}

static void HandleDirectionEvent(ControllerContext *ctx, const Joystick_Direction dir, const Joystick_State *input)
{
    char msg[32];

    switch (ctx->mode)
    {
        case MODE_MANUAL:
            if (dir == JOYSTICK_DIR_UP)
            {
                const uint8_t magnitude = ClampProtocolMagnitude(input->magnitude);
                SetManualDirectionStatus(ctx, "FWD", magnitude);
                IrSendCommand(ctx, IR_CMD_FORWARD, magnitude);
            }
            else if (dir == JOYSTICK_DIR_DOWN)
            {
                const uint8_t magnitude = ClampProtocolMagnitude(input->magnitude);
                SetManualDirectionStatus(ctx, "BACK", magnitude);
                IrSendCommand(ctx, IR_CMD_BACKWARD, magnitude);
            }
            else if (dir == JOYSTICK_DIR_LEFT)
            {
                const uint8_t magnitude = ClampProtocolMagnitude(input->magnitude);
                SetManualDirectionStatus(ctx, "LEFT", magnitude);
                IrSendCommand(ctx, IR_CMD_LEFT, magnitude);
            }
            else if (dir == JOYSTICK_DIR_RIGHT)
            {
                const uint8_t magnitude = ClampProtocolMagnitude(input->magnitude);
                SetManualDirectionStatus(ctx, "RIGHT", magnitude);
                IrSendCommand(ctx, IR_CMD_RIGHT, magnitude);
            }
            else
            {
                SetManualDirectionStatus(ctx, "STOP", 0U);
                IrSendCommand(ctx, IR_CMD_STOP, 0U);
            }

            snprintf(msg, sizeof(msg), "DIR %s", DirectionString(dir));
            EmitEvent(ctx, msg, input);
            break;

        case MODE_AUTO_SELECT:
            if (dir == JOYSTICK_DIR_LEFT)
            {
                CycleSlot(ctx, -1);
                SetStatus(ctx, "Select Prev");
                EmitEvent(ctx, "SELECT_PREV", input);
            }
            else if (dir == JOYSTICK_DIR_RIGHT)
            {
                CycleSlot(ctx, 1);
                SetStatus(ctx, "Select Next");
                EmitEvent(ctx, "SELECT_NEXT", input);
            }
            break;

        case MODE_CUSTOM_EDIT:
        {
            const uint8_t cidx = ToCustomIndex(ctx->edit_slot);

            if (dir == JOYSTICK_DIR_UP)
            {
                ctx->custom[cidx].steps[ctx->edit_index] = STEP_FORWARD;
                SetStatus(ctx, "STEP=FWD");
                EmitEvent(ctx, "EDIT_SET_FWD", input);
            }
            else if (dir == JOYSTICK_DIR_DOWN)
            {
                ctx->custom[cidx].steps[ctx->edit_index] = STEP_BACKWARD;
                SetStatus(ctx, "STEP=BACK");
                EmitEvent(ctx, "EDIT_SET_BACK", input);
            }
            else if (dir == JOYSTICK_DIR_LEFT)
            {
                ctx->custom[cidx].steps[ctx->edit_index] = STEP_LEFT;
                SetStatus(ctx, "STEP=LEFT");
                EmitEvent(ctx, "EDIT_SET_LEFT", input);
            }
            else if (dir == JOYSTICK_DIR_RIGHT)
            {
                ctx->custom[cidx].steps[ctx->edit_index] = STEP_RIGHT;
                SetStatus(ctx, "STEP=RIGHT");
                EmitEvent(ctx, "EDIT_SET_RIGHT", input);
            }
            break;
        }

        case MODE_AUTO_RUN:
            break;

        case MODE_ESTOP:
        default:
            break;
    }
}

static void HandleShortPress(ControllerContext *ctx, const Joystick_Direction dir, const Joystick_State *input)
{
    switch (ctx->mode)
    {
        case MODE_MANUAL:
            IrSendCommand(ctx, IR_CMD_STOP, 0);
            ctx->mode = MODE_AUTO_SELECT;
            SetStatus(ctx, "AUTO_SELECT");
            EmitEvent(ctx, "ENTER_AUTO_SELECT", input);
            break;

        case MODE_AUTO_SELECT:
            ctx->mode = MODE_MANUAL;
            SetStatus(ctx, "AUTO_CANCEL");
            EmitEvent(ctx, "AUTO_SELECT_CANCEL", input);
            IrSendCommand(ctx, IR_CMD_AUTO_STOP, 0);
            break;

        case MODE_CUSTOM_EDIT:
            if (ctx->edit_index < (MAX_INTERSECTIONS - 1U))
            {
                ctx->edit_index++;
                SetStatus(ctx, "STEP_SAVED");
                EmitEvent(ctx, "EDIT_SAVE_NEXT", input);
            }
            else
            {
                SetStatus(ctx, "LAST STEP");
                EmitEvent(ctx, "EDIT_AT_LAST", input);
            }
            break;

        case MODE_AUTO_RUN:
            SetStatus(ctx, "AUTO_RUN");
            EmitEvent(ctx, "AUTO_NOOP", input);
            break;

        case MODE_ESTOP:
        default:
            break;
    }

    (void)dir;
}

static void HandleDoublePress(ControllerContext *ctx, const Joystick_State *input)
{
    char msg[32];

    switch (ctx->mode)
    {
        case MODE_MANUAL:
            SetStatus(ctx, "MANUAL");
            EmitEvent(ctx, "DOUBLE_NOOP", input);
            break;

        case MODE_AUTO_SELECT:
            if (IsCustomSlot(ctx->selected_slot))
            {
                const uint8_t cidx = ToCustomIndex(ctx->selected_slot);
                if (ctx->custom[cidx].valid != 0U)
                {
                    ctx->mode = MODE_AUTO_RUN;
                    IrSendCommand(ctx, CommandForSelectedSlot(ctx), 0);
                    IrSendCommand(ctx, IR_CMD_AUTO_START, 0);
                    snprintf(msg, sizeof(msg), "AUTO C%u ST:%c",
                             (unsigned int)(cidx + 1U),
                             StepChar(SlotStepAt(ctx, ctx->selected_slot, 0U)));
                    SetStatus(ctx, msg);
                    EmitEvent(ctx, msg, input);
                }
                else
                {
                    ctx->mode = MODE_CUSTOM_EDIT;
                    ctx->edit_slot = ctx->selected_slot;
                    ctx->edit_index = 0U;
                    SetStatus(ctx, "EDIT_CUSTOM");
                    snprintf(msg, sizeof(msg), "ENTER_EDIT C%u", (unsigned int)(cidx + 1U));
                    EmitEvent(ctx, msg, input);
                }
            }
            else
            {
                ctx->mode = MODE_AUTO_RUN;
                IrSendCommand(ctx, CommandForSelectedSlot(ctx), 0);
                IrSendCommand(ctx, IR_CMD_AUTO_START, 0);
                snprintf(msg, sizeof(msg), "AUTO P%u ST:%c",
                         (unsigned int)(ctx->selected_slot + 1U),
                         StepChar(SlotStepAt(ctx, ctx->selected_slot, 0U)));
                SetStatus(ctx, msg);
                EmitEvent(ctx, msg, input);
            }
            break;

        case MODE_CUSTOM_EDIT:
        {
            const uint8_t cidx = ToCustomIndex(ctx->edit_slot);
            ctx->custom[cidx].steps[ctx->edit_index] = STEP_STOP;
            ctx->custom[cidx].valid = 1U;
            ctx->custom[cidx].version++;
            ctx->mode = MODE_MANUAL;
            SetStatus(ctx, "PATH_SAVE->MAN");
            snprintf(msg, sizeof(msg), "PATH_END C%u V%u",
                     (unsigned int)(cidx + 1U),
                     (unsigned int)ctx->custom[cidx].version);
            EmitEvent(ctx, msg, input);
            break;
        }

        case MODE_AUTO_RUN:
            ctx->mode = MODE_MANUAL;
            SetStatus(ctx, "AUTO_STOP");
            EmitEvent(ctx, "AUTO_STOP", input);
            IrSendCommand(ctx, IR_CMD_AUTO_STOP, 0);
            IrSendCommand(ctx, IR_CMD_STOP, 0);
            break;

        case MODE_ESTOP:
        default:
            break;
    }
}

static void HandleLongPress(ControllerContext *ctx, const Joystick_State *input)
{
    ctx->sw_short_pending = 0U;
    ctx->sw_double_timer = 0U;

    if (ctx->mode != MODE_ESTOP)
    {
        ctx->mode = MODE_ESTOP;
        ctx->estop_repeat_tick = 0U;
        SetStatus(ctx, "EMERG STOP");
        EmitEvent(ctx, "E_STOP", input);
        for (uint8_t i = 0; i < ESTOP_ENTRY_BURST_COUNT; i++)
            IrSendCommand(ctx, IR_CMD_STOP, 0);
        return;
    }

    if (input->direction == JOYSTICK_DIR_CENTER)
    {
        ctx->mode = MODE_MANUAL;
        SetStatus(ctx, "E_STOP_CLEAR");
        EmitEvent(ctx, "E_STOP_CLEAR", input);
        EmitEvent(ctx, "STOP", input);
        IrSendCommand(ctx, IR_CMD_STOP, 0);
    }
}

int main(void)
{
    Joystick_State input_state = {0};
    ControllerContext ctx;

    memset(&ctx, 0, sizeof(ctx));
    ctx.mode = MODE_MANUAL;
    ctx.last_direction = JOYSTICK_DIR_CENTER;
    ctx.last_magnitude = 0U;

    for (uint8_t c = 0; c < CUSTOM_PATH_COUNT; c++)
    {
        for (uint8_t i = 0; i < MAX_INTERSECTIONS; i++)
            ctx.custom[c].steps[i] = STEP_FORWARD;
        ctx.custom[c].valid = 0U;
        ctx.custom[c].version = 0U;
    }

    CFGCON = 0;
    DDPCON = 0;
    INTCONbits.MVEC = 1;

    ANSELB_DisableAnalog(IR_LED_MASK_B);
    GPIOB_SetOutput(IR_LED_MASK_B);
    GPIOB_SetLow(IR_LED_MASK_B);

    UART2Configure(115200);
    Joystick_Init();
    LCD_4BIT();
    SetStatus(&ctx, "BOOT");
    UpdateLCD(&ctx);

    delayms(500);

    printf("\r\n========================================\r\n");
    printf(" ELEC291 Joystick Control Scheme Demo\r\n");
    printf("========================================\r\n");
    printf("Output: UART + LCD + RC5 IR TX on RB6\r\n");
    printf("Modes: MANUAL, AUTO_SELECT, CUSTOM_EDIT, AUTO_RUN, E_STOP\r\n");
    printf("Rotate180: rapid UP->DOWN or DOWN->UP within %u ms\r\n", (unsigned int)(ROTATE_WINDOW_SAMPLES * INPUT_SAMPLE_MS));
    printf("SW short: %u-%u samples, SW long: %u samples\r\n",
           (unsigned int)SW_SHORT_MIN_SAMPLES,
           (unsigned int)SW_SHORT_MAX_SAMPLES,
           (unsigned int)SW_LONG_SAMPLES);
    printf("SW double: second short press within %u ms\r\n", (unsigned int)SW_DOUBLE_GAP_MS);
    printf("========================================\r\n\r\n");

    SetStatus(&ctx, "READY");
    UpdateLCD(&ctx);

    while (1)
    {
        WDTCONSET = _WDTCON_WDTCLR_MASK;
        Joystick_Read(&input_state);

        const Joystick_Direction dir = (Joystick_Direction)input_state.direction;
        const uint8_t magnitude_changed = (input_state.magnitude != ctx.last_magnitude) ? 1U : 0U;

        if (input_state.sw_button)
        {
            if (ctx.sw_press_samples < 0xFFFFU)
                ctx.sw_press_samples++;
        }

        if (dir != ctx.last_direction ||
            (ctx.mode == MODE_MANUAL &&
             dir != JOYSTICK_DIR_CENTER &&
             dir == ctx.last_direction &&
             magnitude_changed != 0U))
        {
            HandleDirectionEvent(&ctx, dir, &input_state);
            ctx.last_direction = dir;
        }

        ctx.last_magnitude = input_state.magnitude;

        if (ctx.mode == MODE_MANUAL && DetectRotateGesture(&ctx, dir) != 0U)
        {
            const uint8_t magnitude = ClampProtocolMagnitude(input_state.magnitude);
            SetManualDirectionStatus(&ctx, "ROT180", magnitude);
            EmitEvent(&ctx, "ROTATE_180", &input_state);
            IrSendCommand(&ctx, IR_CMD_ROTATE_180, magnitude);
        }
        else if (ctx.mode == MODE_CUSTOM_EDIT && DetectRotateGesture(&ctx, dir) != 0U)
        {
            const uint8_t cidx = ToCustomIndex(ctx.edit_slot);
            ctx.custom[cidx].steps[ctx.edit_index] = STEP_ROT180;
            SetStatus(&ctx, "STEP=ROT180");
            EmitEvent(&ctx, "EDIT_SET_ROT180", &input_state);
        }

        if (input_state.sw_button)
        {
            if (ctx.sw_press_samples >= SW_LONG_SAMPLES && ctx.sw_long_fired == 0U)
            {
                ctx.sw_long_fired = 1U;
                HandleLongPress(&ctx, &input_state);
            }
        }

        if (input_state.sw_released)
        {
            if (ctx.sw_long_fired == 0U &&
                ctx.sw_press_samples >= SW_SHORT_MIN_SAMPLES &&
                ctx.sw_press_samples <= SW_SHORT_MAX_SAMPLES)
            {
                if (ctx.sw_short_pending != 0U && ctx.sw_double_timer <= SW_DOUBLE_GAP_SAMPLES)
                {
                    HandleDoublePress(&ctx, &input_state);
                    ctx.sw_short_pending = 0U;
                    ctx.sw_double_timer = 0U;
                }
                else
                {
                    ctx.sw_short_pending = 1U;
                    ctx.sw_double_timer = 0U;
                }
            }
            else
            {
                ctx.sw_short_pending = 0U;
                ctx.sw_double_timer = 0U;
            }

            ctx.sw_long_fired = 0U;
            ctx.sw_press_samples = 0U;
        }

        if (ctx.sw_short_pending != 0U)
        {
            if (ctx.sw_double_timer < 0xFFU)
                ctx.sw_double_timer++;

            if (ctx.sw_double_timer > SW_DOUBLE_GAP_SAMPLES)
            {
                HandleShortPress(&ctx, dir, &input_state);
                ctx.sw_short_pending = 0U;
                ctx.sw_double_timer = 0U;
            }
        }

        if (ctx.mode == MODE_ESTOP)
        {
            if (ctx.estop_repeat_tick == 0U)
            {
                EmitEvent(&ctx, "E_STOP_REPEAT", &input_state);
                SetStatus(&ctx, "EMERG STOP");
                IrSendCommand(&ctx, IR_CMD_STOP, 0);
            }
            ctx.estop_repeat_tick++;
            if (ctx.estop_repeat_tick >= ESTOP_REPEAT_SAMPLES)
                ctx.estop_repeat_tick = 0U;
        }

        UpdateLCD(&ctx);
        delayms(INPUT_SAMPLE_MS);
    }
}
