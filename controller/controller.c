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

#define INPUT_SAMPLE_MS               10U
#define MANUAL_REPEAT_MS              100U
#define MANUAL_REPEAT_SAMPLES         (MANUAL_REPEAT_MS / INPUT_SAMPLE_MS)
#define MODE_BROADCAST_MS             100U
#define MODE_BROADCAST_SAMPLES        (MODE_BROADCAST_MS / INPUT_SAMPLE_MS)
#define SW_SHORT_MIN_SAMPLES          1U
#define SW_SHORT_MAX_SAMPLES          40U
#define SW_LONG_SAMPLES               120U
#define SW_DOUBLE_GAP_MS              500U
#define SW_DOUBLE_GAP_SAMPLES         (SW_DOUBLE_GAP_MS / INPUT_SAMPLE_MS)
#define ROTATE_WINDOW_SAMPLES         10U
#define ROTATE_COOLDOWN_SAMPLES       100U
#define EDIT_SAVE_FEEDBACK_MS         500U
#define EDIT_SAVE_BROADCAST_MS        100U
#define EDIT_SAVE_BROADCAST_COUNT     (EDIT_SAVE_FEEDBACK_MS / EDIT_SAVE_BROADCAST_MS)
#define ESTOP_REPEAT_MS               100U
#define ESTOP_REPEAT_SAMPLES          (ESTOP_REPEAT_MS / INPUT_SAMPLE_MS)
#define ESTOP_ENTRY_BURST_COUNT       5U
#define IR_TX_REPEATS                 2U
#define IR_CARRIER_HALF_US            13U

#define DEFAULT_PATH_COUNT            3U
#define PATH_SLOT_COUNT               16U
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
    ControlMode mode;
    uint8_t selected_slot;
    uint8_t edit_slot;
    uint8_t edit_index;
    uint8_t path_is_saved[PATH_SLOT_COUNT];

    Joystick_Direction last_direction;
    uint8_t sw_long_fired;
    uint16_t sw_press_samples;
    uint8_t sw_short_pending;
    uint8_t sw_double_timer;

    int8_t rotate_pending;      // -1 down pending, +1 up pending, 0 none
    uint8_t rotate_age;
    uint8_t rotate_cooldown;
    uint8_t rotate_rearm_center;
    uint8_t manual_rotate_lock;

    uint8_t manual_repeat_tick;
    uint8_t mode_broadcast_tick;
    uint8_t estop_repeat_tick;
    uint8_t ir_toggle;
    int8_t last_throttle;
    int8_t last_turn;
    PathStep edit_selected_step;
    char status_text[CHARS_PER_LINE + 1];
} ControllerContext;

void wait_1ms(void);
void delayms(int len);

static const char *ModeString(ControlMode mode);
static char StepChar(PathStep step);
static void SetStatus(ControllerContext *ctx, const char *text);
static void UpdateLCD(const ControllerContext *ctx);
static void EmitEvent(const ControllerContext *ctx, const char *event, const Joystick_State *input);
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
static void IrSendPayload(ControllerContext *ctx, uint8_t data_type, uint8_t data);
static void IrSendMisc(ControllerContext *ctx, uint8_t misc_code);
static void IrSendMovement(ControllerContext *ctx, int8_t x, int8_t y);
static uint8_t MiscCodeForSelectedSlot(const ControllerContext *ctx);
static uint8_t MiscCodeForStep(PathStep step);
static int8_t ClampProtocolAxis(int8_t axis);
static void SetManualMixStatus(ControllerContext *ctx, int8_t throttle, int8_t turn);
static uint8_t SendManualDriveMix(ControllerContext *ctx, int8_t throttle, int8_t turn);
static void BroadcastCurrentModeState(ControllerContext *ctx);
static void ResetRotateTracker(ControllerContext *ctx);
static void ResetSwitchClickState(ControllerContext *ctx);
static void EnterModeAutoSelect(ControllerContext *ctx, const Joystick_State *input);
static void EnterModeAutoRun(ControllerContext *ctx, const Joystick_State *input);
static void EnterModeCustomEdit(ControllerContext *ctx, const Joystick_State *input);
static void SaveEditedPath(ControllerContext *ctx, const Joystick_State *input);
static void EnterModeEstop(ControllerContext *ctx, const Joystick_State *input);
static void ClearModeEstop(ControllerContext *ctx, const Joystick_State *input);
static void TickModeBroadcast(ControllerContext *ctx);

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

static void IrSendPayload(ControllerContext *ctx, const uint8_t data_type, const uint8_t data)
{
    IrRC5Frame frame;

    if (IrRC5_BuildFrame(ctx->ir_toggle, data_type, data, &frame) == 0)
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

static void IrSendMisc(ControllerContext *ctx, const uint8_t misc_code)
{
    IrSendPayload(ctx, IR_DATA_MISC, misc_code);
}

static void IrSendMovement(ControllerContext *ctx, const int8_t x, const int8_t y)
{
    uint8_t data;

    if (IrRC5_EncodeMovement(x, y, &data) == 0)
        return;

    IrSendPayload(ctx, IR_DATA_MOVEMENT, data);
}

static uint8_t MiscCodeForSelectedSlot(const ControllerContext *ctx)
{
    if (ctx->selected_slot >= PATH_SLOT_COUNT)
        return IR_MISC_STOP;

    return (uint8_t)(IR_MISC_SELECT_PATH_BASE + ctx->selected_slot);
}

static uint8_t MiscCodeForStep(const PathStep step)
{
    switch (step)
    {
        case STEP_FORWARD: return IR_MISC_FORWARD;
        case STEP_BACKWARD: return IR_MISC_BACKWARD;
        case STEP_LEFT: return IR_MISC_LEFT;
        case STEP_RIGHT: return IR_MISC_RIGHT;
        case STEP_ROT180: return IR_MISC_ROTATE_180;
        case STEP_STOP: return IR_MISC_STOP;
        case STEP_NA:
        default: return IR_MISC_STOP;
    }
}

static int8_t ClampProtocolAxis(int8_t axis)
{
    if (axis > (int8_t)IR_MOVEMENT_AXIS_MAX)
        axis = (int8_t)IR_MOVEMENT_AXIS_MAX;
    if (axis < (int8_t)IR_MOVEMENT_AXIS_MIN)
        axis = (int8_t)IR_MOVEMENT_AXIS_MIN;
    return axis;
}

static void SetManualMixStatus(ControllerContext *ctx, const int8_t throttle, const int8_t turn)
{
    char msg[CHARS_PER_LINE + 1];
    snprintf(msg, sizeof(msg), "DRV Y%+d X%+d", (int)throttle, (int)turn);
    SetStatus(ctx, msg);
}

static uint8_t SendManualDriveMix(ControllerContext *ctx, int8_t throttle, int8_t turn)
{
    throttle = ClampProtocolAxis(throttle);
    turn = ClampProtocolAxis(turn);

    IrSendMovement(ctx, turn, throttle);
    return 1U;
}

static void BroadcastCurrentModeState(ControllerContext *ctx)
{
    switch (ctx->mode)
    {
        case MODE_AUTO_SELECT:
            IrSendMisc(ctx, MiscCodeForSelectedSlot(ctx));
            break;

        case MODE_AUTO_RUN:
            IrSendMisc(ctx, MiscCodeForSelectedSlot(ctx));
            IrSendMisc(ctx, IR_MISC_START_PATH);
            break;

        case MODE_CUSTOM_EDIT:
            IrSendMisc(ctx, MiscCodeForStep(ctx->edit_selected_step));
            break;

        case MODE_MANUAL:
        case MODE_ESTOP:
        default:
            break;
    }
}

static void ResetRotateTracker(ControllerContext *ctx)
{
    ctx->rotate_pending = 0;
    ctx->rotate_age = 0U;
    ctx->rotate_rearm_center = 0U;
}

static void ResetSwitchClickState(ControllerContext *ctx)
{
    ctx->sw_short_pending = 0U;
    ctx->sw_double_timer = 0U;
}

static void EnterModeAutoSelect(ControllerContext *ctx, const Joystick_State *input)
{
    IrSendMisc(ctx, IR_MISC_STOP);
    ctx->mode = MODE_AUTO_SELECT;
    SetStatus(ctx, "PATH_SELECT");
    EmitEvent(ctx, "ENTER_AUTO_SELECT", input);
    BroadcastCurrentModeState(ctx);
    ctx->mode_broadcast_tick = 0U;
}

static void EnterModeAutoRun(ControllerContext *ctx, const Joystick_State *input)
{
    char msg[32];

    ctx->mode = MODE_AUTO_RUN;
    BroadcastCurrentModeState(ctx);
    ctx->mode_broadcast_tick = 0U;
    snprintf(msg, sizeof(msg), "RUN P%u", (unsigned int)(ctx->selected_slot + 1U));
    SetStatus(ctx, msg);
    EmitEvent(ctx, msg, input);
}

static void EnterModeCustomEdit(ControllerContext *ctx, const Joystick_State *input)
{
    char msg[32];

    ctx->mode = MODE_CUSTOM_EDIT;
    ctx->edit_slot = ctx->selected_slot;
    ctx->edit_index = 0U;
    ctx->edit_selected_step = STEP_FORWARD;
    ctx->rotate_pending = 0;
    ctx->rotate_age = 0U;
    ctx->rotate_cooldown = 0U;
    ctx->rotate_rearm_center = 0U;
    SetStatus(ctx, "EDIT_PATH");
    IrSendMisc(ctx, MiscCodeForSelectedSlot(ctx));
    IrSendMisc(ctx, IR_MISC_EDIT_PATH);
    BroadcastCurrentModeState(ctx);
    ctx->mode_broadcast_tick = 0U;
    snprintf(msg, sizeof(msg), "ENTER_EDIT P%u", (unsigned int)(ctx->selected_slot + 1U));
    EmitEvent(ctx, msg, input);
}

static void SaveEditedPath(ControllerContext *ctx, const Joystick_State *input)
{
    char msg[32];

    ctx->path_is_saved[ctx->edit_slot] = 1U;
    SetStatus(ctx, "SAVING PATH");

    // Hold save feedback for 500 ms and repeat SAVE_PATH every 100 ms.
    for (uint8_t i = 0U; i < EDIT_SAVE_BROADCAST_COUNT; i++)
    {
        IrSendMisc(ctx, IR_MISC_SAVE_PATH);
        UpdateLCD(ctx);
        delayms((int)EDIT_SAVE_BROADCAST_MS);
    }

    ctx->mode = MODE_MANUAL;
    SetStatus(ctx, "PATH SAVED");
    snprintf(msg, sizeof(msg), "SAVE P%u",
             (unsigned int)(ctx->edit_slot + 1U));
    EmitEvent(ctx, msg, input);
}

static void EnterModeEstop(ControllerContext *ctx, const Joystick_State *input)
{
    ctx->mode = MODE_ESTOP;
    ctx->estop_repeat_tick = 0U;
    SetStatus(ctx, "EMERG STOP");
    EmitEvent(ctx, "E_STOP", input);
    for (uint8_t i = 0; i < ESTOP_ENTRY_BURST_COUNT; i++)
        IrSendMisc(ctx, IR_MISC_ESTOP);
}

static void ClearModeEstop(ControllerContext *ctx, const Joystick_State *input)
{
    ctx->mode = MODE_MANUAL;
    SetStatus(ctx, "E_STOP_CLEAR");
    EmitEvent(ctx, "E_STOP_CLEAR", input);
    IrSendMisc(ctx, IR_MISC_ESTOP_CLEAR);
}

static void TickModeBroadcast(ControllerContext *ctx)
{
    if (ctx->mode == MODE_AUTO_SELECT ||
        ctx->mode == MODE_AUTO_RUN ||
        ctx->mode == MODE_CUSTOM_EDIT)
    {
        if (ctx->mode_broadcast_tick < 0xFFU)
            ctx->mode_broadcast_tick++;

        if (ctx->mode_broadcast_tick >= MODE_BROADCAST_SAMPLES)
        {
            BroadcastCurrentModeState(ctx);
            ctx->mode_broadcast_tick = 0U;
        }
    }
    else
    {
        ctx->mode_broadcast_tick = 0U;
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
        const char saved_marker = (ctx->path_is_saved[ctx->selected_slot] != 0U) ? '*' : '-';
        snprintf(line1, sizeof(line1), "AUTO PATH %u%c",
                 (unsigned int)(ctx->selected_slot + 1U),
                 saved_marker);
    }
    else if (ctx->mode == MODE_CUSTOM_EDIT)
    {
        snprintf(line1, sizeof(line1), "EDIT P%u I%u:%c",
                 (unsigned int)(ctx->edit_slot + 1U),
                 (unsigned int)(ctx->edit_index + 1U),
                 StepChar(ctx->edit_selected_step));
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
    (void)ctx;
    (void)event;
    (void)input;
}

static void CycleSlot(ControllerContext *ctx, const int8_t delta)
{
    int16_t idx = (int16_t)ctx->selected_slot + delta;

    if (idx < 0)
        idx = (int16_t)PATH_SLOT_COUNT - 1;
    else if (idx >= (int16_t)PATH_SLOT_COUNT)
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
        {
            const int8_t throttle = ClampProtocolAxis(input->throttle);
            const int8_t turn = ClampProtocolAxis(input->turn);

            if (throttle == 0 && turn == 0)
            {
                SetStatus(ctx, "DRV STOP");
                IrSendMisc(ctx, IR_MISC_STOP);
            }
            else
            {
                SetManualMixStatus(ctx, throttle, turn);
                (void)SendManualDriveMix(ctx, throttle, turn);
            }

            snprintf(msg, sizeof(msg), "MIX Y%+d X%+d", (int)throttle, (int)turn);
            EmitEvent(ctx, msg, input);
            break;
        }

        case MODE_AUTO_SELECT:
            if (dir == JOYSTICK_DIR_LEFT)
            {
                CycleSlot(ctx, -1);
                SetStatus(ctx, "Select Prev");
                EmitEvent(ctx, "SELECT_PREV", input);
                BroadcastCurrentModeState(ctx);
                ctx->mode_broadcast_tick = 0U;
            }
            else if (dir == JOYSTICK_DIR_RIGHT)
            {
                CycleSlot(ctx, 1);
                SetStatus(ctx, "Select Next");
                EmitEvent(ctx, "SELECT_NEXT", input);
                BroadcastCurrentModeState(ctx);
                ctx->mode_broadcast_tick = 0U;
            }
            break;

        case MODE_CUSTOM_EDIT:
        {
            PathStep step = STEP_NA;

            if (dir == JOYSTICK_DIR_UP)
            {
                step = STEP_FORWARD;
                SetStatus(ctx, "STEP=FWD");
                EmitEvent(ctx, "EDIT_SET_FWD", input);
            }
            else if (dir == JOYSTICK_DIR_DOWN)
            {
                step = STEP_BACKWARD;
                SetStatus(ctx, "STEP=BACK");
                EmitEvent(ctx, "EDIT_SET_BACK", input);
            }
            else if (dir == JOYSTICK_DIR_LEFT)
            {
                step = STEP_LEFT;
                SetStatus(ctx, "STEP=LEFT");
                EmitEvent(ctx, "EDIT_SET_LEFT", input);
            }
            else if (dir == JOYSTICK_DIR_RIGHT)
            {
                step = STEP_RIGHT;
                SetStatus(ctx, "STEP=RIGHT");
                EmitEvent(ctx, "EDIT_SET_RIGHT", input);
            }

            if (step != STEP_NA)
            {
                ctx->edit_selected_step = step;
                IrSendMisc(ctx, MiscCodeForStep(step));
                ctx->mode_broadcast_tick = 0U;
            }
            break;
        }

        case MODE_AUTO_RUN:
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
            IrSendMisc(ctx, IR_MISC_ROTATE_180);
            SetStatus(ctx, "ROT180");
            EmitEvent(ctx, "ROTATE_180", input);
            break;

        case MODE_AUTO_SELECT:
            ctx->mode = MODE_MANUAL;
            SetStatus(ctx, "AUTO_CANCEL");
            EmitEvent(ctx, "AUTO_SELECT_CANCEL", input);
            IrSendMisc(ctx, IR_MISC_STOP_PATH);
            break;

        case MODE_CUSTOM_EDIT:
            if (ctx->edit_index < (MAX_INTERSECTIONS - 1U))
                ctx->edit_index++;

            SetStatus(ctx, "EDIT NEXT");
            EmitEvent(ctx, "EDIT_SAVE_NEXT", input);
            IrSendMisc(ctx, IR_MISC_EDIT_NEXT_STEP);
            BroadcastCurrentModeState(ctx);
            ctx->mode_broadcast_tick = 0U;
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
    switch (ctx->mode)
    {
        case MODE_MANUAL:
            EnterModeAutoSelect(ctx, input);
            break;

        case MODE_AUTO_SELECT:
            if (ctx->path_is_saved[ctx->selected_slot] != 0U)
            {
                EnterModeAutoRun(ctx, input);
            }
            else
            {
                EnterModeCustomEdit(ctx, input);
            }
            break;

        case MODE_CUSTOM_EDIT:
        {
            SaveEditedPath(ctx, input);
            break;
        }

        case MODE_AUTO_RUN:
            ctx->mode = MODE_MANUAL;
            SetStatus(ctx, "AUTO_STOP");
            EmitEvent(ctx, "AUTO_STOP", input);
            IrSendMisc(ctx, IR_MISC_STOP_PATH);
            break;

        case MODE_ESTOP:
        default:
            break;
    }
}

static void HandleLongPress(ControllerContext *ctx, const Joystick_State *input)
{
    ResetSwitchClickState(ctx);

    if (ctx->mode != MODE_ESTOP)
    {
        EnterModeEstop(ctx, input);
        return;
    }

    if (input->direction == JOYSTICK_DIR_CENTER)
        ClearModeEstop(ctx, input);
}

int main(void)
{
    Joystick_State input_state = {0};
    ControllerContext ctx = {0};

    ctx.mode = MODE_MANUAL;
    ctx.last_direction = JOYSTICK_DIR_CENTER;
    ctx.last_throttle = 0;
    ctx.last_turn = 0;
    ctx.edit_selected_step = STEP_FORWARD;

    for (uint8_t c = 0; c < PATH_SLOT_COUNT; c++)
        ctx.path_is_saved[c] = 0U;

    for (uint8_t p = 0; p < DEFAULT_PATH_COUNT; p++)
        ctx.path_is_saved[p] = 1U;

    CFGCON = 0;
    DDPCON = 0;
    INTCONbits.MVEC = 1;

    ANSELB_DisableAnalog(IR_LED_MASK_B);
    GPIOB_SetOutput(IR_LED_MASK_B);
    GPIOB_SetLow(IR_LED_MASK_B);

    Joystick_Init();
    LCD_4BIT();

    SetStatus(&ctx, "BOOT");
    UpdateLCD(&ctx);

    delayms(500); // wait for setup

    SetStatus(&ctx, "READY");
    UpdateLCD(&ctx);

    // Suppress infinite loop warning
    // NOLINTNEXTLINE
    while (1)
    {
        WDTCONSET = _WDTCON_WDTCLR_MASK;
        Joystick_Read(&input_state);

        const Joystick_Direction dir = (Joystick_Direction)input_state.direction;
        const uint8_t direction_changed = (dir != ctx.last_direction) ? 1U : 0U;
        const uint8_t manual_mix_changed =
            ((input_state.throttle != ctx.last_throttle) ||
             (input_state.turn != ctx.last_turn)) ? 1U : 0U;
        const uint8_t active_control_changed =
            (ctx.mode == MODE_MANUAL) ? manual_mix_changed : direction_changed;
        uint8_t rotate_gesture_consumed = 0U;

        if (ctx.mode == MODE_CUSTOM_EDIT)
        {
            if (DetectRotateGesture(&ctx, dir) != 0U)
            {
                ctx.edit_selected_step = STEP_ROT180;
                SetStatus(&ctx, "STEP=ROT180");
                EmitEvent(&ctx, "EDIT_SET_ROT180", &input_state);
                IrSendMisc(&ctx, IR_MISC_ROTATE_180);
                ctx.mode_broadcast_tick = 0U;
                rotate_gesture_consumed = 1U;
            }
        }
        else
        {
            ResetRotateTracker(&ctx);
        }

        if (input_state.sw_button)
        {
            if (ctx.sw_press_samples < 0xFFFFU)
                ctx.sw_press_samples++;

            if (ctx.sw_long_fired == 0U && ctx.sw_press_samples >= SW_LONG_SAMPLES)
            {
                ctx.sw_long_fired = 1U;
                HandleLongPress(&ctx, &input_state);
            }
        }
        else
        {
            if (ctx.sw_press_samples > 0U)
            {
                if (ctx.sw_long_fired == 0U &&
                    ctx.sw_press_samples >= SW_SHORT_MIN_SAMPLES &&
                    ctx.sw_press_samples <= SW_SHORT_MAX_SAMPLES)
                {
                    if (ctx.sw_short_pending != 0U)
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

                ctx.sw_press_samples = 0U;
                ctx.sw_long_fired = 0U;
            }

            if (ctx.sw_short_pending != 0U)
            {
                if (ctx.sw_double_timer < SW_DOUBLE_GAP_SAMPLES)
                {
                    ctx.sw_double_timer++;
                }
                else
                {
                    HandleShortPress(&ctx, dir, &input_state);
                    ctx.sw_short_pending = 0U;
                    ctx.sw_double_timer = 0U;
                }
            }
        }

        if (active_control_changed != 0U)
        {
            if (!(ctx.mode == MODE_CUSTOM_EDIT && rotate_gesture_consumed != 0U))
                HandleDirectionEvent(&ctx, dir, &input_state);
            ctx.last_direction = dir;
            ctx.manual_repeat_tick = 0U;
        }
        else if (ctx.mode == MODE_MANUAL)
        {
            if (ctx.manual_repeat_tick < 0xFFU)
                ctx.manual_repeat_tick++;

            if (ctx.manual_repeat_tick >= MANUAL_REPEAT_SAMPLES)
            {
                HandleDirectionEvent(&ctx, dir, &input_state);
                ctx.manual_repeat_tick = 0U;
            }
        }
        else
        {
            ctx.manual_repeat_tick = 0U;
            TickModeBroadcast(&ctx);
        }

        ctx.last_throttle = input_state.throttle;
        ctx.last_turn = input_state.turn;
        ctx.last_direction = dir;


        if (ctx.mode == MODE_ESTOP)
        {
            if (ctx.estop_repeat_tick == 0U)
            {
                EmitEvent(&ctx, "E_STOP_REPEAT", &input_state);
                SetStatus(&ctx, "EMERG STOP");
                IrSendMisc(&ctx, IR_MISC_ESTOP);
            }
            ctx.estop_repeat_tick++;
            if (ctx.estop_repeat_tick >= ESTOP_REPEAT_SAMPLES)
                ctx.estop_repeat_tick = 0U;
        }

        UpdateLCD(&ctx);
        delayms(INPUT_SAMPLE_MS);
    }
}
