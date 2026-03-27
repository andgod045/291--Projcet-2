#include <EFM8LB1.h>
#include <stdint.h>

#define SYSCLK 72000000L
#define SARCLK 18000000L
#define VDD    3.3035
#define SIREN_DURATION 3000

// ==================== Calibration ====================
#define RIGHT_WHEEL_TRIM 90
#define TURN_180_TIME_MS 900
#define CURVE_FAST       80
#define CURVE_SLOW       35
#define FIG8_TIME_MS     3000

// ==================== Pins ====================
#define LCD_RS  P1_6
#define LCD_E   P1_5
#define LCD_D4  P1_4
#define LCD_D5  P1_3
#define LCD_D6  P1_2
#define LCD_D7  P1_1
#define BTN_L       P3_7
#define BTN_R       P3_1
#define MOTOR_L_FWD P2_4
#define MOTOR_L_REV P2_3
#define MOTOR_R_FWD P2_1
#define MOTOR_R_REV P2_2
#define BUZZER      P2_0
#define IR_PIN      P0_0
#define LED_LEFT    P0_2
#define LED_RIGHT   P3_0

// ==================== ADC Pins ====================
#define LEFT_COIL_PIN   QFP32_MUX_P1_0
#define RIGHT_COIL_PIN  QFP32_MUX_P2_6
#define CENTER_COIL_PIN QFP32_MUX_P0_1

// ==================== IR RC5 Protocol ====================
#define IR_RC5_ADDRESS            0x00U
#define IR_HALF_BIT_US            889U
#define IR_FULL_BIT_US            1778U
#define IR_TIMING_TOLERANCE_US    200U
#define IR_RC5_FRAME_BITS         14U
#define IR_MOVEMENT_AXIS_MIN      (-7)
#define IR_MOVEMENT_AXIS_MAX      (7)
#define IR_MOVEMENT_NIBBLE_INVALID 0x08U
#define IR_MISC_SELECT_PATH_BASE  16U
#define IR_MISC_SELECT_PATH_MAX   31U

typedef enum { IR_DATA_MISC = 0, IR_DATA_MOVEMENT = 1 } IrDataType;
typedef enum {
    IR_MISC_STOP = 0, IR_MISC_FORWARD = 1, IR_MISC_BACKWARD = 2,
    IR_MISC_LEFT = 3, IR_MISC_RIGHT = 4, IR_MISC_ROTATE_180 = 5,
    IR_MISC_START_PATH = 6, IR_MISC_EDIT_PATH = 8,
    IR_MISC_ESTOP_CLEAR = 12,
    IR_MISC_SELECT_PATH_1 = IR_MISC_SELECT_PATH_BASE
} IrMiscCode;

typedef struct {
    uint8_t bits[IR_RC5_FRAME_BITS];
    uint8_t toggle, address, data_type, payload;
} IrRC5Frame;

typedef enum { IR_DECODE_BUSY = 0, IR_DECODE_DONE, IR_DECODE_ERROR } IrDecodeStatus;
typedef struct {
    IrRC5Frame frame;
    uint8_t bit_index, half_count, last_level, toggle;
} IrRC5Decoder;

static uint8_t is_half_bit(uint32_t d) { return (d >= 689U) && (d <= 1089U); }
static uint8_t is_full_bit(uint32_t d) { return (d >= 1578U) && (d <= 1978U); }

static int nibble_to_axis(uint8_t nib, int8_t *axis) {
    nib &= 0x0FU;
    if (!axis || nib == 0x08U) return 0;
    *axis = (nib & 0x08U) ? (int8_t)(nib | 0xF0U) : (int8_t)nib;
    return (*axis >= IR_MOVEMENT_AXIS_MIN && *axis <= IR_MOVEMENT_AXIS_MAX);
}

int IrRC5_IsValidDataType(uint8_t dt) { return (dt <= 1) ? 1 : 0; }
int IrRC5_IsValidMiscCode(uint8_t mc) {
    return (mc <= 12 || (mc >= IR_MISC_SELECT_PATH_BASE && mc <= IR_MISC_SELECT_PATH_MAX)) ? 1 : 0;
}

int IrRC5_DecodeMovement(uint8_t payload, int8_t *x, int8_t *y) {
    int8_t xd, yd;
    if (!x || !y) return 0;
    if (!nibble_to_axis(payload & 0x0F, &xd)) return 0;
    if (!nibble_to_axis((payload >> 4) & 0x0F, &yd)) return 0;
    *x = xd; *y = yd; return 1;
}

void IrRC5_DecoderReset(IrRC5Decoder *d) {
    uint8_t i;
    if (!d) return;
    d->bit_index = 0; d->half_count = 0; d->last_level = 0; d->toggle = 0;
    for (i = 0; i < IR_RC5_FRAME_BITS; i++) d->frame.bits[i] = 0;
    d->frame.toggle = 0; d->frame.address = IR_RC5_ADDRESS;
    d->frame.data_type = 0; d->frame.payload = 0;
}

IrDecodeStatus IrRC5_DecoderFeed(IrRC5Decoder *d, uint8_t level, uint32_t dur) {
    uint8_t hc, bv;
    if (!d || d->bit_index >= IR_RC5_FRAME_BITS) return IR_DECODE_ERROR;
    if (is_half_bit(dur)) hc = 1;
    else if (is_full_bit(dur)) hc = 2;
    else return IR_DECODE_ERROR;

    while (hc > 0) {
        hc--;
        if (d->half_count == 0) { d->last_level = level; d->half_count = 1; }
        else {
            if (d->last_level == 1 && level == 0) bv = 1;
            else if (d->last_level == 0 && level == 1) bv = 0;
            else return IR_DECODE_ERROR;
            d->frame.bits[d->bit_index++] = bv;
            d->half_count = 0;
            if (d->bit_index == IR_RC5_FRAME_BITS) {
                uint8_t i, addr = 0, dt, pay = 0;
                if (d->frame.bits[0] != 1 || d->frame.bits[1] != 1) return IR_DECODE_ERROR;
                for (i = 0; i < 2; i++) addr = (addr << 1) | d->frame.bits[3+i];
                if (addr != IR_RC5_ADDRESS) return IR_DECODE_ERROR;
                dt = d->frame.bits[5];
                if (!IrRC5_IsValidDataType(dt)) return IR_DECODE_ERROR;
                for (i = 0; i < 8; i++) pay = (pay << 1) | d->frame.bits[6+i];
                if (dt == 0) { if (!IrRC5_IsValidMiscCode(pay)) return IR_DECODE_ERROR; }
                else { int8_t tx, ty; if (!IrRC5_DecodeMovement(pay, &tx, &ty)) return IR_DECODE_ERROR; }
                d->frame.toggle = d->frame.bits[2];
                d->frame.address = addr; d->frame.data_type = dt; d->frame.payload = pay;
                return IR_DECODE_DONE;
            }
        }
    }
    return IR_DECODE_BUSY;
}

int IrRC5_FrameToMisc(const IrRC5Frame *f, uint8_t *mc) {
    if (!f || !mc || f->data_type != 0 || !IrRC5_IsValidMiscCode(f->payload)) return 0;
    *mc = f->payload; return 1;
}

int IrRC5_FrameToMovement(const IrRC5Frame *f, int8_t *x, int8_t *y) {
    if (!f || f->data_type != 1) return 0;
    return IrRC5_DecodeMovement(f->payload, x, y);
}

// ==================== Forward Declarations ====================
void Timer3us(unsigned char us);
void waitms(unsigned int ms);
void LCD_Pulse_E(void);
void LCD_Write_Nibble(unsigned char nib);
void LCD_Write_Byte(unsigned char byte, bit isData);
void LCD_Command(unsigned char cmd);
void LCD_Data(unsigned char ch);
void LCD_Print(const char *s);
void LCD_SetCursor(unsigned char row, unsigned char col);
void LCD_Init(void);
void LCD_PrintMotorCompact(unsigned char pwmL, unsigned char pwmR);

// ==================== Globals ====================
volatile unsigned char pwm_L = 0, dir_L = 1, pwm_R = 0, dir_R = 1;
volatile unsigned char play_siren = 0;
volatile unsigned int siren_countdown = 0;
IrRC5Decoder ir_decoder;
volatile IrDecodeStatus ir_status = IR_DECODE_BUSY;

typedef enum { MODE_MANUAL, MODE_CIRCLE, MODE_FIGURE_8, MODE_ROTATE_180, MODE_TRACK } RobotMode;

// ==================== Track Following Config ====================
#define DEFAULT_ROBOT_SPEED   70
#define FOLLOW_TURN_SPEED     14
#define COIL_DIFF_OFFSET_MV   0
#define ALIGN_DEADBAND_MV     220
#define INTERSECTION_THRESHOLD_HIGH_MV 500
#define INTERSECTION_THRESHOLD_LOW_MV  250
#define INTERSECTION_DEBOUNCE_COUNT   1
#define INTERSECTION_COOLDOWN_MS      3500
#define ACTION_SETTLE_STOP_MS       50
#define BEFORE_TURN_CENTER_MS       1250
#define BEFORE_TURN_CENTER_SPEED    35
#define AFTER_INTERSECTION_MS       2000
#define AFTER_INTERSECTION_SPEED    40
#define INTERSECTION_HOLD_MS        120
#define SNAP_TURN_SPEED             55
#define SNAP_TURN_LEFT_MS           1900
#define SNAP_TURN_RIGHT_MS          1900
#define TURN_LED_BLINK_MS           100
#define NUM_PATHS             3
#define NUM_INTERSECTIONS     8
#define GO_FORWARD  0
#define GO_LEFT     1
#define GO_RIGHT    2
#define STOP_ACTION 3
#define CENTER_READY     0
#define CENTER_TRIGGERED 1

unsigned char code path_table[NUM_PATHS][NUM_INTERSECTIONS] = {
    { GO_FORWARD, GO_LEFT,    GO_LEFT,    GO_FORWARD, GO_RIGHT,   GO_LEFT,    GO_RIGHT,   STOP_ACTION },
    { GO_LEFT,    GO_RIGHT,   GO_LEFT,    GO_RIGHT,   GO_FORWARD, GO_FORWARD, STOP_ACTION, STOP_ACTION },
    { GO_RIGHT,   GO_FORWARD, GO_RIGHT,   GO_LEFT,    GO_RIGHT,   GO_LEFT,    GO_FORWARD, STOP_ACTION }
};

idata unsigned char track_path = 0, track_intersection = 0, track_done = 0;
idata unsigned int  track_cooldown = 0;
idata unsigned char track_center_state = CENTER_READY, track_high_deb = 0, track_low_deb = 0;
idata unsigned int track_left_mv = 0, track_right_mv = 0, track_center_mv = 0;

// ==================== Motor Helpers ====================
static unsigned char trim_right(unsigned char p) {
    unsigned int s = ((unsigned int)p * RIGHT_WHEEL_TRIM) / 100;
    return (s > 100) ? 100 : (unsigned char)s;
}
static void motors_stop(void) { pwm_L = 0; pwm_R = 0; dir_L = 0; dir_R = 0; }
static void motors_forward(unsigned char s) { dir_L = 0; dir_R = 0; pwm_L = s; pwm_R = trim_right(s); }
static void pivot_left(unsigned char s) { dir_L = 1; dir_R = 0; pwm_L = s; pwm_R = trim_right(s); }
static void pivot_right(unsigned char s) { dir_L = 0; dir_R = 1; pwm_L = s; pwm_R = trim_right(s); }

static void follow_wire(void) {
    int diff = ((int)track_left_mv - (int)track_right_mv) + COIL_DIFF_OFFSET_MV;
    if (diff > ALIGN_DEADBAND_MV) {
        dir_L = 1; dir_R = 0; pwm_L = FOLLOW_TURN_SPEED; pwm_R = DEFAULT_ROBOT_SPEED;
        LED_LEFT = 1; LED_RIGHT = 0;
    } else if (diff < -ALIGN_DEADBAND_MV) {
        dir_L = 0; dir_R = 1; pwm_L = DEFAULT_ROBOT_SPEED; pwm_R = FOLLOW_TURN_SPEED;
        LED_LEFT = 0; LED_RIGHT = 1;
    } else {
        dir_L = 0; dir_R = 0; pwm_L = DEFAULT_ROBOT_SPEED; pwm_R = trim_right(DEFAULT_ROBOT_SPEED);
        LED_LEFT = 0; LED_RIGHT = 0;
    }
}

static void show_next_turn_led(void) {
    unsigned char a;
    if (track_done || track_intersection >= NUM_INTERSECTIONS) {
        LED_LEFT = 0; LED_RIGHT = 0;
        LCD_SetCursor(0, 0); LCD_Print("TRACK DONE    ");
        return;
    }
    a = path_table[track_path][track_intersection];
    LED_LEFT = (a == GO_LEFT); LED_RIGHT = (a == GO_RIGHT);
    LCD_SetCursor(0, 0);
    LCD_Data('T'); LCD_Data('1' + track_path);
    LCD_Data(' '); LCD_Data('#'); LCD_Data('1' + track_intersection); LCD_Data(' ');
    switch(a) {
        case GO_FORWARD: LCD_Print("FWD     "); break;
        case GO_LEFT:    LCD_Print("LEFT    "); break;
        case GO_RIGHT:   LCD_Print("RIGHT   "); break;
        case STOP_ACTION:LCD_Print("STOP    "); break;
        default:         LCD_Print("???     "); break;
    }
}

static void blink_turn(unsigned char is_left, unsigned int total_ms) {
    static xdata unsigned int elapsed;
    unsigned int step;
    unsigned char led = 1;
    elapsed = 0;
    while (elapsed < total_ms) {
        step = TURN_LED_BLINK_MS;
        if ((total_ms - elapsed) < step) step = total_ms - elapsed;
        waitms(step); elapsed += step;
        led = !led;
        if (is_left) LED_LEFT = led; else LED_RIGHT = led;
    }
    LED_LEFT = 0; LED_RIGHT = 0;
}

static void do_intersection_action(void) {
    unsigned char action;
    if (track_done) return;
    action = (track_intersection >= NUM_INTERSECTIONS) ? STOP_ACTION : path_table[track_path][track_intersection];
    switch(action) {
        case GO_FORWARD:
            LED_LEFT = 0; LED_RIGHT = 0;
            motors_forward(AFTER_INTERSECTION_SPEED); waitms(AFTER_INTERSECTION_MS);
            break;
        case GO_LEFT:
            LED_LEFT = 1; LED_RIGHT = 0;
            motors_forward(BEFORE_TURN_CENTER_SPEED); waitms(BEFORE_TURN_CENTER_MS);
            motors_stop(); waitms(ACTION_SETTLE_STOP_MS + INTERSECTION_HOLD_MS);
            pivot_left(SNAP_TURN_SPEED); blink_turn(1, SNAP_TURN_LEFT_MS);
            motors_stop();
            motors_forward(AFTER_INTERSECTION_SPEED); waitms(AFTER_INTERSECTION_MS);
            break;
        case GO_RIGHT:
            LED_LEFT = 0; LED_RIGHT = 1;
            motors_forward(BEFORE_TURN_CENTER_SPEED); waitms(BEFORE_TURN_CENTER_MS);
            motors_stop(); waitms(ACTION_SETTLE_STOP_MS + INTERSECTION_HOLD_MS);
            pivot_right(SNAP_TURN_SPEED); blink_turn(0, SNAP_TURN_RIGHT_MS);
            motors_stop();
            motors_forward(AFTER_INTERSECTION_SPEED); waitms(AFTER_INTERSECTION_MS);
            break;
        default:
            LED_LEFT = 0; LED_RIGHT = 0; motors_stop(); track_done = 1;
            break;
    }
    if (track_intersection < 255) track_intersection++;
    track_cooldown = INTERSECTION_COOLDOWN_MS;
    show_next_turn_led();
}

void Track_Init(unsigned char pi) {
    track_path = (pi >= NUM_PATHS) ? 0 : pi;
    track_intersection = 0; track_done = 0; track_cooldown = 2000;
    track_center_state = CENTER_READY; track_high_deb = 0; track_low_deb = 0;
    motors_stop(); show_next_turn_led();
}

void Track_Process(void) {
    unsigned char triggered = 0;
    if (track_cooldown > 30) track_cooldown -= 30; else track_cooldown = 0;
    if (!track_done) {
        if (track_center_state == CENTER_READY) {
            if (track_cooldown == 0 && track_center_mv > INTERSECTION_THRESHOLD_HIGH_MV) {
                if (track_high_deb < 255) track_high_deb++;
                track_low_deb = 0;
                if (track_high_deb >= INTERSECTION_DEBOUNCE_COUNT) {
                    track_center_state = CENTER_TRIGGERED; track_high_deb = 0;
                    triggered = 1; do_intersection_action();
                }
            } else { track_high_deb = 0; }
        } else {
            if (track_center_mv < INTERSECTION_THRESHOLD_LOW_MV) {
                if (track_low_deb < 255) track_low_deb++;
                if (track_low_deb >= INTERSECTION_DEBOUNCE_COUNT) { track_center_state = CENTER_READY; track_low_deb = 0; }
            } else { track_low_deb = 0; }
        }
        if (!triggered && !track_done) follow_wire();
    } else { motors_stop(); }
}

void LCD_PrintAxis(int8_t v) {
    LCD_Data((v < 0) ? '-' : ' ');
    if (v < 0) v = -v;
    LCD_Data('0' + v);
}

// ==================== Hardware Init & ISRs ====================
void trigger_siren(void) { play_siren = 1; siren_countdown = SIREN_DURATION; }

char _c51_external_startup(void) {
    SFRPAGE = 0x00; WDTCN = 0xDE; WDTCN = 0xAD;
    VDM0CN = 0x80; RSTSRC = 0x02|0x04;
    SFRPAGE = 0x10; PFE0CN = 0x20; SFRPAGE = 0x00;
    CLKSEL = 0x00; CLKSEL = 0x00; while ((CLKSEL & 0x80) == 0);
    CLKSEL = 0x03; CLKSEL = 0x03; while ((CLKSEL & 0x80) == 0);
    P0MDOUT |= 0x10; XBR0 = 0x01; XBR1 = 0x00; XBR2 = 0x40;
    return 0;
}

void Timer2_Init(void) {
    TMR2CN0 = 0x00; CKCON0 |= 0x10;
    TMR2RL = 65536 - 7200; TMR2 = TMR2RL;
    IE |= 0x20; EA = 1; TR2 = 1;
}

void Timer2_ISR(void) interrupt 5 using 1 {
    static unsigned char pwm_counter = 0, last_ir = 1, siren_tone = 0, buzzer_tick = 0;
    static unsigned int ir_ticks = 0, siren_timer = 0;
    unsigned char cur_ir, tick_limit;
    TF2H = 0;

    // Siren
    if (play_siren) {
        if (siren_countdown > 0) siren_countdown--; else play_siren = 0;
        siren_timer++;
        if (siren_timer > 2500) { siren_timer = 0; siren_tone = !siren_tone; }
        buzzer_tick++;
        tick_limit = siren_tone ? 8 : 5;
        if (buzzer_tick >= tick_limit) { buzzer_tick = 0; BUZZER = !BUZZER; }
    } else { BUZZER = 0; siren_timer = 0; buzzer_tick = 0; }

    // PWM
    pwm_counter++;
    if (pwm_counter >= 100) pwm_counter = 0;
    if (pwm_L == 0) { MOTOR_L_FWD = 0; MOTOR_L_REV = 0; }
    else if (pwm_counter < pwm_L) { MOTOR_L_FWD = !dir_L; MOTOR_L_REV = dir_L; }
    else { MOTOR_L_FWD = 0; MOTOR_L_REV = 0; }
    if (pwm_R == 0) { MOTOR_R_FWD = 0; MOTOR_R_REV = 0; }
    else if (pwm_counter < pwm_R) { MOTOR_R_FWD = !dir_R; MOTOR_R_REV = dir_R; }
    else { MOTOR_R_FWD = 0; MOTOR_R_REV = 0; }

    // IR Decoder
    ir_ticks += 100;
    cur_ir = IR_PIN;
    if (cur_ir != last_ir) {
        if (ir_status != IR_DECODE_DONE)
            ir_status = IrRC5_DecoderFeed(&ir_decoder, (last_ir == 0) ? 1 : 0, ir_ticks);
        ir_ticks = 0; last_ir = cur_ir;
    } else if (ir_ticks > 10000) {
        if (ir_status != IR_DECODE_DONE) { IrRC5_DecoderReset(&ir_decoder); ir_status = IR_DECODE_BUSY; }
        ir_ticks = 10000;
    }
}

// ==================== ADC ====================
void InitADC(void) {
    SFRPAGE = 0x00; ADEN = 0;
    ADC0CN1 = 0x80; ADC0CF0 = ((SYSCLK/SARCLK) << 3);
    ADC0CF1 = 0x1E; ADC0CN0 = 0x00;
    ADC0CF2 = 0x3F; ADC0CN2 = 0x00; ADEN = 1;
}

void InitPinADC(unsigned char port, unsigned char pin) {
    unsigned char m = 1 << pin;
    SFRPAGE = 0x20;
    switch (port) {
        case 0: P0MDIN &= ~m; P0SKIP |= m; break;
        case 1: P1MDIN &= ~m; P1SKIP |= m; break;
        case 2: P2MDIN &= ~m; P2SKIP |= m; break;
    }
    SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin) {
    ADC0MX = pin; Timer3us(100);
    ADINT = 0; ADBUSY = 1; while (!ADINT);
    ADINT = 0; ADBUSY = 1; while (!ADINT);
    return ADC0;
}

static unsigned int ADC_to_mV(unsigned char pin) {
    return (unsigned int)(((unsigned long)ADC_at_Pin(pin) * 3304UL) / 16383UL);
}

float Volts_at_Pin(unsigned char pin) {
    return ((ADC_at_Pin(pin) * VDD) / 0x3FFF);
}

// ==================== LCD ====================
void LCD_Pulse_E(void) { LCD_E = 1; Timer3us(2); LCD_E = 0; Timer3us(50); }
void LCD_Write_Nibble(unsigned char n) {
    LCD_D4 = (n & 1); LCD_D5 = (n >> 1) & 1; LCD_D6 = (n >> 2) & 1; LCD_D7 = (n >> 3) & 1;
    LCD_Pulse_E();
}
void LCD_Write_Byte(unsigned char b, bit isData) { LCD_RS = isData; LCD_Write_Nibble(b >> 4); LCD_Write_Nibble(b & 0x0F); }
void LCD_Command(unsigned char c) { LCD_Write_Byte(c, 0); if (c == 0x01 || c == 0x02) waitms(2); }
void LCD_Data(unsigned char c) { LCD_Write_Byte(c, 1); }
void LCD_Print(const char *s) { while(*s) LCD_Data(*s++); }
void LCD_SetCursor(unsigned char r, unsigned char c) { LCD_Command(0x80 | ((r ? 0x40 : 0x00) + c)); }
void LCD_Init(void) {
    LCD_RS = 0; LCD_E = 0; waitms(20);
    LCD_Write_Nibble(0x03); waitms(5);
    LCD_Write_Nibble(0x03); Timer3us(150);
    LCD_Write_Nibble(0x03); Timer3us(150);
    LCD_Write_Nibble(0x02);
    LCD_Command(0x28); LCD_Command(0x0C); LCD_Command(0x06); LCD_Command(0x01); waitms(2);
}
void LCD_PrintMotorCompact(unsigned char L, unsigned char R) {
    LCD_SetCursor(1, 0); LCD_Print("L:");
    if (L >= 100) LCD_Print("99"); else { LCD_Data('0'+L/10); LCD_Data('0'+L%10); }
    LCD_Print(" R:");
    if (R >= 100) LCD_Print("99"); else { LCD_Data('0'+R/10); LCD_Data('0'+R%10); }
    LCD_Print("  ");
}

// ==================== Timer3 / Waitms ====================
void Timer3us(unsigned char us) {
    unsigned char i; CKCON0 |= 0x40;
    TMR3RL = (-(SYSCLK)/1000000L); TMR3 = TMR3RL; TMR3CN0 = 0x04;
    for (i = 0; i < us; i++) { while (!(TMR3CN0 & 0x80)); TMR3CN0 &= ~0x80; }
    TMR3CN0 = 0;
}
void waitms(unsigned int ms) {
    unsigned int j; unsigned char k;
    for (j = 0; j < ms; j++) for (k = 0; k < 4; k++) Timer3us(250);
}

// ==================== Main ====================
void main(void) {
    xdata unsigned char btn_L_pressed = 0, btn_R_pressed = 0;
    xdata RobotMode current_mode = MODE_MANUAL;
    xdata unsigned int mode_timer_ms = 0;
    xdata unsigned char fig8_state = 0;

    _c51_external_startup();
    SFRPAGE = 0x20;
    P1MDOUT |= 0x7E; P2MDOUT |= 0x1E;
    P3MDIN |= 0x83; P3MDOUT |= 0x01; P3MDOUT &= ~0x82;
    P0MDIN |= 0x05; P0MDOUT |= 0x04; P0MDOUT &= ~0x01; P0SKIP |= 0x01;
    SFRPAGE = 0x00;
    P3 |= 0x82; P0 |= 0x01;
    LED_LEFT = 0; LED_RIGHT = 0;

    InitPinADC(0, 1); InitPinADC(1, 0); InitPinADC(2, 6); InitADC();
    MOTOR_L_FWD = 0; MOTOR_L_REV = 0; MOTOR_R_FWD = 0; MOTOR_R_REV = 0;
    IrRC5_DecoderReset(&ir_decoder); Timer2_Init();
    LCD_Init();
    LCD_SetCursor(0, 0); LCD_Print("WAITING       ");
    LCD_SetCursor(1, 0); LCD_Print("L:00 R:00     ");

    while(1) {
        // --- Mode Execution ---
        if (current_mode == MODE_CIRCLE) {
            dir_L = 0; dir_R = 0; pwm_L = CURVE_FAST;
            pwm_R = ((unsigned int)CURVE_SLOW * RIGHT_WHEEL_TRIM) / 100;
        } else if (current_mode == MODE_FIGURE_8) {
            dir_L = 0; dir_R = 0; mode_timer_ms += 30;
            if (fig8_state == 0) {
                pwm_L = CURVE_FAST; pwm_R = ((unsigned int)CURVE_SLOW * RIGHT_WHEEL_TRIM) / 100;
                if (mode_timer_ms >= FIG8_TIME_MS) { mode_timer_ms = 0; fig8_state = 1; }
            } else {
                pwm_L = CURVE_SLOW; pwm_R = ((unsigned int)CURVE_FAST * RIGHT_WHEEL_TRIM) / 100;
                if (mode_timer_ms >= FIG8_TIME_MS) { mode_timer_ms = 0; fig8_state = 0; }
            }
        } else if (current_mode == MODE_ROTATE_180) {
            dir_L = 1; dir_R = 0; pwm_L = 100; pwm_R = RIGHT_WHEEL_TRIM;
            mode_timer_ms += 30;
            if (mode_timer_ms >= TURN_180_TIME_MS) {
                current_mode = MODE_MANUAL; pwm_L = 0; pwm_R = 0;
                LCD_SetCursor(0, 0); LCD_Print("STOPPED"); trigger_siren();
            }
        } else if (current_mode == MODE_TRACK) {
            track_left_mv   = ADC_to_mV(LEFT_COIL_PIN);
            track_right_mv  = ADC_to_mV(RIGHT_COIL_PIN);
            track_center_mv = ADC_to_mV(CENTER_COIL_PIN);
            Track_Process();
        }

        // --- IR Remote ---
        if (ir_status == IR_DECODE_DONE) {
            if (ir_decoder.frame.data_type == IR_DATA_MISC) {
                uint8_t mc;
                if (IrRC5_FrameToMisc((IrRC5Frame *)&ir_decoder.frame, &mc)) {
                    unsigned char dp = 80, tr = ((unsigned int)80 * RIGHT_WHEEL_TRIM) / 100;
                    if (!(current_mode == MODE_ROTATE_180 && mc == IR_MISC_STOP)) {
                        LCD_SetCursor(0, 0);
                        trigger_siren();
                        switch (mc) {
                            case IR_MISC_START_PATH: LCD_Print("CIRCLE "); current_mode = MODE_CIRCLE; break;
                            case IR_MISC_EDIT_PATH:  LCD_Print("FIG-8  "); current_mode = MODE_FIGURE_8; mode_timer_ms = 0; fig8_state = 0; break;
                            case IR_MISC_FORWARD:    LCD_Print("FWD    "); current_mode = MODE_MANUAL; dir_L = 0; dir_R = 0; pwm_L = dp; pwm_R = tr; LED_LEFT = 0; LED_RIGHT = 0; break;
                            case IR_MISC_BACKWARD:   LCD_Print("BCK    "); current_mode = MODE_MANUAL; dir_L = 1; dir_R = 1; pwm_L = dp; pwm_R = tr; LED_LEFT = 0; LED_RIGHT = 0; break;
                            case IR_MISC_LEFT:       LCD_Print("LFT    "); current_mode = MODE_MANUAL; dir_L = 1; dir_R = 0; pwm_L = dp; pwm_R = tr; LED_LEFT = 1; LED_RIGHT = 0; break;
                            case IR_MISC_RIGHT:      LCD_Print("RGT    "); current_mode = MODE_MANUAL; dir_L = 0; dir_R = 1; pwm_L = dp; pwm_R = tr; LED_LEFT = 0; LED_RIGHT = 1; break;
                            case IR_MISC_STOP:       LCD_Print("STP    "); current_mode = MODE_MANUAL; pwm_L = 0; pwm_R = 0; LED_LEFT = 0; LED_RIGHT = 0; break;
                            case IR_MISC_ROTATE_180: LCD_Print("180    "); current_mode = MODE_ROTATE_180; mode_timer_ms = 0; break;
                            case IR_MISC_SELECT_PATH_1:     current_mode = MODE_TRACK; Track_Init(0); break;
                            case IR_MISC_SELECT_PATH_1 + 1: current_mode = MODE_TRACK; Track_Init(1); break;
                            case IR_MISC_SELECT_PATH_1 + 2: current_mode = MODE_TRACK; Track_Init(2); break;
                            default: LCD_Print("???    "); break;
                        }
                    }
                }
            } else if (ir_decoder.frame.data_type == IR_DATA_MOVEMENT) {
                int8_t x, y;
                if (IrRC5_FrameToMovement((IrRC5Frame *)&ir_decoder.frame, &x, &y)) {
                    if (!(current_mode == MODE_ROTATE_180 && x == 0 && y == 0)) {
                        int lv, rv;
                        current_mode = MODE_MANUAL; trigger_siren();
                        LCD_SetCursor(0, 0);
                        LCD_Data('Y'); LCD_PrintAxis(y); LCD_Data('X'); LCD_PrintAxis(x); LCD_Data(' ');
                        lv = ((int)y*100)/7 + ((int)x*100)/7;
                        rv = ((int)y*100)/7 - ((int)x*100)/7;
                        if (lv > 100) lv = 100; if (lv < -100) lv = -100;
                        if (rv > 100) rv = 100; if (rv < -100) rv = -100;
                        if (lv >= 0) { dir_L = 0; pwm_L = lv; } else { dir_L = 1; pwm_L = -lv; }
                        if (rv >= 0) { dir_R = 0; pwm_R = ((unsigned int)rv * RIGHT_WHEEL_TRIM)/100; }
                        else         { dir_R = 1; pwm_R = ((unsigned int)(-rv) * RIGHT_WHEEL_TRIM)/100; }
                        LED_LEFT = (x > 0); LED_RIGHT = (x < 0);
                    }
                }
            }
            EA = 0; IrRC5_DecoderReset((IrRC5Decoder *)&ir_decoder); ir_status = IR_DECODE_BUSY; EA = 1;
        }

        // --- Hardware Buttons ---
        if (BTN_L == 0) {
            if (btn_L_pressed == 0) {
                if (current_mode == MODE_TRACK && !track_done) {
                    track_cooldown = 0; track_center_state = CENTER_READY;
                    track_high_deb = INTERSECTION_DEBOUNCE_COUNT;
                    do_intersection_action(); trigger_siren();
                } else {
                    current_mode = MODE_MANUAL;
                    LCD_SetCursor(0, 0); LCD_Print("L-OVRD ");
                    dir_L = !dir_L; pwm_L = 0; trigger_siren();
                }
                btn_L_pressed = 1;
            }
            if (current_mode != MODE_TRACK && pwm_L < 100) pwm_L++;
        } else btn_L_pressed = 0;

        if (BTN_R == 0) {
            if (btn_R_pressed == 0) {
                if (current_mode == MODE_TRACK && !track_done) {
                    track_cooldown = 0; track_center_state = CENTER_READY;
                    track_high_deb = INTERSECTION_DEBOUNCE_COUNT;
                    do_intersection_action(); trigger_siren();
                } else {
                    current_mode = MODE_MANUAL;
                    LCD_SetCursor(0, 0); LCD_Print("R-OVRD ");
                    dir_R = !dir_R; pwm_R = 0; trigger_siren();
                }
                btn_R_pressed = 1;
            }
            if (current_mode != MODE_TRACK && pwm_R < RIGHT_WHEEL_TRIM) pwm_R++;
        } else btn_R_pressed = 0;

        LCD_PrintMotorCompact(pwm_L, pwm_R);
        waitms(30);
    }
}
