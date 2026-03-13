#ifndef IR_RC5_H
#define IR_RC5_H

/*
 * ir_rc5.h - 14-bit RC5-like IR protocol codec
 *
 * Wire format (bit index in transmitted frame):
 *   [0:1]   start bits, always 1
 *   [2]     toggle bit (alternate per message)
 *   [3:4]   2-bit address
 *   [5]     data type (0=misc, 1=movement)
 *   [6:13]  data payload (8 bits)
 *
 * Misc payload codes (data type 0):
 *   0 STOP, 1 FORWARD, 2 BACKWARD, 3 LEFT, 4 RIGHT, 5 ROTATE_180,
 *   6 START_PATH, 7 STOP_PATH, 8 EDIT_PATH, 9 SAVE_PATH, 10 EDIT_NEXT_STEP,
 *   11 E_STOP, 12 E_STOP_CLEAR, 13..15 reserved, 16..31 select Path 1..16.
 *
 * Movement payload (data type 1):
 *   data[7:4] = Y nibble, data[3:0] = X nibble.
 *   Each nibble is signed 4-bit two's-complement with valid range -7..7.
 *   Nibble value 0x8 (-8) is invalid and rejected.
 */

#include <stdint.h>

#define IR_RC5_ADDRESS            0x00U
#define IR_CARRIER_HZ             38000UL
#define IR_HALF_BIT_US            889U
#define IR_FULL_BIT_US            1778U
#define IR_TIMING_TOLERANCE_US    200U
#define IR_RC5_FRAME_BITS         14U

#define IR_MOVEMENT_AXIS_MIN      (-7)
#define IR_MOVEMENT_AXIS_MAX      (7)
#define IR_MOVEMENT_MAGNITUDE_MAX 7U
#define IR_MOVEMENT_NIBBLE_INVALID 0x08U

#define IR_MISC_SELECT_PATH_BASE  16U
#define IR_MISC_SELECT_PATH_COUNT 16U
#define IR_MISC_SELECT_PATH_MAX   (IR_MISC_SELECT_PATH_BASE + IR_MISC_SELECT_PATH_COUNT - 1U)

typedef enum
{
    IR_DATA_MISC = 0,
    IR_DATA_MOVEMENT = 1
} IrDataType;

typedef enum
{
    IR_MISC_STOP = 0,
    IR_MISC_FORWARD = 1,
    IR_MISC_BACKWARD = 2,
    IR_MISC_LEFT = 3,
    IR_MISC_RIGHT = 4,
    IR_MISC_ROTATE_180 = 5,
    IR_MISC_START_PATH = 6,
    IR_MISC_STOP_PATH = 7,
    IR_MISC_EDIT_PATH = 8,
    IR_MISC_SAVE_PATH = 9,
    IR_MISC_EDIT_NEXT_STEP = 10,
    IR_MISC_ESTOP = 11,
    IR_MISC_ESTOP_CLEAR = 12,
    IR_MISC_SELECT_PATH_1 = IR_MISC_SELECT_PATH_BASE
} IrMiscCode;

typedef struct
{
    uint8_t bits[IR_RC5_FRAME_BITS];
    uint8_t toggle;
    uint8_t address;
    uint8_t data_type;
    uint8_t data;
} IrRC5Frame;

typedef enum
{
    IR_DECODE_BUSY = 0,
    IR_DECODE_DONE,
    IR_DECODE_ERROR
} IrDecodeStatus;

typedef struct
{
    IrRC5Frame frame;
    uint8_t bit_index;
    uint8_t half_count;
    uint8_t last_level;
    uint8_t toggle;
} IrRC5Decoder;

int IrRC5_IsValidDataType(uint8_t data_type);
int IrRC5_IsValidMiscCode(uint8_t misc_code);

int IrRC5_BuildFrame(uint8_t toggle, uint8_t data_type, uint8_t data, IrRC5Frame *frame);
int IrRC5_BuildMiscFrame(uint8_t misc_code, uint8_t toggle, IrRC5Frame *frame);
int IrRC5_BuildMovementFrame(int8_t x, int8_t y, uint8_t toggle, IrRC5Frame *frame);

int IrRC5_EncodeMovement(int8_t x, int8_t y, uint8_t *data);
int IrRC5_DecodeMovement(uint8_t data, int8_t *x, int8_t *y);

int IrRC5_FrameToMisc(const IrRC5Frame *frame, uint8_t *misc_code);
int IrRC5_FrameToMovement(const IrRC5Frame *frame, int8_t *x, int8_t *y);

void IrRC5_DecoderReset(IrRC5Decoder *dec);
IrDecodeStatus IrRC5_DecoderFeed(IrRC5Decoder *dec, uint8_t level, uint32_t duration_us);

#endif /* IR_RC5_H */
