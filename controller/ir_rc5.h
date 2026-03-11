#ifndef IR_RC5_H
#define IR_RC5_H

/*
 * ir_rc5.h  -  RC5 IR protocol codec with magnitude support
 *
 * Usage
 * -----
 *   Transmitter:
 *     IrRC5Frame frame;
 *     IrRC5_BuildFrame(IR_CMD_FORWARD, 0, 7, &frame);  // magnitude 0-7
 *     // Bit-bang frame.bits (14 bits) at 38 kHz, 889 us half-period Manchester.
 *
 *   Receiver (called from input-capture ISR after each edge):
 *     IrRC5Decoder dec;
 *     IrRC5_DecoderReset(&dec);
 *     // On each edge, call IrRC5_DecoderFeed().
 *     // When it returns IR_DECODE_DONE, read dec.frame.
 *
 * Frame layout (14 bits, MSB first, Manchester encoded):
 *   [0]      Start bit 1  (always 1)
 *   [1]      Start bit 2  (always 1)
 *   [2]      Toggle       (flips on each new button press)
 *   [3..4]   Address      (2 bits, 0-3 device addressing, fixed to IR_RC5_ADDRESS)
 *   [5..10]  Command      (6 bits, see IrCommand enum for direction/special commands)
 *   [11..13] Magnitude    (3 bits, 0-7 normalized from joystick, 0=stop, 7=max)
 *
 * Timing:
 *   Carrier:  38 kHz (26.3 us period)
 *   Half-bit: 889 us
 *   Full bit: 1778 us
 *   Frame:    ~24.9 ms (14 bits * 1778 us)
 *   Repeat:   113.778 ms inter-frame gap recommended
 */

#include <stdint.h>

/* -------------------------------------------------------------------------
 * Configuration - change these to match your hardware addressing scheme
 * ---------------------------------------------------------------------- */

/* Device address sent in every frame.  2 bits, 0-3. */
#define IR_RC5_ADDRESS      0x00U

/* Carrier frequency and derived half-bit period */
#define IR_CARRIER_HZ       38000UL
#define IR_HALF_BIT_US      889U
#define IR_FULL_BIT_US      1778U

/* Tolerance window (+/-) for pulse-width matching in the decoder, in us */
#define IR_TIMING_TOLERANCE_US  200U

/* Total bits in one RC5 frame */
#define IR_RC5_FRAME_BITS   14U

/* Magnitude range: 0-7 (3 bits) */
#define IR_MAGNITUDE_MAX    7U

/* Command range: 0-63 (6 bits) */
#define IR_COMMAND_MAX      63U

/* -------------------------------------------------------------------------
 * Named commands  (6-bit command field, values 0-63)
 * ---------------------------------------------------------------------- */

typedef enum
{
    IR_CMD_STOP       = 0x0U,
    IR_CMD_FORWARD    = 0x1U,
    IR_CMD_BACKWARD   = 0x2U,
    IR_CMD_LEFT       = 0x3U,
    IR_CMD_RIGHT      = 0x4U,
    IR_CMD_ROTATE_180 = 0x5U,
    IR_CMD_PATH_1     = 0x6U,
    IR_CMD_PATH_2     = 0x7U,
    IR_CMD_PATH_3     = 0x8U,
    IR_CMD_AUTO_START = 0x9U,
    IR_CMD_AUTO_STOP  = 0xAU,
    IR_CMD_CUSTOM_1   = 0xBU,
    IR_CMD_CUSTOM_2   = 0xCU,
    IR_CMD_CUSTOM_3   = 0xDU,
    IR_CMD_CUSTOM_4   = 0xEU
} IrCommand;

/* -------------------------------------------------------------------------
 * Frame structure
 * ---------------------------------------------------------------------- */

/*
 * IrRC5Frame - a ready-to-transmit or freshly-decoded RC5 frame.
 *
 * bits[0..13]: raw bit values (each 0 or 1), MSB first, suitable for
 *              direct bit-bang output to the IR LED driver.
 * command:     decoded IrCommand value.
 * toggle:      toggle bit value (0 or 1).
 * magnitude:   magnitude value (0-7), representing joystick intensity.
 */
typedef struct
{
    uint8_t  bits[IR_RC5_FRAME_BITS]; /* 0 or 1 each, ready for Manchester TX */
    IrCommand command;
    uint8_t  toggle;
    uint8_t  magnitude;               /* 0-7 */
} IrRC5Frame;

/* -------------------------------------------------------------------------
 * Decoder state  (one instance per receiver, lives in caller's memory)
 * ---------------------------------------------------------------------- */

typedef enum
{
    IR_DECODE_BUSY = 0, /* still receiving bits                          */
    IR_DECODE_DONE,     /* frame complete - read .frame                  */
    IR_DECODE_ERROR     /* framing/timing error - call IrRC5_DecoderReset */
} IrDecodeStatus;

typedef struct
{
    IrRC5Frame frame;        /* populated when status == IR_DECODE_DONE */
    uint8_t    bit_index;    /* next bit position to fill (0-13)         */
    uint8_t    half_count;   /* half-bits accumulated for current bit    */
    uint8_t    last_level;   /* last sampled line level (0 or 1)         */
    uint8_t    toggle;       /* running toggle value                     */
} IrRC5Decoder;

/* -------------------------------------------------------------------------
 * Transmitter API
 * ---------------------------------------------------------------------- */

/*
 * IrRC5_BuildFrame - fill an IrRC5Frame ready for transmission.
 *
 *   cmd       : one of the IrCommand values above.
 *   toggle    : 0 or 1 (caller tracks this and flips on each new key press).
 *   magnitude : 0-7 representing joystick intensity (0=no movement, 7=full).
 *   frame     : output frame, populated on success.
 *
 * Returns 1 on success, 0 if cmd, toggle, or magnitude is out of range.
 */
int IrRC5_BuildFrame(IrCommand cmd, uint8_t toggle, uint8_t magnitude, IrRC5Frame *frame);

/* -------------------------------------------------------------------------
 * Receiver API
 * ---------------------------------------------------------------------- */

/*
 * IrRC5_DecoderReset - initialize or reset a decoder to idle state.
 * Call once at startup and after every IR_DECODE_DONE or IR_DECODE_ERROR.
 */
void IrRC5_DecoderReset(IrRC5Decoder *dec);

/*
 * IrRC5_DecoderFeed - feed one measured pulse into the decoder.
 *
 * Call from your input-capture ISR (or polling loop) after each edge.
 *
 *   dec        : decoder state struct.
 *   level      : the level that just ENDED (1=mark, 0=space).
 *   duration_us: how long that level lasted, in microseconds.
 *
 * Returns IR_DECODE_BUSY while building the frame, IR_DECODE_DONE when the
 * 14th bit has been received (read dec->frame), or IR_DECODE_ERROR on a
 * timing violation.
 */
IrDecodeStatus IrRC5_DecoderFeed(IrRC5Decoder *dec, uint8_t level, uint32_t duration_us);

#endif /* IR_RC5_H */
