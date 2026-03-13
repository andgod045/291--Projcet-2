#include "ir_rc5.h"

/*
 * ir_rc5.c  -  RC5 IR protocol codec implementation
 *
 * No heap allocation, no libc dependencies beyond stdint.h.
 * Safe to include in both the transmitter and receiver firmware.
 */

/* -------------------------------------------------------------------------
 * Internal helpers
 * ---------------------------------------------------------------------- */

/* Returns 1 if duration is within one half-bit period (+/- tolerance). */
static uint8_t is_half_bit(const uint32_t duration_us)
{
    return (duration_us >= (IR_HALF_BIT_US - IR_TIMING_TOLERANCE_US)) &&
           (duration_us <= (IR_HALF_BIT_US + IR_TIMING_TOLERANCE_US));
}

/* Returns 1 if duration is within one full-bit period (+/- tolerance). */
static uint8_t is_full_bit(const uint32_t duration_us)
{
    return (duration_us >= (IR_FULL_BIT_US - IR_TIMING_TOLERANCE_US)) &&
           (duration_us <= (IR_FULL_BIT_US + IR_TIMING_TOLERANCE_US));
}

static uint8_t axis_to_nibble(const int8_t axis)
{
    return (uint8_t)axis & 0x0FU;
}

static int nibble_to_axis(uint8_t nibble, int8_t *axis)
{
    nibble &= 0x0FU;
    if (axis == (void *)0)
        return 0;
    if (nibble == IR_MOVEMENT_NIBBLE_INVALID)
        return 0;

    if ((nibble & 0x08U) != 0U)
        *axis = (int8_t)(nibble | 0xF0U);
    else
        *axis = (int8_t)nibble;

    if (*axis < IR_MOVEMENT_AXIS_MIN || *axis > IR_MOVEMENT_AXIS_MAX)
        return 0;

    return 1;
}

int IrRC5_IsValidDataType(const uint8_t data_type)
{
    return (data_type == (uint8_t)IR_DATA_MISC || data_type == (uint8_t)IR_DATA_MOVEMENT) ? 1 : 0;
}

int IrRC5_IsValidMiscCode(const uint8_t misc_code)
{
    if (misc_code <= (uint8_t)IR_MISC_ESTOP_CLEAR)
        return 1;

    if (misc_code >= IR_MISC_SELECT_PATH_BASE && misc_code <= IR_MISC_SELECT_PATH_MAX)
        return 1;

    return 0;
}

int IrRC5_EncodeMovement(const int8_t x, const int8_t y, uint8_t *data)
{
    if (data == (void *)0)
        return 0;

    if (x < IR_MOVEMENT_AXIS_MIN || x > IR_MOVEMENT_AXIS_MAX)
        return 0;
    if (y < IR_MOVEMENT_AXIS_MIN || y > IR_MOVEMENT_AXIS_MAX)
        return 0;

    *data = (uint8_t)((axis_to_nibble(y) << 4U) | axis_to_nibble(x));
    return 1;
}

int IrRC5_DecodeMovement(const uint8_t data, int8_t *x, int8_t *y)
{
    int8_t x_decoded;
    int8_t y_decoded;

    if (x == (void *)0 || y == (void *)0)
        return 0;

    if (nibble_to_axis((uint8_t)(data & 0x0FU), &x_decoded) == 0)
        return 0;
    if (nibble_to_axis((uint8_t)((data >> 4U) & 0x0FU), &y_decoded) == 0)
        return 0;

    *x = x_decoded;
    *y = y_decoded;
    return 1;
}

int IrRC5_BuildFrame(const uint8_t toggle, const uint8_t data_type, const uint8_t data, IrRC5Frame *frame)
{
    if (frame == (void *)0)
        return 0;
    if (toggle > 1U)
        return 0;
    if (IrRC5_IsValidDataType(data_type) == 0)
        return 0;

    if (data_type == (uint8_t)IR_DATA_MISC)
    {
        if (IrRC5_IsValidMiscCode(data) == 0)
            return 0;
    }
    else
    {
        int8_t x;
        int8_t y;
        if (IrRC5_DecodeMovement(data, &x, &y) == 0)
            return 0;
    }

    frame->bits[0] = 1U;
    frame->bits[1] = 1U;
    frame->bits[2] = toggle;
    frame->bits[3] = (uint8_t)((IR_RC5_ADDRESS >> 1U) & 0x1U);
    frame->bits[4] = (uint8_t)(IR_RC5_ADDRESS & 0x1U);
    frame->bits[5] = data_type;

    for (uint8_t i = 0U; i < 8U; i++)
        frame->bits[6U + i] = (uint8_t)((data >> (7U - i)) & 0x1U);

    frame->toggle = toggle;
    frame->address = IR_RC5_ADDRESS;
    frame->data_type = data_type;
    frame->data = data;

    return 1;
}

int IrRC5_BuildMiscFrame(const uint8_t misc_code, const uint8_t toggle, IrRC5Frame *frame)
{
    if (IrRC5_IsValidMiscCode(misc_code) == 0)
        return 0;

    return IrRC5_BuildFrame(toggle, IR_DATA_MISC, misc_code, frame);
}

int IrRC5_BuildMovementFrame(const int8_t x, const int8_t y, const uint8_t toggle, IrRC5Frame *frame)
{
    uint8_t data;

    if (IrRC5_EncodeMovement(x, y, &data) == 0)
        return 0;

    return IrRC5_BuildFrame(toggle, IR_DATA_MOVEMENT, data, frame);
}

int IrRC5_FrameToMisc(const IrRC5Frame *frame, uint8_t *misc_code)
{
    if (frame == (void *)0 || misc_code == (void *)0)
        return 0;
    if (frame->data_type != (uint8_t)IR_DATA_MISC)
        return 0;
    if (IrRC5_IsValidMiscCode(frame->data) == 0)
        return 0;

    *misc_code = frame->data;
    return 1;
}

int IrRC5_FrameToMovement(const IrRC5Frame *frame, int8_t *x, int8_t *y)
{
    if (frame == (void *)0)
        return 0;
    if (frame->data_type != (uint8_t)IR_DATA_MOVEMENT)
        return 0;

    return IrRC5_DecodeMovement(frame->data, x, y);
}

/* -------------------------------------------------------------------------
 * Receiver
 * ---------------------------------------------------------------------- */

void IrRC5_DecoderReset(IrRC5Decoder *dec)
{
    if (dec == (void *)0)
        return;

    dec->bit_index = 0U;
    dec->half_count = 0U;
    dec->last_level = 0U;
    dec->toggle = 0U;

    for (uint8_t i = 0U; i < IR_RC5_FRAME_BITS; i++)
        dec->frame.bits[i] = 0U;

    dec->frame.toggle = 0U;
    dec->frame.address = IR_RC5_ADDRESS;
    dec->frame.data_type = (uint8_t)IR_DATA_MISC;
    dec->frame.data = (uint8_t)IR_MISC_STOP;
}

IrDecodeStatus IrRC5_DecoderFeed(IrRC5Decoder *dec, const uint8_t level, const uint32_t duration_us)
{
    uint8_t half_count;
    uint8_t bit_val;

    if (dec == (void *)0)
        return IR_DECODE_ERROR;

    if (dec->bit_index >= IR_RC5_FRAME_BITS)
        return IR_DECODE_ERROR;

    if (is_half_bit(duration_us))
        half_count = 1U;
    else if (is_full_bit(duration_us))
        half_count = 2U;
    else
        return IR_DECODE_ERROR;

    while (half_count > 0U)
    {
        half_count--;

        if (dec->half_count == 0U)
        {
            dec->last_level = level;
            dec->half_count = 1U;
        }
        else
        {
            if (dec->last_level == 1U && level == 0U)
                bit_val = 1U;
            else if (dec->last_level == 0U && level == 1U)
                bit_val = 0U;
            else
                return IR_DECODE_ERROR;

            dec->frame.bits[dec->bit_index++] = bit_val;
            dec->half_count = 0U;

            if (dec->bit_index == IR_RC5_FRAME_BITS)
            {
                uint8_t address_rx = 0U;
                uint8_t data_type_rx;
                uint8_t data_rx = 0U;

                if (dec->frame.bits[0] != 1U || dec->frame.bits[1] != 1U)
                    return IR_DECODE_ERROR;

                for (uint8_t i = 0U; i < 2U; i++)
                    address_rx = (uint8_t)((address_rx << 1U) | dec->frame.bits[3U + i]);

                if (address_rx != IR_RC5_ADDRESS)
                    return IR_DECODE_ERROR;

                data_type_rx = dec->frame.bits[5U];
                if (IrRC5_IsValidDataType(data_type_rx) == 0)
                    return IR_DECODE_ERROR;

                for (uint8_t i = 0U; i < 8U; i++)
                    data_rx = (uint8_t)((data_rx << 1U) | dec->frame.bits[6U + i]);

                if (data_type_rx == (uint8_t)IR_DATA_MISC)
                {
                    if (IrRC5_IsValidMiscCode(data_rx) == 0)
                        return IR_DECODE_ERROR;
                }
                else
                {
                    int8_t x;
                    int8_t y;
                    if (IrRC5_DecodeMovement(data_rx, &x, &y) == 0)
                        return IR_DECODE_ERROR;
                }

                dec->frame.toggle = dec->frame.bits[2U];
                dec->frame.address = address_rx;
                dec->frame.data_type = data_type_rx;
                dec->frame.data = data_rx;

                return IR_DECODE_DONE;
            }
        }
    }

    return IR_DECODE_BUSY;
}
