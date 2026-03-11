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
static uint8_t is_half_bit(uint32_t duration_us)
{
    return (duration_us >= (IR_HALF_BIT_US - IR_TIMING_TOLERANCE_US)) &&
           (duration_us <= (IR_HALF_BIT_US + IR_TIMING_TOLERANCE_US));
}

/* Returns 1 if duration is within one full-bit period (+/- tolerance). */
static uint8_t is_full_bit(uint32_t duration_us)
{
    return (duration_us >= (IR_FULL_BIT_US - IR_TIMING_TOLERANCE_US)) &&
           (duration_us <= (IR_FULL_BIT_US + IR_TIMING_TOLERANCE_US));
}

/* -------------------------------------------------------------------------
 * Transmitter
 * ---------------------------------------------------------------------- */

int IrRC5_BuildFrame(IrCommand cmd, uint8_t toggle, uint8_t magnitude, IrRC5Frame *frame)
{
    uint8_t i;
    uint8_t address = IR_RC5_ADDRESS;

    if (frame == (void *)0)
        return 0;
    if (toggle > 1U)
        return 0;
    if ((uint8_t)cmd > IR_COMMAND_MAX)
        return 0;
    if (magnitude > IR_MAGNITUDE_MAX)
        return 0;

    /*
     * New frame layout (14 bits, index 0 = MSB transmitted first):
     *   [0]      = 1  (start bit S1)
     *   [1]      = 1  (start bit S2)
     *   [2]      = toggle
     *   [3..4]   = address (2 bits, MSB first)
     *   [5..10]  = command (6 bits, MSB first)
     *   [11..13] = magnitude (3 bits, MSB first)
     */
    frame->bits[0] = 1U;
    frame->bits[1] = 1U;
    frame->bits[2] = toggle;

    for (i = 0U; i < 2U; i++)
        frame->bits[3U + i] = (uint8_t)((address >> (1U - i)) & 0x1U);

    for (i = 0U; i < 6U; i++)
        frame->bits[5U + i] = (uint8_t)(((uint8_t)cmd >> (5U - i)) & 0x1U);

    for (i = 0U; i < 3U; i++)
        frame->bits[11U + i] = (uint8_t)((magnitude >> (2U - i)) & 0x1U);

    frame->toggle    = toggle;
    frame->command   = cmd;
    frame->magnitude = magnitude;

    return 1;
}

/* -------------------------------------------------------------------------
 * Receiver
 * ---------------------------------------------------------------------- */

void IrRC5_DecoderReset(IrRC5Decoder *dec)
{
    uint8_t i;
    if (dec == (void *)0)
        return;

    dec->bit_index  = 0U;
    dec->half_count = 0U;
    dec->last_level = 0U;
    dec->toggle     = 0U;

    for (i = 0U; i < IR_RC5_FRAME_BITS; i++)
        dec->frame.bits[i] = 0U;

    dec->frame.command   = IR_CMD_STOP;
    dec->frame.toggle    = 0U;
    dec->frame.magnitude = 0U;
}

/*
 * IrRC5_DecoderFeed
 *
 * RC5 uses biphase (Manchester) encoding: every bit cell is one full-bit
 * period (1778 us) split into two half-bits (889 us each).
 *
 *   Logical 1: mark (889 us) then space (889 us)
 *   Logical 0: space (889 us) then mark (889 us)
 *
 * The TSOP33338 outputs an active-LOW demodulated signal, so the level
 * passed to this function should already be inverted by the caller to
 * match the above convention (1=mark, 0=space), or the command mapping
 * in IrRC5_BuildFrame should account for that.
 *
 * Decoding strategy (half-bit counting):
 *   - A pulse of ~889 us contributes one half-bit.
 *   - A pulse of ~1778 us contributes two half-bits (bit boundary crossing).
 *   - Any other width is a framing error.
 *
 * We accumulate half-bits per logical bit cell:
 *   half_count=0: waiting for first half of new bit.
 *   half_count=1: first half received; second half will complete the bit.
 */
IrDecodeStatus IrRC5_DecoderFeed(IrRC5Decoder *dec, uint8_t level, uint32_t duration_us)
{
    uint8_t half_count;
    uint8_t bit_val;

    if (dec == (void *)0)
        return IR_DECODE_ERROR;

    if (dec->bit_index >= IR_RC5_FRAME_BITS)
        return IR_DECODE_ERROR;

    if (is_half_bit(duration_us))
    {
        half_count = 1U;
    }
    else if (is_full_bit(duration_us))
    {
        half_count = 2U;
    }
    else
    {
        /* Duration outside all valid windows - framing error */
        return IR_DECODE_ERROR;
    }

    /*
     * Process each half-bit contributed by this pulse.
     * Each logical bit is determined at its mid-transition:
     *   first half = level, second half = ~level.
     * So when half_count==1: we are either storing the first half of
     * a new cell, or completing a pending cell.
     */
    while (half_count > 0U)
    {
        half_count--;

        if (dec->half_count == 0U)
        {
            /* First half of a new bit cell: record this level */
            dec->last_level = level;
            dec->half_count = 1U;
        }
        else
        {
            /* Second half: the transition defines the bit value.
             * Manchester: mid-cell rising edge (0->1) = logical 0
             *             mid-cell falling edge (1->0) = logical 1 */
            if (dec->last_level == 1U && level == 0U)
                bit_val = 1U;
            else if (dec->last_level == 0U && level == 1U)
                bit_val = 0U;
            else
                return IR_DECODE_ERROR; /* no transition - invalid */

            dec->frame.bits[dec->bit_index++] = bit_val;
            dec->half_count = 0U;

            /* Check if full frame has been received */
            if (dec->bit_index == IR_RC5_FRAME_BITS)
            {
                uint8_t i;
                uint8_t address_rx;
                uint8_t command_rx;
                uint8_t magnitude_rx;

                /* Validate start bits */
                if (dec->frame.bits[0] != 1U || dec->frame.bits[1] != 1U)
                    return IR_DECODE_ERROR;

                /* Extract address and verify it matches ours */
                address_rx = 0U;
                for (i = 0U; i < 2U; i++)
                    address_rx = (uint8_t)((address_rx << 1U) | dec->frame.bits[3U + i]);

                if (address_rx != IR_RC5_ADDRESS)
                    return IR_DECODE_ERROR;

                /* Extract command */
                command_rx = 0U;
                for (i = 0U; i < 6U; i++)
                    command_rx = (uint8_t)((command_rx << 1U) | dec->frame.bits[5U + i]);

                /* Extract magnitude */
                magnitude_rx = 0U;
                for (i = 0U; i < 3U; i++)
                    magnitude_rx = (uint8_t)((magnitude_rx << 1U) | dec->frame.bits[11U + i]);

                dec->frame.toggle    = dec->frame.bits[2];
                dec->frame.command   = (IrCommand)command_rx;
                dec->frame.magnitude = magnitude_rx;

                return IR_DECODE_DONE;
            }
        }
    }

    return IR_DECODE_BUSY;
}

