#pragma once

#include <stddef.h>


namespace TransportLimits
{

/*
 * Maximum opaque application frame accepted by Stage 1 transports.
 *
 * This is deliberately independent of Protocol V1. The coordinator checks
 * that the current V1 maximum fits, while UART framing remains reusable for a
 * different message format with the same bounded transport contract.
 */
static constexpr size_t MAX_OPAQUE_FRAME_SIZE =
    266U;

}
