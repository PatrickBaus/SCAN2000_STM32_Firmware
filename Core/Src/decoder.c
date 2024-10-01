#include <stddef.h>
#include <decoder.h>

const uint8_t scan2000ChannelOffSequence[] = {17, 19, 21, 23, 8, 14, 0, 2, 4, 5, 12};      // CH1...CH10, 4W
const uint8_t scan2000ChannelOnSequence[] =  {16, 18, 20, 22, 9, 13, 15, 1, 3, 6, 11};      // CH1...CH10, 4W

const uint8_t scan2000_20ChannelSequence[] = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 1, 2, 3, 4, 5, 6, 7 , 8, 9, 10, 21, 22};    // CH11...CH20, CH1...CH10, Bank 2 to OUT, Bank 2 to 4W

/**
 * @brief Decode 10 channel command
 * @param command Command received
 * @param relaySetRegister ptr to bit map for setting relays
 * @param relayUnsetRegister ptr to bit map for unsetting relays
 * @return decodeResult_t
 */
decodeResult_t decode_10channels(uint32_t command, uint32_t *relaySetRegister, uint32_t *relayUnsetRegister) {
    decodeResult_t rv;
    // Check always high bits
    if ((command & SCAN_2000_ALWAYS_HIGH_BITS) != SCAN_2000_ALWAYS_HIGH_BITS) {
        return decodeDataError;
    }
    // Remove the bits, that are always high
    command &= ~SCAN_2000_ALWAYS_HIGH_BITS;
    *relaySetRegister = 0x0000;
    *relayUnsetRegister = 0x0000;

    // There is no need to run the loop, if there is nothing to do. Every second command
    // only contains the bits that are always high. We can ignore those commands.
    if (command != 0x00000000) {
        // 10 channels + 1 4W relay
        for (uint8_t i = 0; i < 11; i++) {
            // Compile a list of channels, that are to be turned on
            // The scanner card supports up to 20 channels on two separate buses (CH1-CH10 and CH11-CH20),
            // but the DMM might only support 10 channels.
            // We therefore use channels CH1-CH5 and CH11-CH15 on the scanner card and skip CH6-CH10,
            // because CH1-CH5 and CH6-CH10 are connected to the same bus. This is why we use the
            // i + (i/5) * 5 term to skip CH6-CH10.
            // The MSB is the 4W relay, the LSB is CH1

            *relaySetRegister |= !!(command & (1 << scan2000ChannelOnSequence[i])) << (i + (i / 5) * 5);
            // The list of channels, that are to tbe turned off
            *relayUnsetRegister |= !!(command & (1 << scan2000ChannelOffSequence[i])) << (i + (i / 5) * 5);
        }
        rv = decodeOK;
    } else {
        rv = decodeIgnored;
    }
    // Always unset CH6-CH10 and CH-16-CH20
    // There is no need to not set the relaySetRegister, because unsetting a relay takes precedence.
    *relayUnsetRegister |= 0b11111000001111100000;

    return rv;
}

/**
* @brief Decode 20 channel command
 * 48 bits in, so we decode all, even when we do not use all
 * @param command Command received
 * @param relaySetRegister ptr to bit map for setting relays
 * @param relayUnsetRegister ptr to bit map for unsetting relays
 * @return decodeResult_t
 */
decodeResult_t decode_20channels(const uint64_t command, uint32_t *relaySetRegister, uint32_t *relayUnsetRegister) {
    *relaySetRegister = 0x0000;
    *relayUnsetRegister = 0x0000;

    // There is no need to run the loop, if there is nothing to do.
    if (command != 0x00000000) {
        // Process the channels (incl. CH21, 4W mode)
        for (size_t i = 0; i < sizeof(scan2000_20ChannelSequence)/sizeof(scan2000_20ChannelSequence[0]); i++) {
            *relayUnsetRegister |= !!(command & (1 << (2 * (scan2000_20ChannelSequence[i] - 1)))) << i;    // Even clock pulses -> turn relays off
            *relaySetRegister |= !!(command & (1 << (2 * (scan2000_20ChannelSequence[i] - 1) + 1))) << i;  // Odd clock pulses -> turn relays on
        }
        return decodeOK;
    }
    return decodeIgnored;
}
