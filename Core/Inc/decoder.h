#ifndef DECODER_H
#define DECODER_H

#include <stdint.h>

#define SCAN_2000_ALWAYS_HIGH_BITS ((1 << 10) | (1 << 7))     // 0x0480

typedef enum decodeResult_t {decodeOK = 0, decodeIgnored, decodeDataError, decodeLengthError} decodeResult_t;

decodeResult_t decode_10channels(uint32_t command, uint32_t *relaySetRegister, uint32_t *relayUnsetRegister);
decodeResult_t decode_20channels(uint64_t command, uint32_t *relaySetRegister, uint32_t *relayUnsetRegister);

#endif //DECODER_H
