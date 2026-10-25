/* The network's one translation unit, compiled the way the Neuro N6
 * generated network is compiled: its epoch-controller blobs
 * go to .ext_psram_init, which the startup copies to PSRAM before use.
 * Written by harness/n6model.py. */
#define ECBLOB_CONST_SECTION __attribute__((section(".ext_psram_init")))

#include "palmdet.c"

/* The weights the network was generated against, as model.json records
 * them. They are not in the image: the NPU reads them in place from
 * external flash, so an application can check that the flash still holds
 * them before it runs the network. */
const uint32_t palmdet_weights_address = 0x90300000u;
const uint32_t palmdet_weights_size    = 1122641u;
const uint32_t palmdet_weights_crc32   = 0xEEE62DB9u;
