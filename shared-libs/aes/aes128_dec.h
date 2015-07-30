#include <stdint.h>

/* aes128_dec.h */
#ifndef AES128_DEC_H_
#define AES128_DEC_H_

#include "aes_types.h"
#include "aes_dec.h"


/**
 * \brief decrypt with 128 bit key.
 *
 * This function decrypts one block with the AES algorithm under control of
 * a keyschedule produced from a 128 bit key.
 * \param buffer pointer to the block to decrypt
 * \param ctx    pointer to the key schedule
 */
void aes128_dec(void* buffer, aes128_ctx_t* ctx);



#endif /* AES128_DEC_H_ */
