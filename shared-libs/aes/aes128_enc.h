/* aes128_enc.h */

#ifndef AES128_ENC_H_
#define AES128_ENC_H_

#include "aes_types.h"
#include "aes_enc.h"


/**
 * \brief encrypt with 128 bit key.
 *
 * This function encrypts one block with the AES algorithm under control of
 * a keyschedule produced from a 128 bit key.
 * \param buffer pointer to the block to encrypt
 * \param ctx    pointer to the key schedule
 */
void aes128_enc(void* buffer, aes128_ctx_t* ctx);



#endif /* AES128_ENC_H_ */
