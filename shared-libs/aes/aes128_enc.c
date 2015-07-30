#include "aes.h"
#include "aes_enc.h"

void aes128_enc(void* buffer, aes128_ctx_t* ctx){
	aes_encrypt_core(buffer, (aes_genctx_t*)ctx, 10);
}

