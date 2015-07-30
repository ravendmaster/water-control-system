#include "aes.h"

void aes128_dec(void* buffer, aes128_ctx_t* ctx){
	aes_decrypt_core(buffer, (aes_genctx_t*)ctx, 10);
}

