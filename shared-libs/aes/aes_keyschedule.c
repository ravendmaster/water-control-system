/* aes_keyschedule.c */

#include "aes.h"
#include "aes_keyschedule.h"
#include "aes_sbox.h"

static
void aes_rotword(void* a){
	uint8_t t;
	t=((uint8_t*)a)[0];
	((uint8_t*)a)[0] = ((uint8_t*)a)[1];
	((uint8_t*)a)[1] = ((uint8_t*)a)[2];
	((uint8_t*)a)[2] = ((uint8_t*)a)[3];
	((uint8_t*)a)[3] = t;
}

const uint8_t rc_tab[] = { 0x01, 0x02, 0x04, 0x08,
                           0x10, 0x20, 0x40, 0x80,
                           0x1b, 0x36 };

void aes_init(void* key, uint16_t keysize_b, aes_genctx_t* ctx){
	uint8_t hi,i,nk, next_nk;
	uint8_t rc=0;
	union {
		uint32_t v32;
		uint8_t  v8[4];
	} tmp;
	nk=keysize_b>>5; /* 4, 6, 8 */
	hi=4*(nk+6+1);
	memcpy(ctx, key, keysize_b/8);
	next_nk = nk;
	for(i=nk;i<hi;++i){
		tmp.v32 = ((uint32_t*)(ctx->key[0].ks))[i-1];
		if(i!=next_nk){
			if(nk==8 && i%8==4){
				tmp.v8[0] = aes_sbox[(tmp.v8[0])];
				tmp.v8[1] = aes_sbox[(tmp.v8[1])];
				tmp.v8[2] = aes_sbox[(tmp.v8[2])];
				tmp.v8[3] = aes_sbox[(tmp.v8[3])];
			}
		} else {
			next_nk += nk;
			aes_rotword(&(tmp.v32));
			tmp.v8[0] = aes_sbox[(tmp.v8[0])];
			tmp.v8[1] = aes_sbox[(tmp.v8[1])];
			tmp.v8[2] = aes_sbox[(tmp.v8[2])];
			tmp.v8[3] = aes_sbox[(tmp.v8[3])];
			tmp.v8[0] ^= rc_tab[rc];
			rc++;
		}
		((uint32_t*)(ctx->key[0].ks))[i] = ((uint32_t*)(ctx->key[0].ks))[i-nk]
		                                   ^ tmp.v32;
	}
}

void aes128_init(void* key, aes128_ctx_t* ctx){
	aes_init(key, 128, (aes_genctx_t*)ctx);
}

void aes192_init(void* key, aes192_ctx_t* ctx){
	aes_init(key, 192, (aes_genctx_t*)ctx);
}

void aes256_init(void* key, aes256_ctx_t* ctx){
	aes_init(key, 256, (aes_genctx_t*)ctx);
}
