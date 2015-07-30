/* gf256mul.c */
#include "gf256mul.h"

uint8_t gf256mul(uint8_t a, uint8_t b, uint8_t reducer) {
	uint8_t i;
	uint8_t p=0,t;
	for(i=0; i<8; ++i){
		if(b&1)
			p ^= a;
		t=a&0x80;
		a<<=1;
		if(t)
			a ^= reducer;
		b>>=1;
	}
	return p;
}
