#include <stdint.h>

typedef struct 
{
	uint32_t token;
	
	uint8_t data[28];
	
} EncryptedBlock;

typedef struct
{
	uint16_t air_interval;
//	uint16_t probe_accident_level;
}ControllerData;

typedef struct
{
	uint16_t sensor_val;
}ProbeData;

