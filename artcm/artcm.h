#ifndef _ARTCM_H_
#define _ARTCM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ins data struct */
typedef struct 
{
	uint64_t mcut; /* mcu time in us */
	uint64_t gpst; /* gps time in ns */
	uint8_t status;
	double lat;
	double lon;
	float ht;
	float vn;
	float ve;
	float vd;
	float roll;
	float pitch;
	float heading;
	uint8_t zupt;
}ins_t;

/* rtcm basic functions (need to add function for 64 bit) */
uint32_t crc24q(const uint8_t *buff, int len);
void setbitu(uint8_t *buff, int pos, int len, uint32_t data);
void setbits(uint8_t *buff, int pos, int len, int32_t data);
uint32_t getbitu(const uint8_t *buff, int pos, int len);
int32_t getbits(const uint8_t *buff, int pos, int len);

/* encode ins message */
int encode_rtcm3_ins(uint8_t *buff, ins_t *ins);

#ifdef __cplusplus
}
#endif

#endif
