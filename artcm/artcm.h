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






// Message structs
// 
// Pack struct to 1 byte offsets
#pragma pack(1)
typedef struct
{
	uint16_t message_type;
	uint64_t    MCU_Time;  // UInt64   ns   Time since power on
	uint64_t    Sync_Time; // UInt64   ns   Timestamp of external sync pulse
	uint64_t    ODO_time;  // UInt64   ns   Timestamp of Odometer reading
	int32_t     AX;        // Int32   15 g   X-axis accel
	int32_t     AY;        // Int32   15 g   Y-axis accel
	int32_t     AZ;	       // Int32   15 g   Z-axis accel
	int32_t     WX;        // Int32   450 dps   X-axis angular rate (MEMS)
	int32_t     WY;        // Int32   450 dps   Y-axis angular rate (MEMS)
	int32_t     WZ;        // Int32   450 dps   Z-axis angular rate (MEMS)
	int32_t     OG_WZ;     // Int32   450 dps   High precision z-axis angular rate 
	int16_t     ODO;       // Int16   m/s   Scaled composite odometer value
	int16_t     Temp_C;    // Int16   °C
} RTCM_msg_IMU_t;

// Pack struct to 1 byte offsets
#pragma pack(1)
typedef struct
{
	uint16_t message_type;
	uint64_t    MCU_Time;  // UInt64  ns   Time since power on
	uint64_t    Sync_Time; // UInt64  ns   Timestamp of external sync pulse
	int32_t     AX;	       // Int32	15 g	X-axis accel
	int32_t     AY;	       // Int32	15 g	Y-axis accel
	int32_t     AZ;	       // Int32	15 g	Z-axis accel
	int32_t     WX;	       // Int32	450 dps	X-axis angular rate (MEMS)
	int32_t     WY;	       // Int32	450 dps	Y-axis angular rate (MEMS)
	int32_t     WZ;	       // Int32	450 dps	Z-axis angular rate (MEMS)
	int32_t     OG_WZ;	   // Int32	450 dps	High precision z-axis angular rate 
	int16_t     Temp_C;	   // Int16	°C
} RTCM_msg_IM1_t;

// Pack struct to 1 byte offsets
#pragma pack(1)
typedef struct
{
	uint16_t message_type;
	uint64_t    Time;	        //UInt64	ns	    Time since power on
	uint64_t    GPS_Time;	    //UInt64	ns	    GPS time (GTOW)
	int32_t     Latitude;	    //Int32	    1e-7 deg	
	int32_t     Longitude;	    //Int32	    1e-7 deg	
	int32_t     Alt_ellipsoid;  //Int32	    0.001 m	
	int32_t     Alt_msl;	    //Int32	    0.001 m	
	int32_t     Speed;	        //Int32	    0.001 mps	
	int32_t     Heading;	    //Int32	    0.001 deg	
	uint32_t    Hor_Acc;	    //UInt32	0.001 m	
	uint32_t    Ver_Acc;	    //UInt32	0.001 m	
	uint32_t    Hdg_Acc;	    //UInt32	1e-5 deg	
	uint32_t    Spd_Acc;	    //UInt32	0.001 mps	
	uint16_t    PDOP;	        //UInt16	0.01	
	uint8_t     FixType;	    //UInt8	 	
	uint8_t     SatNum;	        //UInt8	 	
	uint8_t     RTK_Status;	    //UInt8	 	
	uint8_t     Antenna_ID;	    //Uint8	    Primary antenna or 2nd antenna	
} RTCM_msg_PVT_t;

// Pack struct to 1 byte offsets
#pragma pack(1)
typedef struct
{
	uint16_t message_type;
	uint64_t    Time;	        //UInt64	ns
	uint64_t    GPS_Time;	    //UInt64	    ns
	int32_t     relPosN;	    //Int32	    0.001 m
	int32_t     relPosE;	    //Int32	    0.001 m
	int32_t     resPosD;	    //Int32	    0.001 m
	int32_t     relPosLength;	//Int32	    0.001 m
	int32_t     relPosHeading;  //Int32	    1e-5 deg
	uint32_t    relPosLength_Accuracy;  //UInt32    0.1 mm
	uint32_t    relPosHeading_Accuracy; //UInt32    1e-5 deg
	uint16_t    statusFlags;      // uint16
} RTCM_msg_HDR_t;

// Pack struct to 1 byte offsets
#pragma pack(1)
typedef struct
{
	uint16_t message_type;
	uint64_t    Time;	        //UInt64	ns	
	uint64_t    GPS_Time;	    //UInt64	ns	
	int32_t     Latitude;	    //Int32	    1.0e-7 deg	
	int32_t     Longitude;	    //Int32	    1.0e-7 deg	
	int32_t     Alt_ellipsoid;  //Int32	    0.001 m	
	int32_t     Vn;	            //Int32	    0.001 mps	
	int32_t     Ve;	            //Int32	    0.001 mps	
	int32_t     Vd;	            //Int32	    0.001 mps	
	int32_t     Roll;	        //Int32	    1e-5 deg	
	int32_t     Pitch;	        //Int32	    1e-5 deg	
	int32_t     Heading_Yaw;	//Int32	    1e-5 deg	
	uint8_t     ZUPT;	        //UInt8	    1 – stationary, 0 - moving
	uint8_t     Status;	        //UInt8	    See ASCII packet
} RTCM_msg_INS_t;




// Wrapper structs
// 
// Pack struct to 1 byte offsets
#pragma pack(1)
typedef struct
{
	uint8_t preamble;
	// 6-bit 0's 10-bit length
	uint16_t length;

	RTCM_msg_IMU_t msg;

	uint8_t crc[3];

} rtcm_IMU_t;

// Pack struct to 1 byte offsets
#pragma pack(1)
typedef struct
{
	uint8_t preamble;
	// 6-bit 0's 10-bit length
	uint16_t length;

	RTCM_msg_IM1_t msg;

	uint8_t crc[3];

} rtcm_IM1_t;

// Pack struct to 1 byte offsets
#pragma pack(1)
typedef struct
{
	uint8_t preamble;
	// 6-bit 0's 10-bit length
	uint16_t length;

	RTCM_msg_PVT_t msg;

	uint8_t crc[3];

} rtcm_PVT_t;

// Pack struct to 1 byte offsets
#pragma pack(1)
typedef struct
{
	uint8_t preamble;
	// 6-bit 0's 10-bit length
	uint16_t length;

	RTCM_msg_HDR_t msg;

	uint8_t crc[3];

} rtcm_HDR_t;

// Pack struct to 1 byte offsets
#pragma pack(1)
typedef struct
{
	uint8_t preamble;
	// 6-bit 0's 10-bit length
	uint16_t length;

	RTCM_msg_INS_t msg;

	uint8_t crc[3];

} rtcm_INS_t;

typedef union
{
	// Size of this array should be the largest of the structs
	uint8_t packet[sizeof(rtcm_PVT_t)];

	rtcm_IMU_t IMU_msg;
	rtcm_IM1_t IM1_msg;
	rtcm_PVT_t PVT_msg;
	rtcm_HDR_t HDR_msg;
	rtcm_INS_t INS_msg;

} rtcm_msg_t;

enum message_sub_types
{
	IMU_msg = 1,
	PVT_msg = 2,
	HDR_msg = 3,
	INS_msg = 4,
	IM1_msg = 6,
};

#ifdef __cplusplus
}
#endif

#endif
