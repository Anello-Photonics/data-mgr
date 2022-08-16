
#ifndef _MESSAGE_DEF_H
#define _MESSAGE_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>  /* uint8_t, ... */
#include <string.h>
#include <stdio.h>


#ifndef NUM_OF_CAL_SECTORS_MEMS
#define NUM_OF_CAL_SECTORS_MEMS  (6)
#endif

#ifndef NUM_OF_CAL_SECTORS_FOG
#define NUM_OF_CAL_SECTORS_FOG   (6)
#endif

// List the possible output messages
#define  NO_OUTPUT_MSG                0

// ASCII messages are defined below
#define  SCALED_SENSOR_VALUES_ASCII   1
#define  RAW_SENSOR_VALUES_ASCII      2
#define  EKF_SOLUTION_ASCII           3

#define  ID_ASCII                     31
#define  SN_ASCII                     32
#define  FW_VER_ASCII                 33

#define  TEST_MESSAGE_ASCII           49

// Binary messages are defined below
#define  SCALED_SENSOR_VALUES_BINARY  51
#define  RAW_SENSOR_VALUES_BINARY     52
#define  EKF_SOLUTION_BINARY          53

#define  ID_BINARY                    81
#define  SN_BINARY                    82
#define  FW_VER_BINARY                83

#define  TEST_MESSAGE_BINARY          99

#define  ASCII_MSG_END                TEST_MESSAGE_ASCII
#define  BINARY_MSG_END               TEST_MESSAGE_BINARY


/*
               Anello binary message format
  
             |-------------|--------|--------|------------|----------------|---------------|
             |   preamble  | class  |   ID   | length (N) |     payload    |   checksum    |
             |-------------|--------|--------|------------|----------------|---------------|
             |  (2 bytes)  | 1 byte | 1 byte |  uint16_t  |    (N bytes)   |   (2 bytes)   |
             |------|------|--------|--------|-----|------|----------------|-------|-------|
             | 0xAA | 0xBB |        |        | lo  |  hi  | data structure | chk_a | chk_b |
             |------|------|--------|--------|-----|------|----------------|-------|-------|
byte offset: 0      1      2        3        4     5      6                6+N     7+N
                           |-----------------------------------------------|
                           range over which the check sum is to be calculated

 The calculation of check sum:
    uint16_t CK_A = 0, CK_B = 0;
    for (i = 2; i < N+6; i++)
    {
        CK_A = CK_A + buffer[i];
        CK_B = CK_B + CK_A;
    }
    CH_A &= 0x00FF;//keep low 8 bits
    CH_B &= 0x00FF;//keep low 8 bits
 */

#define MSG_PREAMBLE_SIZE               2   //bytes
#define MSG_TYPE_SIZE                   2   //bytes
#define MSG_LENGTH_SIZE                 2   //bytes
#define MSG_HEADER_SIZE                 (MSG_PREAMBLE_SIZE+MSG_TYPE_SIZE+MSG_LENGTH_SIZE)

#define MSG_CHECKSUM_SIZE               2   //bytes

#define MGS_CHECKSUM_START_OFFSET       MSG_PREAMBLE_SIZE
#define MSG_PLAYLOAD_OFFSET             MSG_HEADER_SIZE

#define MSG_PREAMBLE0                   (char)'A'
#define MSG_PREAMBLE1                   (char)'P'

//define message ID
#define MSG_ID1_APGPS                   (char)'G'
#define MSG_ID2_APGPS                   (char)'P'

#define MSG_ID1_APGP2                   (char)'G'
#define MSG_ID2_APGP2                   (char)'2'

#define MSG_ID1_APIMU                   (char)'I'
#define MSG_ID2_APIMU                   (char)'M'

#define MSG_ID1_APINS                   (char)'I'
#define MSG_ID2_APINS                   (char)'N'

#define MSG_ID1_APCAL                   (char)'C'
#define MSG_ID2_APCAL                   (char)'A'

#define MSG_ID1_APANT1                  (char)'A'
#define MSG_ID2_APANT1                  (char)'1'

#define MSG_ID1_APANT2                  (char)'A'
#define MSG_ID2_APANT2                  (char)'2'

#define MSG_ID1_APGGA                   (char)'G'
#define MSG_ID2_APGGA                   (char)'G'


#ifndef NUM_OF_CAL_SECTORS_FOG
#define NUM_OF_CAL_SECTORS_FOG          6
#endif

#ifndef NUM_OF_AXES
#define NUM_OF_AXES                     3
#endif


typedef struct{
    char    preamble0;
    char    preamble1;

    char    msgId1;
    char    msgId2;

    union{
        struct{
            char    lengthL;
            char    lengthH;
        };
        uint16_t    length;     //in bytes
    };
}Message_header_t;

//Important notes:
//1. The elements of int64 must be allocated at the pultiple of 8 bytes.
//2. The elements of float and int32 must be allocated at the pultiple of 4 bytes.
//3. The elements of int16 must be allocated at the pultiple of 2 bytes.
typedef struct{
    // GPS Time (ITOW)
    uint64_t    t_nano;

    //GPS-synchronized IMU time
    uint64_t    imu_time_t_nano_new;

    // Altitude
    float       h_ell_m;
    float       h_msl_m;

    // Speed and heading
    float       speed_mps;
    float       heading_motion_deg;

    // Accuracy, HDOP, and PDOP
    float       posAccurEst_m[2];
    float       p_dop;

    // Extra bits for my sim
    float       speedAccur_mps;
    float       headingAccur_deg;

    // IMU time
    uint32_t    imu_secCntr;
    uint32_t    imu_msecCntr;
    uint32_t    imu_usecCntr;
    uint32_t    imu_nsecCntr;

    // Lat and Lon
    uint32_t    lat_frac_deg;
    uint32_t    lon_frac_deg;

    // Lat and Lon
    uint16_t    lat_deg;
    uint16_t    lon_deg;

    // Lat and Lon
    int8_t     lat_sign;
    int8_t     lon_sign;

    // Fix-Type and number of satellites
    uint8_t     fixType;
    uint8_t     nSats;

    // fix-type (RTK)
    uint8_t     carrSoln;

    uint8_t     msgEnd;
}GPSMessage_t;

typedef struct{
    // FIXME: speed_mps stays constant if algorithm is disabled
    uint64_t    time_t_nano_new;

    float       accel_g[3];
    float       angRate_degPerSec[3];

    //output rates for each FOG in order
    float       vg1703_angRate[NUM_OF_CAL_SECTORS_FOG];
    float       vg1703_angRate_combined[3];

    //odometer, time, temperature
    float       odometer_speed_mps[4];
    float       temp_scaledValues;

    //vg1703_GetAngRate_degPerSec(vg1703, )

    // Construct output message (null-terminated string)
    //                         t       ax   ay   az   wx   wy   wz
    uint32_t    secCntr, msecCntr, usecCntr;

    //output rates for each FOG in order
    uint32_t    numOfFogs;

    //odometer, time, temperature
    uint32_t    odometer_time_secCntr;
    uint32_t    odometer_time_msecCntr;
    uint32_t    odometer_time_usecCntr;

    uint8_t     msgEnd;
}IMUMessage_t;


typedef struct{    
    // GPS Time (ITOW) -- replace with nano time
    uint64_t    t_nano;

    //Convert MCU system to GPS-synchronized time
    uint64_t    t_nano_new;

    double      r_N[3];

    // IMU time
    uint32_t    secCntr, msecCntr;

    float       v_N[3];
    float       theta_BinN[3];

    uint8_t     carrSoln;

    //
    uint8_t     GetPositionInitReady;

    // GPS initialized here
    uint8_t     HeadingReady;

    // 
    uint8_t     GetAttitudeInitReady;

    uint8_t     Algo_GetZuptUpdateFlag;

    uint8_t     msgEnd;
}INSMessage_t;

typedef struct{
    uint32_t        sTime_t_secCntr;
    uint32_t        sTime_t_msecCntr;
    uint32_t        sTime_t_usecCntr;

    int32_t         vg1703_counts[NUM_OF_CAL_SECTORS_FOG]; //use max, constant number so it can work at compile
    int16_t         vg1703_temp_counts[NUM_OF_CAL_SECTORS_FOG];

    int16_t         accel_rawCounts1[NUM_OF_AXES];
    int16_t         angRate_rawCounts1[NUM_OF_AXES];
    int16_t         temp_rawCounts1;

    int16_t         accel_rawCounts2[NUM_OF_AXES];
    int16_t         angRate_rawCounts2[NUM_OF_AXES];
    int16_t         temp_rawCounts2;

    int16_t         accel_rawCounts3[NUM_OF_AXES];
    int16_t         angRate_rawCounts3[NUM_OF_AXES];
    int16_t         temp_rawCounts3;

        //put fog counts for each one sensor configuration
    int16_t         numOfMems;
    int16_t         numOfFogs;

    uint8_t         msgEnd;
}CALMessage_t;

void Calculate_Checksum(char *iBuf, int bytes, char *iCka, char *iCkb);
void Calculate_one_byte_Checksum(char *iBuf, int bytes, char *iCka);
int Verify_Checksum(char *iBuf, int bytes, char cka, char ckb);
void Output_Message(void        *dataChannelDataPipe,
                    uint8_t     msgClass,
                    uint8_t     msgID,
                    char        *binMsg,
                    int16_t     bytes);

#ifdef __cplusplus
}
#endif

#endif /* OUTPUT_MESSAGING_H */
