#include "Msg_B2A.h"

#define OUTPUT_ASCII_BUF_SIZE     1024

char    outputMsgStr[OUTPUT_ASCII_BUF_SIZE+1];

int Verify_Message_preamble(Message_header_t *pMsgHeader)
{
    if(pMsgHeader->preamble0!=MSG_PREAMBLE0 
    || pMsgHeader->preamble1!=MSG_PREAMBLE1 
    )
    {
        return 0;
    }
    else{
        return 1;
    }
}

#if 0
void Calculate_Checksum(char *iBuf, int bytes, char *iCka, char *iCkb)
{
    // The checksum is calculated over the message, starting and including the class field up
    // until, but excluding, the checksum fields (see the figure UBX frame structure).  The
    // checksum values and the input are 8-bit unsigned integers.
    uint8_t cka = 0;
    uint8_t ckb = 0;
    for(int i = 2; i < bytes; i=i+1) {
        cka = cka + ((uint8_t)iBuf[i]); 
        ckb = ckb + cka;
    }

    *iCka = cka & 0xFF;
    *iCkb = ckb & 0xFF;
}
#else
void Calculate_one_byte_Checksum(char *iBuf, int bytes, char *iCka)
{
	// Append checksum to string (an XOR of all the bytes between the $ and the *)
    // 1) Start at the first element of the buffer after $ ((char*)buf + 1)
    // 2) Init sum to zero
    // 3) Continue the loop until null is reached
    // 4) Increment the pointer location and repeat
    // Note: According to the website, deliminators are not included.  Not so in what follows!
    char sum = 0;
	for (int i=0; i<bytes; i++,iBuf++) {
		sum ^= (*iBuf);	// check-sum (xor)
	}

    *iCka = sum;
}
#endif

char * Convert_B2A_GPS_core(void *ipMsg, char *gpsId, int msgSize)
{
    uint64_t         msgContainer[((sizeof(GPSMessage_t)+MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+7)/8)*8];
    GPSMessage_t    *gpsMsg = (GPSMessage_t*)&msgContainer[0];
    char            *p = (char *)outputMsgStr;


    //Copy input message into IMUMessage_t data strcuture
    memcpy((void*)gpsMsg, (char*)ipMsg + MSG_PLAYLOAD_OFFSET, msgSize);

    // Construct the output message, "#APGPS" or "#APGP2"
    p += sprintf(p, "%s,", gpsId);

    // IMU time
    if(gpsMsg->imu_usecCntr >= 1000) {
        gpsMsg->imu_msecCntr = (uint32_t)gpsMsg->imu_msecCntr + 1;
        gpsMsg->imu_usecCntr = (uint32_t)gpsMsg->imu_usecCntr - 1000;
    }
    p += sprintf(p, "%I64lu.%03d,", ((uint64_t)gpsMsg->imu_secCntr * 1000+gpsMsg->imu_msecCntr),
                                (uint32_t)gpsMsg->imu_usecCntr);
    // GPS Time (ITOW)
    uint32_t t_nano_sec     = gpsMsg->t_nano * 1e-9;
    uint32_t t_nano_fracSec = gpsMsg->t_nano - t_nano_sec * 1e9;

    p += sprintf(p, "%u%09u,", t_nano_sec, t_nano_fracSec);

    // Lat
    if(gpsMsg->lat_sign < 0) {
        p += sprintf(p, "-");
    }
    p += sprintf(p, "%d.%07d,", gpsMsg->lat_deg, gpsMsg->lat_frac_deg);
    
    // Lon
    if(gpsMsg->lon_sign < 0) {
        p += sprintf(p, "-");
    }
    p += sprintf(p, "%d.%07d,", gpsMsg->lon_deg, gpsMsg->lon_frac_deg);

    // Altitude
    p += sprintf(p, "%.4f,", gpsMsg->h_ell_m);
    p += sprintf(p, "%.4f,", gpsMsg->h_msl_m);

    // Speed and heading
    p += sprintf(p, "%.4f,", gpsMsg->speed_mps);
    p += sprintf(p, "%.4f,", gpsMsg->heading_motion_deg);

    // Accuracy, HDOP, and PDOP
    p += sprintf(p, "%.4f,", gpsMsg->posAccurEst_m[0]);
    p += sprintf(p, "%.4f,", gpsMsg->posAccurEst_m[1]);
    p += sprintf(p, "%.4f,", gpsMsg->p_dop);

    // Fix-Type and number of satellites
    p += sprintf(p, "%d,", gpsMsg->fixType);
    p += sprintf(p, "%d,", gpsMsg->nSats);
    
    // Extra bits for my sim*
    p += sprintf(p, "%.4f,", gpsMsg->speedAccur_mps);
    p += sprintf(p, "%.4f,", gpsMsg->headingAccur_deg);

    // fix-type (RTK)
    // warning: ‘sprintf’ may write a terminating nul past the end of the destination
    p += sprintf(p, "%u", gpsMsg->carrSoln);

    //Convert MCU system to GPS-synchronized time
    uint64_t newTime = gpsMsg->imu_time_t_nano_new;

    uint32_t newTimeSec = (uint64_t)(newTime*1e-9);
    newTime = newTime - ((uint64_t)newTimeSec*1e+9);
    p += sprintf(p, ",%u%09u", newTimeSec, (uint32_t)newTime);

    // Append a separator, checksum, and EOL characters (CR + NL)
    char chk;// = (unsigned char*)ipMsg + MSG_PLAYLOAD_OFFSET + msgSize;
    Calculate_one_byte_Checksum(&outputMsgStr[1], (int)(p-outputMsgStr)-1, &chk);
    p += sprintf(p, "*%02x%c%c", chk, 0x0D, 0x0A);

    p[0] = '\0';

    return((char*)outputMsgStr);
}

char * Convert_B2A_GPS(void *ipMsg, int msgSize)
{
    return Convert_B2A_GPS_core(ipMsg, "#APGPS", msgSize);
}

char * Convert_B2A_GP2(void *ipMsg, int msgSize)
{
    return Convert_B2A_GPS_core(ipMsg, "#APGP2", msgSize);
}

char * Convert_B2A_IMU(void *ipMsg, int msgSize)
{
    uint64_t         msgContainer[((sizeof(IMUMessage_t)+MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+7)/8)*8];
    IMUMessage_t    *imuMsg = (IMUMessage_t*)&msgContainer[0];
    char             *p = (char *)outputMsgStr;

    p += sprintf(p, "#APIMU,");

    //Copy input message into IMUMessage_t data strcuture
    memcpy((void*)imuMsg, (char*)ipMsg + MSG_PLAYLOAD_OFFSET, msgSize);

    // Adjust time if the usec counter is 1000 or more
    if(imuMsg->usecCntr >= 1000) {
        imuMsg->msecCntr = (uint32_t)imuMsg->msecCntr + 1;
        imuMsg->usecCntr = (uint32_t)imuMsg->usecCntr - 1000;
    }
    
    // Construct output message (null-terminated string)
    //                       -----------  MEMS  -----------
    //               ---t---  ax   ay   az   wx   wy   wz
    p += sprintf(p, "%I64lu.%03d,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,",
                        ((uint64_t)imuMsg->secCntr * 1000 + imuMsg->msecCntr), imuMsg->usecCntr,
                        imuMsg->accel_g[X_AXIS],  imuMsg->accel_g[Y_AXIS], imuMsg->accel_g[Z_AXIS],
                        imuMsg->angRate_degPerSec[X_AXIS], imuMsg->angRate_degPerSec[Y_AXIS], imuMsg->angRate_degPerSec[Z_AXIS]);

    // Append FOG rates
    if(imuMsg->numOfFogs == 1) {
        p += sprintf(p, "%.5f,", imuMsg->vg1703_angRate[0]);
    } else {
        for(unsigned int fogNum = 0; fogNum < imuMsg->numOfFogs; fogNum++) {
            p += sprintf(p, "%.5f,", imuMsg->vg1703_angRate_combined[fogNum]);
        }
    }

    // Check odometer time for rollover and update
    if(imuMsg->odometer_time_usecCntr >= 1000) {
        imuMsg->odometer_time_msecCntr = imuMsg->odometer_time_msecCntr + 1;
        imuMsg->odometer_time_usecCntr = imuMsg->odometer_time_usecCntr - 1000;
    }
    
    // Append odometer speed and time
    // FIXME: speed_mps stays constant if algorithm is disabled
    p += sprintf(p, "%.4f,%d.%03d,",                       
                imuMsg->odometer_speed_mps[0],
                (imuMsg->odometer_time_secCntr * 1000 + imuMsg->odometer_time_msecCntr), 
                imuMsg->odometer_time_usecCntr);
    
    // Append temperature         
    p += sprintf(p, "%.4f,",
                imuMsg->temp_scaledValues);

    //Convert MCU system to GPS-synchronized time
    uint64_t newTime = imuMsg->time_t_nano_new;
    uint64_t newTime_sec = (uint64_t)(newTime*1e-9);
    newTime = newTime - ((uint64_t)newTime_sec*1e+9);
    newTime = (newTime+500)/1000/1000;
    p += sprintf(p, "%I64lu.%03u", newTime_sec, (uint32_t)newTime);

    // Append a separator, checksum, and EOL characters (CR + NL)
    char chk;// = (unsigned char*)ipMsg + MSG_PLAYLOAD_OFFSET + msgSize;
    Calculate_one_byte_Checksum(&outputMsgStr[1], (int)(p-outputMsgStr)-1, &chk);
    p += sprintf(p, "*%02x%c%c", chk, 0x0D, 0x0A);

    p[0] = '\0';

    //
    return((char*)outputMsgStr);
}

char * Convert_B2A_INS(void *ipMsg, int msgSize)
{
    uint64_t         msgContainer[((sizeof(INSMessage_t)+MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+7)/8)*8];
    INSMessage_t     *insMsg = (INSMessage_t*)&msgContainer[0];
    char             *p = (char *)outputMsgStr;

    //Copy input message into GPSMessage_t data strcuture
    memcpy((void*)insMsg, (char*)ipMsg + MSG_PLAYLOAD_OFFSET, msgSize);

    // Construct the output INS message
    p += sprintf(p, "#APINS,");
    
    // IMU time
    p += sprintf(p, "%d,", insMsg->secCntr * 1000 + insMsg->msecCntr);
    
    // GPS Time (ITOW) -- replace with nano time
    if (insMsg->GetPositionInitReady == 1) {
        uint32_t t_nano_sec     = insMsg->t_nano * 1e-9;
        uint32_t t_nano_fracSec = insMsg->t_nano - t_nano_sec * 1e9;

        p += sprintf(p, "%u%09u,", t_nano_sec, t_nano_fracSec);

    } else {
        p += sprintf(p, ",");
    }

    if((insMsg->GetAttitudeInitReady== 1)) {
        if(insMsg->GetPositionInitReady == 1) {
            if(insMsg->HeadingReady == 1) {
                if(insMsg->carrSoln > 0) {
                    //1 for RTK float and 2 for RTK fixed
                    if(insMsg->carrSoln == 1) {
                        p += sprintf(p, "3,");
                    } else {
                        p += sprintf(p, "4,");
                    }
                } else {
                    p += sprintf(p, "2,");
                }
            } else {
                p += sprintf(p, "1,");
            }
        } else {
            p += sprintf(p, "0,");
        }
    } else {
        p += sprintf(p, ",");
    }

    //
    if (insMsg->GetPositionInitReady == 1) {
        // Output position
        p += sprintf(p, "%.12f,", (float)(insMsg->r_N[0]));
        p += sprintf(p, "%.12f,", (float)(insMsg->r_N[1]));
        p += sprintf(p, "%.12f,", insMsg->r_N[2]);
    } else {
        // Output position
        p += sprintf(p, ",");
        p += sprintf(p, ",");
        p += sprintf(p, ",");
    }

    // GPS initialized here
    if (insMsg->HeadingReady == 1) {
        // Output velocity
#if 0
        // ENU
        p += sprintf(p, "%.6f,", insMsg->v_N[0]);
        p += sprintf(p, "%.6f,", insMsg->v_N[1]);
        p += sprintf(p, "%.6f,", insMsg->v_N[2]);
#else
        // NED
        p += sprintf(p, "%.6f,",  insMsg->v_N[1]);
        p += sprintf(p, "%.6f,",  insMsg->v_N[0]);
        p += sprintf(p, "%.6f,", -insMsg->v_N[2]);
#endif
    } else {
        // Output velocity
        p += sprintf(p, ",");
        p += sprintf(p, ",");
        p += sprintf(p, ",");
    }

    // 
    if (insMsg->GetAttitudeInitReady == 1) {
        // Output attitude
#if 0
        // ENU
        p += sprintf(p, "%.6f,", insMsg->theta_BinN[0]);
        p += sprintf(p, "%.6f,", insMsg->theta_BinN[1]);
        p += sprintf(p, "%.6f,", insMsg->theta_BinN[2]);
#else
        // NED
        p += sprintf(p, "%.6f,",  (float)(insMsg->theta_BinN[1]));
        p += sprintf(p, "%.6f,",  (float)(insMsg->theta_BinN[0]));
        p += sprintf(p, "%.6f,", (float)(-insMsg->theta_BinN[2]));
#endif
    } else {
        // Output attitude
        p += sprintf(p, ",");
        p += sprintf(p, ",");
        p += sprintf(p, ",");
    }

    if(insMsg->Algo_GetZuptUpdateFlag) {
        p += sprintf(p, "1");
    } else {
        p += sprintf(p, "0");
    }

    //Convert MCU system to GPS-synchronized time
    uint64_t newTime = insMsg->t_nano_new;
    uint32_t newTimeSec = (uint64_t)(newTime*1e-9);
    newTime = newTime - ((uint64_t)newTimeSec*1e+9);
    p += sprintf(p, ",%u%09u", newTimeSec, (uint32_t)newTime);

    // Append a separator, checksum, and EOL characters (CR + NL)
    char chk;// = (unsigned char*)ipMsg + MSG_PLAYLOAD_OFFSET + msgSize;
    Calculate_one_byte_Checksum(&outputMsgStr[1], (int)(p-outputMsgStr)-1, &chk);
    p += sprintf(p, "*%02x%c%c", chk, 0x0D, 0x0A);

    p[0] = '\0';

    return((char*)outputMsgStr);

}


char * Convert_B2A_CAL(void *ipMsg, int msgSize)
{
    uint64_t         msgContainer[((sizeof(CALMessage_t)+MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE+7)/8)*8];
    CALMessage_t    *calMsg = (CALMessage_t*)&msgContainer[0];
    char             *p = (char *)outputMsgStr;

    // Add preamble to the message
    p += sprintf(p, "#APCAL,");

    //Copy input message into IMUMessage_t data strcuture
    memcpy((void*)calMsg, (char*)ipMsg + MSG_PLAYLOAD_OFFSET, msgSize);

    // Adjust time if the usec counter is 1000 or more
    if(calMsg->sTime_t_usecCntr >= 1000) {
        calMsg->sTime_t_msecCntr = calMsg->sTime_t_msecCntr + 1;
        calMsg->sTime_t_usecCntr = calMsg->sTime_t_usecCntr - 1000;
    }
    
    // Construct output message (null-terminated string)
    //                       ----- MEMS #1 -----  ----- MEMS #2 -----  ----- MEMS #3 -----
    //               ---t--- ax ay az wx wy wz T  ax ay az wx wy wz T  ax ay az wx wy wz T
    p += sprintf(p, "%d.%03d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,",
                    // Time Data
                    (calMsg->sTime_t_secCntr * 1000 + calMsg->sTime_t_msecCntr), calMsg->sTime_t_usecCntr,

                    // MEMS #1
                    calMsg->accel_rawCounts1[X_AXIS],   calMsg->accel_rawCounts1[Y_AXIS],   calMsg->accel_rawCounts1[Z_AXIS],
                    calMsg->angRate_rawCounts1[X_AXIS], calMsg->angRate_rawCounts1[Y_AXIS], calMsg->angRate_rawCounts1[Z_AXIS],
                    calMsg->temp_rawCounts1,

                    // MEMS #2
                    calMsg->accel_rawCounts2[X_AXIS],   calMsg->accel_rawCounts2[Y_AXIS],   calMsg->accel_rawCounts2[Z_AXIS],
                    calMsg->angRate_rawCounts2[X_AXIS], calMsg->angRate_rawCounts2[Y_AXIS], calMsg->angRate_rawCounts2[Z_AXIS],
                    calMsg->temp_rawCounts2,

                    // MEMS #3
                    calMsg->accel_rawCounts3[X_AXIS],   calMsg->accel_rawCounts3[Y_AXIS],   calMsg->accel_rawCounts3[Z_AXIS],
                    calMsg->angRate_rawCounts3[X_AXIS], calMsg->angRate_rawCounts3[Y_AXIS], calMsg->angRate_rawCounts3[Z_AXIS],
                    calMsg->temp_rawCounts3);

    //put fog counts for each one sensor configuration
    int16_t  numOfFogs = calMsg->numOfFogs;
    for(unsigned int fogNum = 0; fogNum < numOfFogs; fogNum++) {
        p += sprintf(p, "%d,%d", calMsg->vg1703_counts[fogNum], calMsg->vg1703_temp_counts[fogNum]);
        if(fogNum == numOfFogs - 1 ) {
            //no comma after last one
        } else {
            p += sprintf(p, ",");
        }
    }

    // Append a separator, checksum, and EOL characters (CR + NL)
    char chk;// = (unsigned char*)ipMsg + MSG_PLAYLOAD_OFFSET + msgSize;
    Calculate_one_byte_Checksum(&outputMsgStr[1], (int)(p-outputMsgStr)-1, &chk);
    p += sprintf(p, "*%02x%c%c", chk, 0x0D, 0x0A);

    p[0] = '\0';

    return((char*)outputMsgStr);

}

char * Convert_Message_B2A(void *ipMsg)
{
    Message_header_t *pMsgHeader = (Message_header_t*)ipMsg;
    char             ck_a, *cPtr;
    uint16_t         length;
    

    if( 0 == Verify_Message_preamble(pMsgHeader) )
    {
        strcpy((char*)outputMsgStr, "Wrong message preamble!\n");
        return (char*)outputMsgStr;
    }

    cPtr = (char*)ipMsg;
    length = (cPtr[4]&0x00FF) | cPtr[5];
#if 0
    char    ck_b;
    Calculate_Checksum((char *)ipMsg, MSG_HEADER_SIZE+length, &ck_a, &ck_b);
    if(cPtr[MSG_HEADER_SIZE+length]!=ck_a || cPtr[MSG_HEADER_SIZE+length+1]!=ck_b)
    {
        strcpy((char*)outputMsgStr, "Wrong message checksum\n");
        return (char*)outputMsgStr;
    }
#else
    Calculate_one_byte_Checksum((char *)ipMsg, MSG_HEADER_SIZE+length, &ck_a);
    if(cPtr[MSG_HEADER_SIZE+length]!=ck_a)
    {
        strcpy((char*)outputMsgStr, "Wrong message checksum\n");
#ifdef _DEBUG
        printf("Wrong message checksum\n");
#endif
        return (char*)outputMsgStr;
    }
#endif



#if 0
    uint16_t         msgSize;
    msgSize = length + (MSG_HEADER_SIZE+MSG_CHECKSUM_SIZE);

    unsigned char *sPtr = (unsigned char*)ipMsg;
    int  i;

//    if(sPtr[3]==MSG_ID_APGPS || sPtr[3]==MSG_ID_APGP2 )
    if(sPtr[3]==MSG_ID_APINS )
    {
        for(i=0; i<msgSize/8; i++, sPtr+=8)
        {
            printf("0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
                    sPtr[0], sPtr[1], sPtr[2], sPtr[3], sPtr[4], sPtr[5], sPtr[6], sPtr[7]);
        }
        for(i=i*8; i<msgSize; i++, sPtr++)
        {
            printf("0x%02x, ", sPtr[0]);
        }
        printf("\n");
    }
#endif

    char msgId1 = pMsgHeader->msgId1;
    char msgId2 = pMsgHeader->msgId2;
    
    if(msgId1==MSG_ID1_APGPS && msgId2==MSG_ID2_APGPS){
            return Convert_B2A_GPS(ipMsg, length);
    }
    else if(msgId1==MSG_ID1_APGP2 && msgId2==MSG_ID2_APGP2){
            return Convert_B2A_GP2(ipMsg, length);
    }
    else if(msgId1==MSG_ID1_APIMU && msgId2==MSG_ID2_APIMU){
            return Convert_B2A_IMU(ipMsg, length);
    }
    else if(msgId1==MSG_ID1_APINS && msgId2==MSG_ID2_APINS){
            return Convert_B2A_INS(ipMsg, length);
    }
    else if(msgId1==MSG_ID1_APCAL && msgId2==MSG_ID2_APCAL){
            return Convert_B2A_CAL(ipMsg, length);
    }

    return "";
}