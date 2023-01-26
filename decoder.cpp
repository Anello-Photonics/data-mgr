// decoder.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>
#include <string>
#include <algorithm>

#include "NComRxC.h"
#include "Message_def.h"

#ifndef PI
#define	PI 3.14159265358979
#endif

#ifndef D2R
#define D2R (PI/180.0)
#endif

#ifndef R2D
#define R2D (180.0/PI)
#endif

#ifndef TWOPI
#define	TWOPI (PI+PI)
#endif

#ifndef ae_WGS84
#define ae_WGS84 6378137.0
#endif

#ifndef finv_WGS84
#define finv_WGS84 298.257223563
#endif

#ifndef grav_WGS84
#define	grav_WGS84 9.7803267714e0
#endif

typedef struct
{
	float timeimu;
	float fxyz[3];
	float wxyz[3];
	float countodr;
	float timeodr;
	float timegps;
	float weeksec;
	double lat;
	double lon;
	float ht;
	float msl;
	float speed;
	float yaw;
	float hdop;
	float pdop;
	float accuracy;
	uint8_t fixtype;
	uint8_t nsat;
	uint8_t pvtupdate; /* set to 1 if there are new GPS, set to 0 after */
}pvt_imu_dat_t;

#define MAXFIELD 100



/*****************************************
 * returns true if the checksum is correct
 * @param buf
 * @return true - if checksum is correct
 */
static int verify_checksum(char* buf) {
	char* q, * msg_ck, sum;
	char ck[10] = { '\0' };
	int i = 0;

	for (q = (char*)buf + 1, sum = 0; *q != '*' && *q; q++) {
		sum ^= *q;
	}

	sprintf(ck, "%02X", sum);

	msg_ck = strchr(buf, '*') + 1;
	return (msg_ck[0] == ck[0] && msg_ck[1] == ck[1]) ? 1 : 0;
}


/******************************************************
 * Parse Fields
 * @param buffer
 * @param val
 * @return
 */
static int parse_fields(char* const buffer, char** val)
{
	char* p, *q;
	int n = 0;

	/* parse fields */
	for (p = buffer; *p && n < MAXFIELD; p = q + 1) {
		if (p == NULL) break;
		if ((q = strchr(p, ',')) || (q = strchr(p, '*')) || (q = strchr(p, '\n')) || (q = strchr(p, '\r'))) {
			val[n++] = p; *q = '\0';
		}
		else break;
	}
	return n;
}


/*************************************************************
 * Parse Fields Data
 * @param buffer
 * @param data
 * @return
 */
static int parse_fields_data(char* const buffer, double* data)
{
	char* val[MAXFIELD];
	int n = parse_fields(buffer, val);
	for (int i = 0; i < n; ++i)
		data[i] = atof(val[i]);
	return n;
}


/***************************************************************
 * Set Output File
 * @param fname
 * @param key
 * @return Returns a file descriptor
 */
// dwg - outfilename size changed to 257 to suppress warning
static FILE* set_output_file(const char* fname, const char* key)
{
	char filename[255] = { 0 }, outfilename[257] = { 0 };
	strcpy(filename, fname);
	char* temp = strrchr(filename, '.');
	if (temp) temp[0] = '\0';
	sprintf(outfilename, "%s-%s", filename, key);
	return fopen(outfilename, "w");
}


/*****************************************
 * Print Log
 * @param val
 * @param num
 */
static void print_log(char** val, int num)
{
	for (int i = 0; i < num; ++i)
		printf("%s%c", val[i], (i + 1) ==
                         num ? '\n' : (i + 2) ==
                         num ? '*' : ',');
}


/**************************************************************
 * Degrees to Degrees Minutes and Seconds
 * @param deg
 * @param dms
 */
static void deg2dms(double deg, double* dms)
{
	double sign = deg < 0.0 ? (-1.0) : (1.0), a = fabs(deg);
	dms[0] = floor(a); a = (a - dms[0]) * 60.0;
	dms[1] = floor(a); a = (a - dms[1]) * 60.0;
	dms[2] = a; dms[0] *= sign;
}


/******************************************
 * Output an nmea gga sentence
 * @param buff
 * @param time
 * @param type
 * @param blh
 * @param ns
 * @param dop
 * @param age
 * @return Returns the size of the output string not including the zero byte
 */
extern int outnmea_gga(unsigned char* buff,
                       float time, int type,
                       double* blh, int ns,
                       float dop, float age)
{
	double h, ep[6], dms1[3], dms2[3];
	char* p = (char*)buff, * q, sum;

	if (type != 1 && type != 4 && type != 5) {
		p += sprintf(p, "$GPGGA,,,,,,,,,,,,,,");
		for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q;
		p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
		return (int)(p - (char*)buff);
	}
	time -= 18.0;
	ep[2] = floor(time / (24 * 3600));
	time -= (float)(ep[2] * 24 * 3600.0);
	ep[3] = floor(time / 3600);
	time -= (float)(ep[3] * 3600);
	ep[4] = floor(time / 60);
	time -= (float)(ep[4] * 60);
	ep[5] = time;
	h = 0.0;
	deg2dms(fabs(blh[0]) * 180 / PI, dms1);
	deg2dms(fabs(blh[1]) * 180 / PI, dms2);
	p += sprintf(p,
                 "$GPGGA,"
                 "%02.0f%02.0f%05.2f,"
                 "%02.0f%010.7f,"
                 "%s,"
                 "%03.0f%010.7f,"
                 "%s,"
                 "%d,"
                 "%02d,"
                 "%.1f,"
                 "%.3f,"
                 "M,"
                 "%.3f,"
                 "M,"
                 "%.1f,",
		        ep[3], ep[4], ep[5], dms1[0],
                dms1[1] + dms1[2] / 60.0, blh[0] >= 0 ? "N" : "S",
		        dms2[0], dms2[1] + dms2[2] / 60.0, blh[1] >= 0 ? "E" : "W",
                type, ns, dop, blh[2] - h, h, age);

    // dwg  this next line of code makes me nervous ;-(
	for (q = (char*)buff + 1, sum = 0; *q; q++) {

        sum ^= *q; /* check-sum */
    }
	p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
	return (int)(p - (char*)buff);
}

/*
 * Since this program is standalone and only includes it's own files,
 * it's odd that it doesn't know whether MAX_BUF_LEN has been defined
 * elsewhere.
 */
#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN 4096
#endif

typedef struct
{
	uint8_t dat[MAX_BUF_LEN];
	uint32_t nbyte;
}nmea_buff_t;

static int add_buff(nmea_buff_t* buff, uint8_t data)
{
	int ret = 0;
	if (buff->nbyte >= MAX_BUF_LEN) {
        buff->nbyte = 0;
    }
	if (buff->nbyte == 0)
	{
		memset(buff, 0, sizeof(nmea_buff_t));
		if (data == '#')
		{
			buff->dat[buff->nbyte++] = data;
		}
	}
	else
	{
		buff->dat[buff->nbyte++] = data;
	}
	if (buff->nbyte > 2 && buff->dat[buff->nbyte - 1] == '\n' && buff->dat[buff->nbyte - 2] == '\r')
	{
		ret = 1;
	}
	else if (buff->nbyte > 4 && buff->dat[buff->nbyte - 1] == '\n' && buff->dat[buff->nbyte - 4] == '*')
	{
		ret = 1;
	}
	else if (buff->nbyte > 4 && buff->dat[buff->nbyte - 1] == '\r' && buff->dat[buff->nbyte - 4] == '*')
	{
		ret = 1;
	}
	return ret;
}


/***************************************************************
 * Find the time offset using #APGPS
 * @param imufname
 * @param time_offset
 * @return returns either a 0 or a 1
 */
/* find the time offset using the #APGPS */
int found_time_offset(const char* imufname, double *time_offset)
{
	std::vector< double> timeoffset;
	FILE *fIMU = fopen(imufname, "rb"); if (fIMU == NULL) return 0;

	char* val[MAXFIELD];

	uint8_t data = 0;
	nmea_buff_t buff = { 0 };
	while (fIMU != NULL && !feof(fIMU))
	{
		if ((data = fgetc(fIMU)) == EOF) {
            break;
        }
		if (!add_buff(&buff, data)) {
            continue;
        }

		if (strstr((char*)buff.dat, "#APGPS") != NULL)
		{
			double gpsdata[20] = { 0 };
			int num = parse_fields((char*)buff.dat, val);
			if (num > 16)
			{
				/* real-time GPS solution */
				gpsdata[0] = atof(val[1]);  /* IMU time */
				gpsdata[1] = atof(val[2]);  /* GPS nsec */
				gpsdata[2] = atof(val[3]);  /* latitude -- deg*/
				gpsdata[3] = atof(val[4]);  /* longitude -- deg*/
				gpsdata[4] = atof(val[5]);  /* ht */
				gpsdata[5] = atof(val[6]);  /* MSL */
				gpsdata[6] = atof(val[7]);  /* speed [m/s] */
				gpsdata[7] = atof(val[8]);  /* heading [deg] */
				gpsdata[8] = atof(val[9]);  /* hor. accuracy [m] */
				gpsdata[9] = atof(val[10]); /* ver. accuracy [m] */
				gpsdata[10] = atof(val[11]); /* PDOP */
				gpsdata[11] = atof(val[12]); /* fixType */
				gpsdata[12] = atof(val[13]); /* sat number */
				gpsdata[13] = atof(val[14]); /* speed accur */
				gpsdata[14] = atof(val[15]); /* heading accur */
				gpsdata[15] = atof(val[16]); /* rtk fix status */
				if (gpsdata[12] > 0)
				{
					double cur_time_offset = gpsdata[1] * 1.0e-9 - gpsdata[0] * 1.0e-3;
					timeoffset.push_back(cur_time_offset);
				}
			}
		}
		buff.nbyte = 0;
	}
	if (fIMU) fclose(fIMU);
	if (timeoffset.size() > 0)
	{
		std::sort(timeoffset.begin(), timeoffset.end());
		int loc = timeoffset.size() / 2;
		*time_offset = timeoffset[loc];
		return 1;
	}
	else
	{
		return 0;
	}
}

/* septentrio asc solution
* -1: PVTGeodetic block
Col1:  -1
Col2:  time (GPS second since Jan 06, 1980)
Col3:  Latitude in radians, or -20000000000 if not available
Col4:  Longitude in radians, or -20000000000 if not available
Col5:  Ellipsoidal height in meters, or -20000000000 if not available
Col6:  Geodetic Ondulation, or -20000000000 if not available
Col7:  Vn in m/s, or -20000000000 if not available
Col8:  Ve in m/s, or -20000000000 if not available
Col9:  Vu in m/s, or -20000000000 if not available
Col10:  Clock bias in seconds, or -20000000000 if not available
Col11: Clock drift in seconds/seconds, or -20000000000 if not available
Col12: NbrSV
Col13: PVT Mode field
Col14: MeanCorrAge in 1/100 seconds, or 65535 if not available
Col15: PVT Error
Col16: COG

-2: PVTCov block
Col1:  -2
Col2:  time (GPS second since Jan 06, 1980)
Col3:  Covariance xx
Col4:  Covariance yy
Col5:  Covariance zz
Col6:  Covariance tt

-3: PVTDOP block
Col1:  -3
Col2:  time (GPS second since Jan 06, 1980)
Col3:  PDOP value, or NA if PDOP not available
Col4:  TDOP value, or NA if TDOP not available
Col5:  HDOP value, or NA if HDOP not available
Col6:  VDOP value, or NA if VDOP not available
Col7:  HPL value in meters, or NA if not available
Col8:  VPL value in meters, or NA if not available
Col9:  NbrSV
*/
typedef struct
{
	double time;
	double lat;
	double lon;
	double ht;
	double geod;
	double vn;
	double ve;
	double vu;
	double cdt;
	double cdt_drift;
	int nsat;
	int type;
	int age;
	int err;
	double cog;
	double pdop;
	double tdop;
	double hdop;
	double vdop;
	double hpl;
	double vpl;
	double speed;
	double heading;
	double acc_h;
	double acc_v;
}pvt_t;

int read_sept_pvt(const char* pvtfname, std::vector< pvt_t> &pvts)
{
	FILE* fGPS = fopen(pvtfname, "r"); if (!fGPS) return 0;
	char buffer[512] = { 0 };
	pvt_t pvt = { 0 };
	FILE* fOUT = set_output_file(pvtfname, "-pvt.csv");
	FILE* fGGA = NULL;
	int index = 0;
	double pre_time = 0;
	while (fGPS != NULL && !feof(fGPS))
	{
		if (fgets(buffer, sizeof(buffer), fGPS) == NULL) break;
		/*
-1 1330754218.90  0.59020675017  -1.86143267962     1484.99971      -24.38316   -0.00214   -0.00280   -0.00028  8.31954443e-05 -4.815372e-09  32   6 440   0 -20000000000.000
-3  1330754218.90          0.790          0.400          0.480          0.630          6.059          7.668  32
-1 1330754219.00  0.59020675022  -1.86143267961     1484.99962      -24.38316   -0.00011   -0.00136    0.00023  8.31949744e-05 -4.838394e-09  32   6 450   0 -20000000000.000
-3  1330754219.00          0.790          0.400          0.480          0.630          6.004          7.671  32
		*/
		int id = 0;
		int num = sscanf(buffer, "%i", &id);
		if (num < 1) continue;
		if (id == -1)
		{
			num = sscanf(buffer, "%i %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %i %i %i %i %lf", &id, &pvt.time, &pvt.lat, &pvt.lon, &pvt.ht, &pvt.geod, &pvt.vn, &pvt.ve, &pvt.vu, &pvt.cdt, &pvt.cdt_drift, &pvt.nsat, &pvt.type, &pvt.age, &pvt.err, &pvt.cog);
			if (id == -1 && num == 17)
			{
			}
		}
		else if (id == -3)
		{
			double time = 0;
			int nsat1 = 0;
			num = sscanf(buffer, "%i %lf %lf %lf %lf %lf %lf %lf %i", &id, &time, &pvt.pdop, &pvt.tdop, &pvt.hdop, &pvt.vdop, &pvt.hpl, &pvt.vpl, &nsat1);
			if (id == -3 && num == 9)
			{
				if (fabs(time - pvt.time) < 0.01)
				{
					pvts.push_back(pvt);
					int wk = floor(pvt.time / (7 * 24 * 3600.0));
					double ws = pvt.time - wk * (7 * 24 * 3600.0);
					if (fOUT)
					{
						fprintf(fOUT,"%4i,%10.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%i,%i,%i\n", wk, ws, pvt.lat, pvt.lon, pvt.ht, pvt.geod, pvt.vn, pvt.ve, pvt.vu, pvt.pdop, pvt.hdop, pvt.vdop, pvt.hpl, pvt.vpl, pvt.cog, pvt.err, pvt.nsat, pvt.type);
					}
					if ((pvt.time - pre_time) > 30.0)
					{
						if (fGGA) fclose(fGGA);
						char temp[255] = { 0 };
						sprintf(temp, "-pvt%03i.nmea", index);
						fGGA = set_output_file(pvtfname, temp);
						++index;
					}
					pre_time = pvt.time;
					if (fGGA)
					{
						char gga_buffer[255] = { 0 };
						double blh[3] = { pvt.lat, pvt.lon, pvt.ht };
						outnmea_gga((unsigned char*)gga_buffer, ws, 1, blh, pvt.nsat, pvt.pdop, 0);
						fprintf(fGGA, "%s", gga_buffer);
					}
				}
			}
		}
	}
	if (fGPS) fclose(fGPS);
	if (fOUT) fclose(fOUT);
	if (fGGA) fclose(fGGA);
	return pvts.size() > 0;
}
#if 0
int read_ubx_pvt(const char* pvtfname, std::vector< pvt_t>& pvts)
{
	FILE* fGPS = fopen(pvtfname, "r"); if (!fGPS) return 0;
	char buffer[512] = { 0 };
	pvt_t pvt = { 0 };
	FILE* fGGA = NULL;
	int index = 0;
	double pre_time = 0;
	while (fGPS != NULL && !feof(fGPS))
	{
		if (fgets(buffer, sizeof(buffer), fGPS) == NULL) break;
		/*
 4474.8644,  33.816155100,-106.652717000, 1483.5121,    0.0010,    0.0000,  234.8708,    0.2630,    0.3630,    0.9500,  3, 26,1330757017499660032.0000
 4475.1055,  33.816155100,-106.652716900, 1483.5131,    0.0050,    0.0000,  234.8708,    0.2630,    0.3630,    0.9500,  3, 26,1330757017749660160.0000
 4475.3473,  33.816155100,-106.652716900, 1483.5121,    0.0030,    0.0000,  234.8708,    0.2630,    0.3630,    0.9500,  3, 26,1330757017999660032.0000
 4475.5885,  33.816155100,-106.652716900, 1483.5100,    0.0010,    0.0000,  234.8708,    0.2630,    0.3640,    0.9500,  3, 26,1330757018249659904.0000
		*/
		int id = 0;
		int num = sscanf(buffer, "%i", &id);
		if (num < 1) continue;
		if (id == -1)
		{
			double timeIMU = 0;
			double rate = 0;
			num = sscanf(buffer, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%i,%i,%lf", &timeIMU, &pvt.lat, &pvt.lon, &pvt.ht, &pvt.speed, &rate, &pvt.heading, &pvt.acc_h, &pvt.acc_v, &pvt.pdop, &pvt.type, &pvt.nsat, &pvt.time);
			if (id == -1 && num == 17)
			{
			}
		}
		else if (id == -3)
		{
			double time = 0;
			int nsat1 = 0;
			num = sscanf(buffer, "%i %lf %lf %lf %lf %lf %lf %lf %i", &id, &time, &pvt.pdop, &pvt.tdop, &pvt.hdop, &pvt.vdop, &pvt.hpl, &pvt.vpl, &nsat1);
			if (id == -3 && num == 9)
			{
				if (fabs(time - pvt.time) < 0.01)
				{
					pvts.push_back(pvt);
					int wk = floor(pvt.time / (7 * 24 * 3600.0));
					double ws = pvt.time - wk * (7 * 24 * 3600.0);
					if (fOUT)
					{
						fprintf(fOUT, "%4i,%10.4f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%i,%i,%i\n", wk, ws, pvt.lat, pvt.lon, pvt.ht, pvt.geod, pvt.vn, pvt.ve, pvt.vu, pvt.pdop, pvt.hdop, pvt.vdop, pvt.hpl, pvt.vpl, pvt.cog, pvt.err, pvt.nsat, pvt.type);
					}
					if ((pvt.time - pre_time) > 30.0)
					{
						if (fGGA) fclose(fGGA);
						char temp[255] = { 0 };
						sprintf(temp, "-pvt%03i.nmea", index);
						fGGA = set_output_file(pvtfname, temp);
						++index;
					}
					pre_time = pvt.time;
					if (fGGA)
					{
						char gga_buffer[255] = { 0 };
						double blh[3] = { pvt.lat, pvt.lon, pvt.ht };
						outnmea_gga((unsigned char*)gga_buffer, ws, 1, blh, pvt.nsat, pvt.pdop, 0);
						fprintf(fGGA, "%s", gga_buffer);
					}
				}
			}
		}
	}
	if (fGPS) fclose(fGPS);
	if (fOUT) fclose(fOUT);
	if (fGGA) fclose(fGGA);
	return pvts.size() > 0;
}
#endif


int Output_GenerateMessage_GPS(double timeIMU, double timeGPS, double lat, double lon, double ht, double geod, double speed, double heading, double pdop, int nsat, unsigned char* buf) {
	// Construct the output message, APGPS
	char* p = (char*)buf;
	p += sprintf(p, "#APGPS,");

	// IMU time
	p += sprintf(p, "%.3f,", timeIMU*1000.0);

	// GPS Time (ITOW)

	p += sprintf(p, "%.0lf,", timeGPS*1.0e9);

	double lat_deg = fabs(lat * 180.0 / PI);
	double lon_deg = fabs(lon * 180.0 / PI);

	int lat_deg_int = (int)floor(lat_deg);
	lat_deg -= lat_deg_int;

	int lon_deg_int = (int)floor(lon_deg);
	lon_deg -= lon_deg_int;

	// Lat
	if (lat < 0) {
		p += sprintf(p, "-");
	}
	p += sprintf(p, "%02i.%07d,", lat_deg_int, (int)floor(lat_deg*1.0e7));

	// Lon
	if (lon < 0) {
		p += sprintf(p, "-");
	}
	p += sprintf(p, "%03i.%07d,", lon_deg_int, (int)floor(lon_deg*1.0e7));

	// Altitude
	p += sprintf(p, "%.4f,", ht);
	p += sprintf(p, "%.4f,", geod);

	// Speed and heading
	p += sprintf(p, "%.4f,", speed);
	p += sprintf(p, "%.4f,", heading);

	// Accuracy, HDOP, and PDOP
	p += sprintf(p, "%.4f,", 0.2);
	p += sprintf(p, "%.4f,", 0.3);
	p += sprintf(p, "%.4f,", pdop);

	// Fix-Type and number of satellites
	p += sprintf(p, "%d,", 3);
	p += sprintf(p, "%d,", nsat);

	// Extra bits for my sim
	p += sprintf(p, "%.4f,", 0.05);
	double heading_acc = 100.0;
	if (speed > 20.0)
		heading_acc = 0.15;
	else if (speed > 10.0)
		heading_acc = 0.5;
	else if (speed > 5.0)
		heading_acc = 5.0;
	p += sprintf(p, "%.4f,", heading_acc);

	// fix-type (RTK)
	p += sprintf(p, "%d", 0);

	// Generate the checksum (an XOR of all the bytes between the $ and the *)
	// 1) Start at the first element of the buffer after $ ((char*)buf + 1)
	// 2) Init sum to zero
	// 3) Continue the loop until null is reached
	// 4) Increment the pointer location and repeat
	// Note: According to the website, deliminators are not included.  Not so in what follows!
	char* q, sum;
	for (q = (char*)buf + 1, sum = 0; *q; q++) {
		sum ^= *q;	// check-sum (xor)
	}

	// Append a separator, checksum, and EOL characters (CR + NL)
	p += sprintf(p, "*%02X%c", sum, 0x0D);

	// Find the terminating character and determine the number of characters in the message;
	// return the value.
	int countOut = 0;
	while (*(buf + countOut) != '\0') {
		countOut++;
	}

	//
	return(countOut);
}

static int decode_a1_asc_file_ins(const char* fname)
{
	FILE* fLOG = fopen(fname, "r"); if (!fLOG) return 0;
	char buffer[512] = { 0 };
	FILE* fGGA = NULL;
	FILE* fCSV = NULL;
	unsigned long index = 0;
	char* val[MAXFIELD];
	while (fLOG != NULL && !feof(fLOG))
	{
		if (fgets(buffer, sizeof(buffer), fLOG) == NULL) break;
		if (strlen(buffer) < 1) continue;
		++index;
		if (index < 2) continue;
		int num = parse_fields(buffer, val);
		if (num < 13) continue;
		double ws = atof(val[1])*1.0e-9;
		int wk = floor(ws) / (7 * 24 * 3600);
		ws -= wk * 7 * 24 * 3600;
		int status = atoi(val[2]);
		double blh[3] = { 0 };
		blh[0] = atof(val[3]) * D2R; /* lat */
		blh[1] = atof(val[4]) * D2R; /* lon */
		blh[2] = atof(val[5]); /* ht */
		double vel[3] = { 0 };
		vel[0] = atof(val[6]);
		vel[1] = atof(val[7]);
		vel[2] = atof(val[8]);
		double att[3] = { 0 };
		att[0] = atof(val[9]);
		att[1] = atof(val[10]);
		att[2] = atof(val[11]);
		int flag = atoi(val[12]);
		/*
imu_time_ms,gps_time_ns,ins_solution_status,lat_deg,lon_deg,alt_m,velocity_0_mps,velocity_1_mps,velocity_2_mps,attitude_0_deg,attitude_1_deg,attitude_2_deg,zupt_flag,position_geojson
428440,1335259598642443520,1,56.6810955,-5.1093139,61.4550018311,,,,-0.049008,1.032018,0.832515,1,"{""type"": ""Feature"", ""geometry"": {""type"": ""Point"", ""coordinates"": [-5.1093139, 56.6810955]}, ""properties"": {""radius"": 1, ""fillColor"": [255, 0, 0]}}"
428450,1335259598652437504,1,56.6810955,-5.1093139,61.4550018311,,,,-0.04765,1.035131,0.832629,1,"{""type"": ""Feature"", ""geometry"": {""type"": ""Point"", ""coordinates"": [-5.1093139, 56.6810955]}, ""properties"": {""radius"": 1, ""fillColor"": [255, 0, 0]}}"
		*/
		if (!fGGA) fGGA = set_output_file(fname, "-ins.nmea");
		if (!fCSV) fCSV = set_output_file(fname, "-ins.csv");
		if (fGGA)
		{
			char gga_buffer[255] = { 0 };
			outnmea_gga((unsigned char*)gga_buffer, ws, 1, blh, 10, 1.0, 0);
			fprintf(fGGA, "%s", gga_buffer);
		}
		if (fCSV)
		{
			fprintf(fCSV, "%10.3f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%i,%i\n", ws, blh[0] * R2D, blh[1] * R2D, blh[2], vel[0], vel[1], vel[2], att[0], att[1], att[2], status, flag);
		}
	}
	if (fLOG) fclose(fLOG);
	if (fCSV) fclose(fCSV);
	if (fGGA) fclose(   fGGA);

    // dwg - non-void function requires return value
    return EXIT_SUCCESS;
}
static int decode_a1_asc_file_gps(const char* fname)
{
	FILE* fLOG = fopen(fname, "r"); if (!fLOG) return 0;
	char buffer[512] = { 0 };
	FILE* fGGA = NULL;
	FILE* fCSV = NULL;
	unsigned long index = 0;
	char* val[MAXFIELD];
	while (fLOG != NULL && !feof(fLOG))
	{
		if (fgets(buffer, sizeof(buffer), fLOG) == NULL) break;
		if (strlen(buffer) < 1) continue;
		++index;
		if (index < 2) continue;
		int num = parse_fields(buffer, val);
		if (num < 14) continue;
		double ws = atof(val[1]) * 1.0e-9;
		int wk = floor(ws) / (7 * 24 * 3600);
		ws -= wk * 7 * 24 * 3600;
		double pvt[20] = { 0 };
		pvt[0] = atof(val[2]) * D2R; /* lat */
		pvt[1] = atof(val[3]) * D2R; /* lon */
		pvt[2] = atof(val[4]); /* ht */
		pvt[3] = atof(val[5]);
		pvt[4] = atof(val[6]);
		pvt[5] = atof(val[7]);
		pvt[6] = atof(val[8]);
		pvt[7] = atof(val[9]);
		pvt[8] = atof(val[10]);
		pvt[9] = atof(val[11]);
		pvt[10] = atof(val[12]);
		pvt[11] = atof(val[13]);
		pvt[12] = atof(val[14]);
		/*
imu_time_ms,gps_time_ns,lat_deg,lon_deg,alt_ellipsoid_m,alt_msl_m,speed_mps,heading_deg,accuracy_horizontal_m,accuracy_vertical_m,PDOP,gnss_fix_type,num_sats,speed_accuracy_mps,heading_accuracy_deg,carrier_solution_status,position_geojson
428557.383,1335259598749614848,56.6810955,-5.1093139,61.453,9.855,0.002,0,0.23,0.352,1.05,3,30,0.086,180,0,"{""type"": ""Feature"", ""geometry"": {""type"": ""Point"", ""coordinates"": [-5.1093139, 56.6810955]}, ""properties"": {""radius"": 3, ""fillColor"": [255, 0, 0]}}"
428797.368,1335259598999614720,56.6810955,-5.109314,61.452,9.853,0.001,0,0.23,0.351,1.05,3,30,0.083,180,0,"{""type"": ""Feature"", ""geometry"": {""type"": ""Point"", ""coordinates"": [-5.109314, 56.6810955]}, ""properties"": {""radius"": 3, ""fillColor"": [255, 0, 0]}}"
429297.369,1335259599499613952,56.6810955,-5.109314,61.453,9.855,0.008,0,0.23,0.351,1.05,3,30,0.088,180,0,"{""type"": ""Feature"", ""geometry"": {""type"": ""Point"", ""coordinates"": [-5.109314, 56.6810955]}, ""properties"": {""radius"": 3, ""fillColor"": [255, 0, 0]}}"
		*/
		if (!fGGA) fGGA = set_output_file(fname, "-gps.nmea");
		if (!fCSV) fCSV = set_output_file(fname, "-gps.csv");
		if (fGGA)
		{
			char gga_buffer[255] = { 0 };
			outnmea_gga((unsigned char*)gga_buffer, ws, 1, pvt, 10, 1.0, 0);
			fprintf(fGGA, "%s", gga_buffer);
		}
		if (fCSV)
		{
			fprintf(fCSV, "%10.3f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", ws, pvt[0] * R2D, pvt[1] * R2D, pvt[2], pvt[3], pvt[4], pvt[5], pvt[6], pvt[7], pvt[8], pvt[9], pvt[10], pvt[11], pvt[12]);
		}
	}
	if (fLOG) fclose(fLOG);
	if (fCSV) fclose(fCSV);
	if (fGGA) fclose(fGGA);
    return EXIT_SUCCESS;
}

static int decode_a1_asc_file_imu(const char* fname)
{
	FILE* fLOG = fopen(fname, "r"); if (!fLOG) return 0;
	char buffer[512] = { 0 };
	FILE* fCSV = NULL;
	unsigned long index = 0;
	char* val[MAXFIELD];
	while (fLOG != NULL && !feof(fLOG))
	{
		if (fgets(buffer, sizeof(buffer), fLOG) == NULL) break;
		if (strlen(buffer) < 1) continue;
		++index;
		if (index < 2) continue;
		int num = parse_fields(buffer, val);
		if (num < 11) continue;
		double imu[20] = { 0 };
		imu[0] = atof(val[0]) * 1.0e-3;
		imu[1] = atof(val[1]); /* fx */
		imu[2] = atof(val[2]); /* fy */
		imu[3] = atof(val[3]); /* fz */
		imu[4] = atof(val[4]); /* wx */
		imu[5] = atof(val[5]); /* wy */
		imu[6] = atof(val[6]); /* wz */
		imu[7] = atof(val[7]); /* wz_fog */
		imu[8] = atof(val[8]); /* odr */
		imu[9] = atof(val[9]); /* odr time */
		imu[10] = atof(val[10]); /* temp */
		/*
imu_time_ms,accel_x_g,accel_y_g,accel_z_g,angrate_x_dps,angrate_y_dps,angrate_z_dps,fog_angrate_dps,odometer_speed_mps,odometer_time_ms,temperature_c
428439.173,0.0154,-0.0013,-1.0078,-0.0753,-0.0451,0.0415,-0.0031,0,0,22.5898
428449.208,0.0189,0.0001,-1.0089,0.1864,0.1983,-0.031,0.00279,0,0,22.5898
428459.202,0.0179,-0.001,-1.0001,0.1433,-0.0165,-0.0129,0.00476,0,0,22.625
428469.196,0.0194,-0.0012,-0.9993,0.2482,-0.0333,-0.005,-0.00505,0,0,22.625
428479.191,0.0205,-0.0031,-0.9954,0.0304,-0.0353,-0.091,0.02633,0,0,22.625
428489.185,0.0185,-0.0026,-1.0076,-0.0917,-0.0185,-0.1251,-0.01484,0,0,22.625
		*/
		if (!fCSV) fCSV = set_output_file(fname, "-imu.csv");
		if (fCSV)
		{
			fprintf(fCSV, "%10.3f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", imu[0], imu[1], imu[2], imu[3], imu[4], imu[5], imu[6], imu[7], imu[8], imu[9], imu[10]);
		}
	}
	if (fLOG) fclose(fLOG);
	if (fCSV) fclose(fCSV);

    // dwg - non-void function requires return value
    return EXIT_SUCCESS;

}

int merge_data_file(const char* imufname, const char *gpsfname)
{
	FILE* fIMU = NULL;
	FILE* fOUT = NULL;

	printf("%s\n%s\n", imufname, gpsfname);

	double time_offset = 1330752542.6489844;
	std::vector<pvt_t> pvts;
	if (!found_time_offset(imufname, &time_offset)) return 0;
	if (!read_sept_pvt(gpsfname, pvts)) return 0;

	char buffer[1024] = { 0 };
	fIMU = fopen(imufname, "rb"); if (fIMU == NULL) return 0;
	fOUT = set_output_file(imufname, "-sept.log");

	char* val[MAXFIELD];

	double rate = 0.0;

	double heading = 0.0, speed = 0.0, lat = 0.0, lon = 0.0, ht = 0.0, pdop = 0.0, time_GPS = 0.0, satnum = 0.0;

	double StartTime = 0;
	uint8_t data = 0;
	nmea_buff_t buff = { 0 };
	std::vector<pvt_t>::iterator pvt = pvts.begin();
	while (fIMU != NULL && !feof(fIMU))
	{
		if ((data = fgetc(fIMU)) == EOF) break;
		if (!add_buff(&buff, data)) continue;

		if (strstr((char*)buff.dat, "#APIMU") != NULL)
		{
			memset(buffer, 0, sizeof(buffer));
			memcpy(buffer, buff.dat, sizeof(char) * buff.nbyte);
			int num = parse_fields(buffer, val);
			double timeIMU = atof(val[1]) * 1.0e-3;

			while (pvt != pvts.end())
			{
				double timeIMU_GPS = pvt->time - time_offset;
				if (timeIMU_GPS <= timeIMU)
				{
					double heading = atan2(pvt->ve, pvt->vn) * 180 / PI;
					if (heading < 0)
						heading += 360.0;
					double speed = sqrt(pvt->vn * pvt->vn + pvt->ve * pvt->ve);
					char gga_buff[255] = { 0 };
					Output_GenerateMessage_GPS(timeIMU_GPS, pvt->time, pvt->lat, pvt->lon, pvt->ht, pvt->geod, speed, heading, pvt->pdop, pvt->nsat, (unsigned char*)gga_buff);
					if (fOUT) fprintf(fOUT, "%s", gga_buff);
					++pvt;
				}
				else
				{
					break;
				}
			}
			if (fOUT) fprintf(fOUT, "%s", buff.dat);
		}

		buff.nbyte = 0;
	}

	if (fIMU) fclose(fIMU);
	if (fOUT) fclose(fOUT);

	return 1;
}

//============================================================================================================
//! \brief Simple decoding progress report.

static void report(const NComRxC* nrx)
{
	printf("\rChars Read %" PRIu64 ", Packets Read %" PRIu64 ", Chars Skipped %" PRIu64,
		NComNumChars(nrx), NComNumPackets(nrx), NComSkippedChars(nrx));

	fflush(stdout);
}


//============================================================================================================
//! \brief Used to write some of the NCom data to a file pointer.
//!
//! There are only a few examples here of how to use the data values.

static void print(FILE* fp, FILE *fsol, const NComRxC* nrx)
{
	static int HeaderWritten = 0;

	// Add in the headers to the file
	if (HeaderWritten == 0)
	{
#if __linux__
		fprintf(fp, " WN,WS,"
#else
		fwprintf(fp, L" WN,WS,"
#endif
			" Latitude(deg),"
			" Longitude(deg),"
			" Altitude(m),"
			" Velocity north(m/s),"
			" Velocity east(m/s),"
			" Velocity down(m/s),"
			" Roll(deg),"
			" Pitch(deg),"
			" Heading(deg),"
			" Acceleration Xv(m/s�),"
			" Acceleration Yv(m/s�),"
			" Acceleration Zv(m/s�),"
			//" Acceleration forward(m/s�),"
			//" Acceleration lateral(m/s�),"
			//" Acceleration down(m/s�),"
			" Angular rate Xv(deg/s),"
			" Angular rate Yv(deg/s),"
			" Angular rate Zv(deg/s)"
			//" Angular rate forward(deg/s),"
			//" Angular rate lateral(deg/s),"
			//" Angular rate down(deg/s)"
		);
		fprintf(fp, "\n");
		HeaderWritten = 1;
	}

	// Print the time - GPS time, local date and time zone.
	if (nrx->mIsTimeValid)
	{
		//double     gps2machine, mMachineTime;
		//time_t     t1;
		//struct tm* td;
		//int        ms;

		// Convert GPS seconds (from 1980-01-06 00:00:00) to machine seconds (from 1970-01-01 00:00:00). It is
		// very likely the machine will adjust for leap seconds, hence the correct GPS UTC difference is
		// applied. If the local machine time does not start from 1970-01-01 00:00:00 then the value of
		// gps2machine below needs to change.
		//gps2machine = 315964800.0;

		//if (nrx->mIsTimeUtcOffsetValid)
		//{
		//	mMachineTime = nrx->mTime + gps2machine + nrx->mTimeUtcOffset;
		//}
		//else { mMachineTime = nrx->mTime + gps2machine - 18; }

		// Compute local time
		//t1 = (time_t)floor(mMachineTime);
		//td = localtime(&t1);
		//ms = floor(0.5 + (mMachineTime - t1) * 1000.0);
		//if (ms < 0) ms = 0; else if (ms > 999) ms = 999;

		// Print: GPS time, local date, time zone.
		//fprintf(fp, "%10.3f,%04d-%02d-%02d,%02d:%02d:%02d.%03d,",
		//	nrx->mTime,
		//	1900 + td->tm_year, 1 + td->tm_mon, td->tm_mday, td->tm_hour, td->tm_min, td->tm_sec, ms);
		int wk = floor(nrx->mTime / (7 * 24 * 3600.0));
		double ws = nrx->mTime - wk * 7 * 24 * 3600;
		fprintf(fp, "%4i,%10.3f,", wk, ws);
	}

	// Print the measurments listed in the header
	if (nrx->mIsTimeValid)
	{
		int wk = floor(nrx->mTime / (7 * 24 * 3600.0));
		double ws = nrx->mTime - wk * 7 * 24 * 3600;

		char gga_buffer[255] = { 0 };
		double blh[3] = { nrx->mLat * D2R, nrx->mLon * D2R, nrx->mAlt };
		outnmea_gga((unsigned char*)gga_buffer, ws, 1, blh, nrx->mGpsNumObs, nrx->mHDOP, 0);
		fprintf(fsol, "%s", gga_buffer);

		// Print the 	PosLat (deg)
		if (nrx->mIsLatValid) fprintf(fp, "%14.9f", nrx->mLat);
		fprintf(fp, ",");

		// Print the 	PosLon (deg)
		if (nrx->mIsLonValid) fprintf(fp, "%14.9f", nrx->mLon);
		fprintf(fp, ",");

		// Print the 	PosAlt (m)
		if (nrx->mIsAltValid) fprintf(fp, "%10.3f", nrx->mAlt);
		fprintf(fp, ",");

		// Print the 	VelNorth (km/h)
		if (nrx->mIsVnValid) fprintf(fp, "%10.3f", nrx->mVn);
		fprintf(fp, ",");

		// Print the 	VelEast (km/h)
		if (nrx->mIsVeValid) fprintf(fp, "%10.3f", nrx->mVe);
		fprintf(fp, ",");

		// Print the 	VelDown (km/h)
		if (nrx->mIsVdValid) fprintf(fp, "%10.3f", nrx->mVd);
		fprintf(fp, ",");

		// Print the 	AngleRoll (deg)
		if (nrx->mIsRollValid) fprintf(fp, "%8.3f", nrx->mRoll);
		fprintf(fp, ",");

		// Print the 	AnglePitch (deg)
		if (nrx->mIsPitchValid) fprintf(fp, "%8.3f", nrx->mPitch);
		fprintf(fp, ",");

		// Print the 	AngleHeading (deg)
		if (nrx->mIsHeadingValid) fprintf(fp, "%8.3f", nrx->mHeading);
		fprintf(fp, ",");

		// Print the 	AccelX (m/s�)
		if (nrx->mIsAxValid) fprintf(fp, "%10.3f", nrx->mAx);
		fprintf(fp, ",");

		// Print the 	AccelY (m/s�)
		if (nrx->mIsAyValid) fprintf(fp, "%10.3f", nrx->mAy);
		fprintf(fp, ",");

		// Print the 	AccelZ (m/s�)
		if (nrx->mIsAzValid) fprintf(fp, "%10.3f", nrx->mAz);
		fprintf(fp, ",");

		// Print the 	AngleRateX (deg/s)
		if (nrx->mIsWxValid) fprintf(fp, "%10.3f", nrx->mWx);
		fprintf(fp, ",");

		// Print the 	AngleRateY (deg/s)
		if (nrx->mIsWyValid) fprintf(fp, "%10.3f", nrx->mWy);
		fprintf(fp, ",");

		// Print the 	AngleRateZ (deg/s)
		if (nrx->mIsWzValid) fprintf(fp, "%10.3f", nrx->mWz);
		//fprintf(fp, ",");

		// Print the 	AccelForward (m/s�)
		//if (nrx->mIsAfValid) fprintf(fp, "%10.3f", nrx->mAf);
		//fprintf(fp, ",");

		// Print the 	AccelLateral (m/s�)
		//if (nrx->mIsAlValid) fprintf(fp, "%10.3f", nrx->mAl);
		//fprintf(fp, ",");

		// Print the 	AccelDown (m/s�)
		//if (nrx->mIsAdValid) fprintf(fp, "%10.3f", nrx->mAd);
		//fprintf(fp, ",");

		// Print the 	AngleRateForward (deg/s)
		//if (nrx->mIsWfValid) fprintf(fp, "%10.3f", nrx->mWf);
		//fprintf(fp, ",");

		// Print the 	AngleRateLateral (deg/s)
		//if (nrx->mIsWlValid) fprintf(fp, "%10.3f", nrx->mWl);
		//fprintf(fp, ",");

		// Print the 	AngleRateDown (deg/s)
		//if (nrx->mIsWdValid) fprintf(fp, "%10.3f", nrx->mWd);

		fprintf(fp, "\n");
	}
}

/* read oxts ncom file, and generate csv file and nmea gga file */
static int read_oxts_data(const char* fname)
{
	FILE* fLOG = fopen(fname, "rb"); if (!fLOG) return 0;
	FILE* fCSV = NULL;
	FILE* fGGA = NULL;

	int c = 0;                // char from input file
	NComRxC* nrx = NComCreateNComRxC();

	while (nrx != NULL && fLOG != NULL && !feof(fLOG) && (c = fgetc(fLOG)) != EOF)
	{
		// Decode the data
		if (NComNewChar(nrx, (unsigned char)c) == COM_NEW_UPDATE)
		{
			// For regular updates then output to main output file, otherwise,
			// for falling edge input triggers then output to trigger file.
			switch (nrx->mOutputPacketType)
			{
			case OUTPUT_PACKET_REGULAR:
			{

				if (!fCSV) fCSV = set_output_file(fname, "-rts.csv");
				if (!fGGA) fGGA = set_output_file(fname, "-rts.nmea");

				print(fCSV, fGGA, nrx);
				break;
			}
			case OUTPUT_PACKET_IN1DOWN:
			{
				//print(fptrig, nrx); 
				break;
			}
			default: break;
			}
		}
	}

	// Report final statistics
	report(nrx);
	printf("\n");

	if (fLOG) fclose(fLOG);
	if (fCSV) fclose(fCSV);
	if (fGGA) fclose(fGGA);

	NComDestroyNComRxC(nrx);

	return 0;
}

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN (1200)
#endif

typedef struct
{
	uint8_t buf[MAX_BUF_LEN];
	int nseg;
	int nbyte;
	int nlen; /* length of binary message */
	int loc[MAXFIELD];
}a1buff_t;

static int input_a1_data(a1buff_t* a1, uint8_t data)
{
	int ret = 0;
	if (a1->nbyte >= MAX_BUF_LEN) a1->nbyte = 0;
	/* #AP */
	if (a1->nbyte == 1 && !((data == 'A'&& a1->buf[0] == '#') || (data == 'G' && a1->buf[0] == '$'))) a1->nbyte = 0;
	//if (a1->nbyte == 2 && data != 'P') a1->nbyte = 0;
	if ( a1->nbyte == 0 )
	{
		if (data == '#'||data=='$')
		{
			memset(a1, 0, sizeof(a1buff_t));
			//a1->nseg = a1->nlen = 0;
			a1->buf[a1->nbyte++] = data;
		}
	}
	else
	{
		if (data == ',')
		{
			a1->loc[a1->nseg++] = a1->nbyte;
			if (a1->nseg == 2)
			{
				if (strstr((char*)a1->buf, "APANT") != NULL || strstr((char*)a1->buf, "APRTK") != NULL)
				{
					uint8_t* temp = a1->buf + (a1->loc[0]) + 1;
					a1->nlen = atof((char*)temp);
				}
				else
				{
					a1->nlen = 0;
				}
			}
		}
		a1->buf[a1->nbyte++] = data;
		if (a1->nlen == 0)
		{
			/* check message end for normal asc message */
			if (data == '\r' || data == '\n')
			{
				/* 1*74 */
				if (a1->nbyte > 3 && a1->buf[a1->nbyte - 4] == '*')
				{
					a1->loc[a1->nseg++] = a1->nbyte - 4;
					ret = 1;
				}
			}
		}
		else
		{
			/* check message end for binary message ,binary msg\r\n */
			if (a1->nbyte >= (a1->nlen + a1->loc[1] + 3))
			{
				if (a1->buf[6] == '1') /* APANT1 */
					ret = 2;
				else if (a1->buf[6] == '2') /* APANT2 */
					ret = 3;
				else /* APRTK */
					ret = 4;
			}
		}
	}
	return ret;
}

/* read A1 file*/
static int read_a1_data(const char* fname)
{
	FILE* fLOG = fopen(fname, "rb"); if (!fLOG) return 0;
	int data = 0;
	FILE* fCSV = NULL;
	FILE* fGGA = NULL;
	FILE* fIMU = NULL;
	FILE* fLOG_GGA = NULL;
	FILE* fGPS_CSV = NULL;
	FILE* fGP2_CSV = NULL;
	FILE* fGPS_GGA = NULL;
	FILE* fGP2_GGA = NULL;
	FILE* fANT1 = NULL;
	FILE* fANT2 = NULL;
	FILE* fBASE = NULL;
	FILE* fASC = NULL;
	char* val[MAXFIELD];
	a1buff_t a1buff = { 0 };

	while (fLOG != NULL && !feof(fLOG) && (data = fgetc(fLOG)) != EOF)
	{
		int ret = input_a1_data(&a1buff, data);
		if (ret)
		{
			if (strstr((char*)a1buff.buf, "*") == NULL) {
				printf("no checksum: %s\n", a1buff.buf);
				a1buff.nbyte = 0;
				continue;
			}

			if (!verify_checksum((char*)a1buff.buf)) {
				printf("checksum fail: %s\n", a1buff.buf);
				a1buff.nbyte = 0;
				continue;
			}

			if (strstr((char*)a1buff.buf, "$G") != NULL)
			{
				if (!fLOG_GGA) fLOG_GGA = set_output_file(fname, "-src.nmea");
				if (fLOG_GGA)
				{
					fprintf(fLOG_GGA, "%s", (char*)a1buff.buf);
				}
				a1buff.nbyte = 0;
				continue;
			}

			int isOK = 0;
			int num = 0;
			if (ret == 1)
			{
				num = parse_fields((char*)a1buff.buf, val);
			}
			else if (ret == 2) /* APANT1 */
			{
				if (!fANT1) fANT1 = set_output_file(fname, "-ant1.log");
				if (fANT1)
				{
					fwrite(a1buff.buf + a1buff.loc[1] + 1, sizeof(char), a1buff.nlen, fANT1);
				}
				isOK = 1;
			}
			else if (ret == 3) /* APANT2 */
			{
				if (!fANT2) fANT2 = set_output_file(fname, "-ant2.log");
				if (fANT2)
				{
					fwrite(a1buff.buf + a1buff.loc[1] + 1, sizeof(char), a1buff.nlen, fANT2);
				}
				isOK = 1;
			}
			else if (ret == 4) /* APRTK */
			{
				if (!fBASE) fBASE = set_output_file(fname, "-base.log");
				if (fBASE)
				{
					fwrite(a1buff.buf + a1buff.loc[1] + 1, sizeof(char), a1buff.nlen, fBASE);
				}
				isOK = 1;
			}

			if (!isOK && num == 18 && strstr(val[0], "APGPS") != NULL)
			{
				/* time [s], lat [deg], lon [deg], ht [m], speed [m/s], heading [deg], hor. accuracy [m], ver. accuracy [m], PDOP, fixType, sat num, gps second [s], pps [s] */
				/*
#APGPS,318213.135,1343773580500184320,37.3988755,-121.9791327,-27.9650,1.9240,0.0110,0.0000,0.2380,0.3820,0.9700,3,29,0.0820,180.0000,0*65
				*/
				double gps[20] = { 0 };
				gps[0] = atof(val[1]); /* time MCU */
				gps[1] = atof(val[2])*1.0e-6; /* GPS ms */

				gps[2] = atof(val[3]); /* lat */
				gps[3] = atof(val[4]); /* lon */
				gps[4] = atof(val[5]); /* ht */
				gps[5] = atof(val[6]); /* msl */

				gps[6] = atof(val[7]); /* speed */
				gps[7] = atof(val[8]); /* heading */
				gps[8] = atof(val[9]); /* acc_h */
				gps[9] = atof(val[10]); /* acc_v */
				gps[10] = atof(val[11]); /* pdop */
				gps[11] = atof(val[12]); /* fixtype */
				gps[12] = atof(val[13]); /* sat number */
				gps[13] = atof(val[14]); /* acc speed */
				gps[14] = atof(val[15]); /* acc heading */
				gps[15] = atof(val[16]); /* rtk fix status */
				if (!fGPS_CSV){
					fGPS_CSV = set_output_file(fname, "-gps.csv");
					if (fGPS_CSV) fprintf(fGPS_CSV, "Time_MCU_ms,GPS_time_ms,lat,lon,alt_ellipsoidal,speed,heading,acc_h,acc_v,pdop,fixtype,sat_num,acc_speed,acc_heading,rtk_fix\n");
				}
				if (fGPS_CSV)
				{
					fprintf(fGPS_CSV, "%10.3f,%14.9f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", gps[0],gps[1], gps[2], gps[3], gps[4], gps[6], gps[7], gps[8], gps[9], gps[10], gps[11], gps[12], gps[13], gps[14], gps[15]);
				}
				if (!fGPS_GGA) fGPS_GGA = set_output_file(fname, "-gps.nmea");
				if (fGPS_GGA)
				{
					char gga_buffer[255] = { 0 };
					double blh[3] = { gps[2] * D2R, gps[3] * D2R, gps[4] };
					outnmea_gga((unsigned char*)gga_buffer, gps[0], 1, blh, gps[12], gps[10], 0);
					fprintf(fGPS_GGA, "%s", gga_buffer);
				}
				isOK = 1;
			}
			if (!isOK && num >= 17 && strstr(val[0], "APGP2") != NULL) {
				/*
#APGP2,318213.258,1343773580499803648,37.3989018,-121.9791254,-27.2050,2.6840,0.0090,0.0000,0.2730,0.4510,1.1400,3,26,0.0600,180.0000,0*07
				*/

				double gps[20] = { 0 };
				gps[0] = atof(val[1]); /* time MCU */
				gps[1] = atof(val[2])*1.0e-6; /* GPS ms */

				gps[2] = atof(val[3]); /* lat */
				gps[3] = atof(val[4]); /* lon */
				gps[4] = atof(val[5]); /* ht */
				gps[5] = atof(val[6]); /* msl */
				gps[6] = atof(val[7]); /* speed */
				gps[7] = atof(val[8]); /* heading */
				gps[8] = atof(val[9]); /* acc_h */
				gps[9] = atof(val[10]); /* acc_v */
				gps[10] = atof(val[11]); /* pdop */
				gps[11] = atof(val[12]); /* fixtype */
				gps[12] = atof(val[13]); /* sat number */
				gps[13] = atof(val[14]); /* acc speed */
				gps[14] = atof(val[15]); /* acc heading */
				gps[15] = atof(val[16]); /* rtk fix status */
				if (!fGP2_CSV) {
					fGP2_CSV = set_output_file(fname, "-gp2.csv");
					if (fGP2_CSV) fprintf(fGP2_CSV, "Time_MCU_ns,GPS_time_ms,lat,lon,alt_ellipsoidal,speed,heading,acc_h,acc_v,pdop,fixtype,sat_num,acc_speed,acc_heading,rtk_fix\n");
				}
				if (fGP2_CSV)
				{
					fprintf(fGP2_CSV, "%10.3f,%14.9f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", gps[0], gps[1], gps[2], gps[3], gps[4], gps[6], gps[7], gps[8], gps[9], gps[10], gps[11], gps[12], gps[13], gps[14], gps[15]);
				}
				if (!fGP2_GGA) fGP2_GGA = set_output_file(fname, "-gp2.nmea");
				if (fGP2_GGA)
				{
					char gga_buffer[255] = { 0 };
					double blh[3] = { gps[2] * D2R, gps[3] * D2R, gps[4] };
					outnmea_gga((unsigned char*)gga_buffer, gps[0], 1, blh, gps[12], gps[10], 0);
					fprintf(fGP2_GGA, "%s", gga_buffer);
				}
				isOK = 1;
			}
			if (!isOK && num >= 12 && strstr(val[0], "APIMU") != NULL)
			{
				/*
				#APIMU,318214.937,0.0344,-0.0128,1.0077,-0.0817,0.0013,-0.0038,0.01051,0.0000,318214.548,47.0547*55
				*/
				double imu[20] = { 0 };
				imu[0] = atof(val[1]); /* imu time ms*/
				imu[1] = atof(val[2]); /* fx */
				imu[2] = atof(val[3]); /* fy */
				imu[3] = atof(val[4]); /* fz */
				imu[4] = atof(val[5]); /* wx */
				imu[5] = atof(val[6]); /* wy */
				imu[6] = atof(val[7]); /* wz */
				imu[7] = atof(val[8]); /* wz_fog */
				imu[8] = atof(val[9]); /* odr */
				imu[9] = atof(val[10]) * 1.0e-3; /* odr time */
				imu[10] = atof(val[11]); /* temp */
				if (!fIMU) {
					fIMU = set_output_file(fname, "-imu.csv");
					if (fIMU) fprintf(fIMU, "imu_time_ms,fx,fy,fz,wx,wy,wz,wz_fog,odr,odr_time,temp\n");
				}
				if (fIMU)
				{
					fprintf(fIMU, "%10.3f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", imu[0], imu[1], imu[2], imu[3], imu[4], imu[5], imu[6], imu[7], imu[8], imu[9], imu[10]);
				}
				isOK = 1;
			}
			if (!isOK && num >= 14 && strstr(val[0], "APINS") != NULL)
			{
				/* time[s], lat[radian], lon[radian], ht[m], vn[m / s], ve[m / s], vd[m / s], roll[deg], pitch[deg], yaw[deg] */
				/*
				#APINS,318215,1343773580502990592,1,37.398875500000,-121.979132700000,-27.965002059937,,,,-0.166232,1.773182,0.250746,1*74
				*/
				double ins[20] = { 0 };
				ins[0] = atof(val[1]);			//IMUtime (ms)
				ins[1] = atof(val[2]) * 1.0e-6;	//GPStime (ms)
				ins[2] = atof(val[3]);			//INS solution

				ins[3] = atof(val[4]);			//lat
				ins[4] = atof(val[5]);			//lon
				ins[5] = atof(val[6]);			//alt

				ins[6] = atof(val[7]);			//vn
				ins[7] = atof(val[8]);			//ve
				ins[8] = atof(val[9]);			//vd

				ins[9] = atof(val[10]);			//roll
				ins[10] = atof(val[11]);		//pitch
				ins[11] = atof(val[12]);		//heading

				ins[12] = atof(val[13]);		//ZUPT
				if (!fCSV) {
					fCSV = set_output_file(fname, "-rts.csv");
					if (fCSV) fprintf(fCSV, "imu_time_ms,GPS_time_ms,lat,lon,alt,vn,ve,vd,roll,pitch,heading,INS_solution\n");
				}
				if (fCSV)
				{
					fprintf(fCSV, "%10.3f,%14.9f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", ins[0],ins[1], ins[3], ins[4], ins[5], ins[6], ins[7], ins[8], ins[9], ins[10], ins[11], ins[2]);
				}
				if (!fGGA) fGGA = set_output_file(fname, "-rts.nmea");
				if (fGGA)
				{
					char gga_buffer[255] = { 0 };
					double blh[3] = { ins[3] * D2R, ins[4] * D2R, ins[5] };
					outnmea_gga((unsigned char*)gga_buffer, ins[0], 1, blh, 10, 1, 0);
					fprintf(fGGA, "%s", gga_buffer);
				}
				isOK = 1;
			}
			if (!isOK)
			{
				printf("No correct data %s\n", a1buff.buf);
			}
			a1buff.nbyte = 0;
		}
	}
	if (fLOG) fclose(fLOG);
	if (fCSV) fclose(fCSV);
	if (fGGA) fclose(fGGA);
	if (fIMU) fclose(fIMU);
	if (fGPS_CSV) fclose(fGPS_CSV);
	if (fGP2_CSV) fclose(fGP2_CSV);
	if (fGPS_GGA) fclose(fGPS_GGA);
	if (fGP2_GGA) fclose(fGP2_GGA);
	if (fANT1) fclose(fANT1);
	if (fANT2) fclose(fANT2);
	if (fBASE) fclose(fBASE);
	if (fLOG_GGA) fclose(fLOG_GGA);
	return 0;
}

double lat2local(double lat, double* lat2north)
{
	double f_WGS84 = (1.0 / finv_WGS84);
	double e2WGS84 = (2.0 * f_WGS84 - f_WGS84 * f_WGS84);
	double slat = sin(lat);
	double clat = cos(lat);
	double one_e2_slat2 = 1.0 - e2WGS84 * slat * slat;
	double Rn = ae_WGS84 / sqrt(one_e2_slat2);
	double Rm = Rn * (1.0 - e2WGS84) / (one_e2_slat2);
	*lat2north = Rm;
	return Rn * clat;
}
int main(int argc, char** argv)
{
	if (argc < 3)
	{
		/* */
		printf("%s format filename\n", argv[0]);
		printf("format => a1, oxts\n");
		//decode_a1_asc_file_ins("D:\\data\\GlencoeSkiCentre\\ins.csv");
		//decode_a1_asc_file_gps("D:\\data\\GlencoeSkiCentre\\gps.csv");
		//decode_a1_asc_file_imu("D:\\data\\GlencoeSkiCentre\\imu.csv");
		//read_oxts_data("D:\\austin\\UpsideDownWithLeverArmMeasurements\\2022_8_5_1525_drive2_Tag SN40434-022.ncom");
		//read_a1_data("D:\\austin\\UpsideDownWithLeverArmMeasurements\\2022_8_5_1525_24drive2");
		//read_a1_data("D:\\austin\\UpsideDownWithLeverArmMeasurements\\2022_8_5_1525_102drive2");
		//read_a1_data("D:\\austin\\X-Y-Z test\\2022_8_9_1120_24drive1.txt");
		//read_a1_data("D:\\austin\\X-Y-Z test\\2022_8_9_1120_102drive1.txt");
		//read_a1_data("D:\\John deere drive test\\output_date_2022_8_9_time_5_29_54_SN_202200000100.txt");
		//read_a1_data("D:\\John deere drive test\\output_date_2022_8_9_time_6_32_58_SN_202200000100.txt");
		//read_a1_data("D:\\John deere drive test\\output_date_2022_8_9_time_7_1_25_SN_202200000100.txt");
		//read_a1_data("D:\\John deere drive test\\output_date_2022_8_9_time_7_28_21_SN_202200000100.txt");
		//decode_a1_asc_file_gps("D:\\sgl\\Ottawa.NewFirmware.GroundRecording.2022.08.08\\gps.csv");
		//decode_a1_asc_file_imu("D:\\sgl\\Ottawa.NewFirmware.GroundRecording.2022.08.08\\imu.csv");
		//decode_a1_asc_file_ins("D:\\sgl\\Ottawa.NewFirmware.GroundRecording.2022.08.08\\ins.csv");
		//read_a1_data("D:\\sgl\\Ottawa.NewFirmware.GroundRecording.2022.08.08\\output_date_2022_8_8_time_14_40_7_SN_202200000104.txt");
		read_a1_data("C:\\projects\\driveData\\2022\\October_10\\28\\output_date_2022_10_28_time_17_19_7_SN_202100000024.txt");
		//read_a1_data("C:\\projects\\driveData\\Validation\\drivetests\\garage1\\output_date_2022_10_26_time_15_55_32_SN_202100000024.txt");
		//read_a1_data("C:\\projects\\driveData\\PNTAX2022\\DAY1\\DAY1\\ShortJam\\day1_shortjam_2.txt");
		//read_a1_data("D:\\anello\\output_date_2022_8_15_time_17_5_38_SN_202200000115--asc.txt");
	}
	else
	{
		if (strstr(argv[1], "a1") != NULL)
		{
			read_a1_data(argv[2]);
		}
		else if (strstr(argv[1], "oxts") != NULL)
		{
			read_oxts_data(argv[2]);
		}
		//merge_data_file(argv[1], argv[2]);
	}
	return 0;
}