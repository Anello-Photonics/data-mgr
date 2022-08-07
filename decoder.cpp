// test.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>
#include <string>
#include <algorithm>

#include "NComRxC.h"

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

static int parse_fields(char* const buffer, char** val)
{
	char* p, *q;
	int n = 0;

	/* parse fields */
	for (p = buffer; *p && n < MAXFIELD; p = q + 1) {
		if ((q = strchr(p, ',')) || (q = strchr(p, '*')) || (q = strchr(p, '\n'))) {
			val[n++] = p; *q = '\0';
		}
		else break;
	}
	return n;
}

static int parse_fields_data(char* const buffer, double* data)
{
	char* val[MAXFIELD];
	int n = parse_fields(buffer, val);
	for (int i = 0; i < n; ++i)
		data[i] = atof(val[i]);
	return n;
}

static FILE* set_output_file(const char* fname, const char* key)
{
	char filename[255] = { 0 }, outfilename[255] = { 0 };
	strcpy(filename, fname);
	char* temp = strrchr(filename, '.');
	if (temp) temp[0] = '\0';
	sprintf(outfilename, "%s-%s", filename, key);
	return fopen(outfilename, "w");
}

static void print_log(char** val, int num)
{
	for (int i = 0; i < num; ++i)
		printf("%s%c", val[i], (i + 1) == num ? '\n' : (i + 2) == num ? '*' : ',');
}

static void deg2dms(double deg, double* dms)
{
	double sign = deg < 0.0 ? (-1.0) : (1.0), a = fabs(deg);
	dms[0] = floor(a); a = (a - dms[0]) * 60.0;
	dms[1] = floor(a); a = (a - dms[1]) * 60.0;
	dms[2] = a; dms[0] *= sign;
}
extern int outnmea_gga(unsigned char* buff, float time, int type, double* blh, int ns, float dop, float age)
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
	p += sprintf(p, "$GPGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,",
		ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, blh[0] >= 0 ? "N" : "S",
		dms2[0], dms2[1] + dms2[2] / 60.0, blh[1] >= 0 ? "E" : "W", type,
		ns, dop, blh[2] - h, h, age);
	for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q; /* check-sum */
	p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
	return (int)(p - (char*)buff);
}

#ifndef MAX_BUF_LEN
#define MAX_BUF_LEN 1024
#endif

typedef struct
{
	uint8_t dat[MAX_BUF_LEN];
	uint32_t nbyte;
}nmea_buff_t;

static int add_buff(nmea_buff_t* buff, uint8_t data)
{
	int ret = 0;
	if (buff->nbyte >= MAX_BUF_LEN) buff->nbyte = 0;
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
		if ((data = fgetc(fIMU)) == EOF) break;
		if (!add_buff(&buff, data)) continue;

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
	if (fGGA) fclose(fGGA);
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
			" Acceleration Xv(m/s²),"
			" Acceleration Yv(m/s²),"
			" Acceleration Zv(m/s²),"
			//" Acceleration forward(m/s²),"
			//" Acceleration lateral(m/s²),"
			//" Acceleration down(m/s²),"
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

		// Print the 	AccelX (m/s²)
		if (nrx->mIsAxValid) fprintf(fp, "%10.3f", nrx->mAx);
		fprintf(fp, ",");

		// Print the 	AccelY (m/s²)
		if (nrx->mIsAyValid) fprintf(fp, "%10.3f", nrx->mAy);
		fprintf(fp, ",");

		// Print the 	AccelZ (m/s²)
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

		// Print the 	AccelForward (m/s²)
		//if (nrx->mIsAfValid) fprintf(fp, "%10.3f", nrx->mAf);
		//fprintf(fp, ",");

		// Print the 	AccelLateral (m/s²)
		//if (nrx->mIsAlValid) fprintf(fp, "%10.3f", nrx->mAl);
		//fprintf(fp, ",");

		// Print the 	AccelDown (m/s²)
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

int main(int argc, char** argv)
{
	if (argc < 3)
	{
		//decode_a1_asc_file_ins("D:\\data\\GlencoeSkiCentre\\ins.csv");
		//decode_a1_asc_file_gps("D:\\data\\GlencoeSkiCentre\\gps.csv");
		//decode_a1_asc_file_imu("D:\\data\\GlencoeSkiCentre\\imu.csv");
		read_oxts_data("D:\\austin\\UpsideDownWithLeverArmMeasurements\\2022_8_5_1525_drive2_Tag SN40434-022.ncom");
	}
	else
	{
		merge_data_file(argv[1], argv[2]);
	}
	return 0;
}