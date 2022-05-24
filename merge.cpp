// test.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include <iostream>
#include <cmath>
#include <cstdio>
#include <vector>
#include <string>
#include <algorithm>

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
				if (fabs(time - pvt.time) < 0.01 && pvt.nsat>0 && fabs(pvt.lat)<= 90.0 && fabs(pvt.lon) <= 360.0 && fabs(pvt.ht) <= 50000.0 && fabs(pvt.vn)<100.0 && fabs(pvt.ve) < 100.0 && fabs(pvt.vu) < 100.0)
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


int Output_GenerateMessage_GPS(double timeIMU, double timeGPS, double lat, double lon, double ht, double geod, double speed, double heading, double acc_h, double acc_v, double pdop, int nsat, unsigned char* buf) {
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
	p += sprintf(p, "%.4f,", acc_h);
	p += sprintf(p, "%.4f,", acc_v);
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

int merge_data_file(const char* imufname, const char *gpsfname)
{
	FILE* fIMU = NULL; 
	FILE* fOUT = NULL;
	FILE* fCSV = NULL;

	printf("%s\n%s\n", imufname, gpsfname);

	double time_offset = 0.0;
	std::vector<pvt_t> pvts;
	if (!read_sept_pvt(gpsfname, pvts)) return 0;
	if (!found_time_offset(imufname, &time_offset)) return 0;

	char buffer[1024] = { 0 };
	fIMU = fopen(imufname, "rb"); if (fIMU == NULL) return 0;
	fOUT = set_output_file(imufname, "-sept.log");
	fCSV = set_output_file(imufname, "-sync.csv");

	char* val[MAXFIELD];

	double rate = 0.0;

	double heading = 0.0, speed = 0.0, lat = 0.0, lon = 0.0, ht = 0.0, pdop = 0.0, time_GPS = 0.0, satnum = 0.0;

	double StartTime = 0;
	uint8_t data = 0;
	nmea_buff_t buff = { 0 };
	std::vector<pvt_t>::iterator pvt = pvts.begin();
	double last_heading = 0;
	unsigned long numofimu = 0;
	double wk = floor(time_offset / (7 * 24 * 3600));
	double ws_offset = time_offset - wk * (7 * 24 * 3600);
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
			double fxyz[3] = { atof(val[2]), atof(val[3]), atof(val[4]) };
			double wxyz[4] = { atof(val[5]), atof(val[6]), atof(val[7]), atof(val[8]) };
			double odr[2] = { atof(val[9]), atof(val[10]) * 1.0e-3 };

			if (fCSV)
			{
				fprintf(fCSV, "%15.7f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.4f,%15.7f\n"
					, timeIMU + ws_offset, fxyz[0], fxyz[1], fxyz[2], wxyz[0], wxyz[1], wxyz[2], wxyz[3], odr[0], odr[1]>0.01? (odr[1] + ws_offset) : (odr[1])
				);
			}

			while (pvt != pvts.end())
			{
				double timeIMU_GPS = pvt->time - time_offset;
				if (timeIMU_GPS <= timeIMU)
				{
					/*
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
					*/
					if (fabs(pvt->lat) > 100000.0 || fabs(pvt->lon) > 100000.0 || fabs(pvt->ht) > 100000.0 || fabs(pvt->vn) > 100000.0 || fabs(pvt->ve) > 100000.0 || fabs(pvt->vu) > 100000.0)
					{

					}
					else if (numofimu > 0)
					{
						double speed = sqrt(pvt->vn * pvt->vn + pvt->ve * pvt->ve);
						double heading = speed > 0.5 ? (atan2(pvt->ve, pvt->vn) * 180 / PI) : (last_heading);
						if (heading < 0)
							heading += 360.0;

						double acc_h = pvt->hpl / 4.5 / pvt->hdop;
						double acc_v = pvt->vpl / 4.5 / pvt->vdop;
						char gga_buff[255] = { 0 };
						Output_GenerateMessage_GPS(timeIMU_GPS, pvt->time, pvt->lat, pvt->lon, pvt->ht, pvt->geod, speed, heading, acc_h, acc_v, pvt->pdop, pvt->nsat, (unsigned char*)gga_buff);
						if (fOUT) fprintf(fOUT, "%s", gga_buff);
					}
					++pvt;
				}
				else
				{
					break;
				}
			}
			if (fOUT) fprintf(fOUT, "%s", buff.dat);
			++numofimu;
		}
		else if (strstr((char*)buff.dat, "#APINS") != NULL)
		{
			if (fOUT) fprintf(fOUT, "%s", buff.dat);
		}

		buff.nbyte = 0;
	}

	if (fIMU) fclose(fIMU);
	if (fOUT) fclose(fOUT);
	if (fCSV) fclose(fCSV);

	return 1;
}

int read_st_pvt(const char* pvtfname, std::vector< pvt_t>& pvts)
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
		/* time, lat, lon, ht, vn, ve, vd, refvar, stdX, stdY, stdZ, pdop, nsat
133255.2000000,  34.058338023,-106.897408318, 1397.9390,    0.0093,    0.0160,    0.0193,   0.22,   3.54,   7.07,   4.58,   1.95,19
133255.3000000,  34.058338167,-106.897408267, 1397.8389,    0.0183,    0.0135,    0.0469,   0.23,   3.54,   7.07,   4.58,   1.95,19
133255.4000000,  34.058337811,-106.897408569, 1397.9250,   -0.0012,   -0.0082,   -0.0246,   0.23,   3.54,   7.07,   4.58,   1.95,19
133255.5000000,  34.058337999,-106.897407839, 1397.7827,   -0.0132,   -0.0109,   -0.0340,   0.22,   3.54,   7.07,   4.58,   1.95,19
		*/
		char* val[MAXFIELD];
		int num = parse_fields(buffer, val);
		if (num < 13) continue;
		pvt.time = atof(val[0]) + 2200 * 7 * 24 * 3600;
		pvt.lat = atof(val[1]) * D2R;
		pvt.lon = atof(val[2]) * D2R;
		pvt.ht = atof(val[3]);
		pvt.vn = atof(val[4]);
		pvt.ve = atof(val[5]);
		pvt.vu = -atof(val[6]);
		double refvar = atof(val[7]);
		double stdXYZ[3] = { atof(val[8]), atof(val[9]), atof(val[10]) };
		double acc_h = sqrt(stdXYZ[0] * stdXYZ[0] + stdXYZ[1] * stdXYZ[1] + stdXYZ[2] * stdXYZ[2]) * refvar;
		pvt.acc_h = acc_h;
		pvt.acc_v = acc_h * 1.5;
		pvt.pdop = atof(val[10]);
		pvt.nsat = atof(val[11]);
		pvts.push_back(pvt);
		int wk = floor(pvt.time / (7 * 24 * 3600.0));
		double ws = pvt.time - wk * (7 * 24 * 3600.0);
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
	if (fGPS) fclose(fGPS);
	if (fOUT) fclose(fOUT);
	if (fGGA) fclose(fGGA);
	return pvts.size() > 0;
}

int merge_st_file(const char* imufname, const char* gpsfname)
{
	FILE* fIMU = NULL;
	FILE* fOUT = NULL;
	FILE* fCSV = NULL;

	printf("%s\n%s\n", imufname, gpsfname);

	double time_offset = 0.0;
	std::vector<pvt_t> pvts;
	if (!read_st_pvt(gpsfname, pvts)) return 0;
	if (!found_time_offset(imufname, &time_offset)) return 0;

	char buffer[1024] = { 0 };
	fIMU = fopen(imufname, "rb"); if (fIMU == NULL) return 0;
	fOUT = set_output_file(imufname, "-st.log");
	fCSV = set_output_file(imufname, "-st.csv");

	char* val[MAXFIELD];

	double rate = 0.0;

	double heading = 0.0, speed = 0.0, lat = 0.0, lon = 0.0, ht = 0.0, pdop = 0.0, time_GPS = 0.0, satnum = 0.0;

	double StartTime = 0;
	uint8_t data = 0;
	nmea_buff_t buff = { 0 };
	std::vector<pvt_t>::iterator pvt = pvts.begin();
	double last_heading = 0;
	unsigned long numofimu = 0;
	double wk = floor(time_offset / (7 * 24 * 3600));
	double ws_offset = time_offset - wk * (7 * 24 * 3600);
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
			double fxyz[3] = { atof(val[2]), atof(val[3]), atof(val[4]) };
			double wxyz[4] = { atof(val[5]), atof(val[6]), atof(val[7]), atof(val[8]) };
			double odr[2] = { atof(val[9]), atof(val[10]) * 1.0e-3 };

			if (fCSV)
			{
				fprintf(fCSV, "%15.7f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.6f,%10.4f,%15.7f\n"
					, timeIMU + ws_offset, fxyz[0], fxyz[1], fxyz[2], wxyz[0], wxyz[1], wxyz[2], wxyz[3], odr[0], odr[1] > 0.01 ? (odr[1] + ws_offset) : (odr[1])
				);
			}

			while (pvt != pvts.end())
			{
				double timeIMU_GPS = pvt->time - time_offset;
				if (timeIMU_GPS <= timeIMU)
				{
					if (numofimu > 0)
					{
						double speed = sqrt(pvt->vn * pvt->vn + pvt->ve * pvt->ve);
						double heading = speed > 0.5 ? (atan2(pvt->ve, pvt->vn) * 180 / PI) : (last_heading);
						if (heading < 0)
							heading += 360.0;

						//double acc_h = pvt->hpl / 4.5 / pvt->hdop;
						//double acc_v = pvt->vpl / 4.5 / pvt->vdop;
						char gga_buff[255] = { 0 };
						Output_GenerateMessage_GPS(timeIMU_GPS, pvt->time, pvt->lat, pvt->lon, pvt->ht, pvt->geod, speed, heading, pvt->acc_h, pvt->acc_v, pvt->pdop, pvt->nsat, (unsigned char*)gga_buff);
						if (fOUT) fprintf(fOUT, "%s", gga_buff);
					}
					++pvt;
				}
				else
				{
					break;
				}
			}
			if (fOUT) fprintf(fOUT, "%s", buff.dat);
			++numofimu;
		}
		else if (strstr((char*)buff.dat, "#APINS") != NULL)
		{
			if (fOUT) fprintf(fOUT, "%s", buff.dat);
		}

		buff.nbyte = 0;
	}

	if (fIMU) fclose(fIMU);
	if (fOUT) fclose(fOUT);
	if (fCSV) fclose(fCSV);

	return 1;
}

int main(int argc, char** argv)
{
	if (argc < 3)
	{
	}
	else
	{
		//merge_data_file(argv[1], argv[2]);
		merge_st_file(argv[1], argv[2]);
	}
	//merge_data_file("D:\\navfest\\WSMR_Day1\\sept\\Scenario_Unit1_1_2-input.txt", "D:\\navfest\\WSMR_Day1\\sept\\measasc.dat");
	//merge_st_file("D:\\navfest\\WSMR_Day1\\st\\Scenario_Unit1_1_2-input.txt", "D:\\navfest\\WSMR_Day1\\st\\st-day1-sol.csv");
	return 0;
}