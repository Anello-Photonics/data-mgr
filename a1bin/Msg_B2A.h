#ifndef __BINARYMESSAGE_To_STRING_H__
#define __BINARYMESSAGE_To_STRING_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "Message_def.h"

#define MSG_NAME_SIZE           6

#define X_AXIS                  0
#define Y_AXIS                  1
#define Z_AXIS                  2

#define  DEG_TO_RAD             (double)(0.017453292519943295)
#define  RAD_TO_DEG             (double)(57.29577951308232)


char * Convert_Message_B2A(void *ipMsg);

#ifdef __cplusplus
}
#endif

#endif