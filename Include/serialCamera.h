#define 			SDCARD			

#define BAUDRATE		0
#define IMAGESIZE		1
#define TAKEPICTURE		2
#define COMPRESSION		3

#define SIZE320x240		0
#define SIZE640x480		1


#define _38400			2
#define _57600			3
#define _115200			4

#define SD				23
#define STREAM			22
#define SIZE			24


extern const void *SerialCam;

#ifdef SDCARD
	#include "sd_manager.h"
#endif
