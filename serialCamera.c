/** \file serialCamera.c
 *  \brief Grove devices support library 
 */

/**
\addtogroup Grove devices
@{
 **************************************************************************																					
 *                                OpenPicus                 www.openpicus.com
 *                                                            italian concept
 * 
 *            openSource wireless Platform for sensors and Internet of Things	
 * **************************************************************************
 *  FileName:        serialCamera.c
 *  Dependencies:    OpenPicus libraries
 *  Module:          FlyPort WI-FI - FlyPort ETH
 *  Compiler:        Microchip C30 v3.12 or higher
 *
 *  Author               Rev.    Date              Comment
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Davide Vicca	     1.0     17/04/2013		   First release  
 *  
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  Software License Agreement
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  This is free software; you can redistribute it and/or modify it under
 *  the terms of the GNU General Public License (version 2) as published by 
 *  the Free Software Foundation AND MODIFIED BY OpenPicus team.
 *  
 *  ***NOTE*** The exception to the GPL is included to allow you to distribute
 *  a combined work that includes OpenPicus code without being obliged to 
 *  provide the source code for proprietary components outside of the OpenPicus
 *  code. 
 *  OpenPicus software is distributed in the hope that it will be useful, but 
 *  WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *  more details. 
 * 
 * 
 * Warranty
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * WE ARE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR
 * CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE.
 *
 **************************************************************************/

#include "taskFlyport.h"
#include "grovelib.h"
#include "serialCamera.h"


#define MINFREESPACE	50	//(KB)
#define PIC_BUF_LEN 	120

#define TIMEOUT_CAM	 	2E5

unsigned long timer;

/*constants*/
const unsigned char ACK_RESET[] ={
	0x76,
	0x00,
	0x26,
	0x00
};


const unsigned char ACK_IMAGESIZE[] =
{
	0x76,
	0x00,
	0x31,
	0x00,
	0x00
};

const unsigned char ACK_BAUD[] =
{
	0x76,
	0x00,
	0x24,
	0x00,
	0x00
};

const unsigned char ACK_TAKEPIC[] =
{
	0x76,
	0x00,
	0x36,
	0x00,
	0x00
};

const unsigned char ACK_COMPRESS[] =
{
	0x76,
	0x00,
	0x31,
	0x00,
	0x00
};

const unsigned char ACK_JPGSIZE[] =
{
	0x76,
	0x00,
	0x34,
	0x00,
	0x04,
	0x00,
	0x00
};

const unsigned char ACK_READIMAGE[] =
{
	0x76,
	0x00,
	0x32,
	0x00,
	0x00,
	0xFF,
	0xD9,
};

const unsigned char RESET[] = 
{
	0x56,
	0x00,
	0x26,
	0x00
};


const unsigned char IMAGE320x240[] = 
{
	0x56,
	0x00,
	0x31,
	0x05,
	0x04,
	0x01,
	0x00,
	0x19,
	0x11
};
const unsigned char IMAGE640x480[] = 
{
	0x56,
	0x00,
	0x31,
	0x05,
	0x04,
	0x01,
	0x00,
	0x19,
	0x00
};
const unsigned char TAKEPIC[] = 
{
	0x56,
	0x00,
	0x36,
	0x01,
	0x00
};

const unsigned char COMPRESS[] = 
{
	0x56,
	0x00,
	0x31,
	0x05,
	0x01,
	0x01,
	0x12,
	0x04
};

const unsigned char BAUD[] = 
{
	0x56,
	0x00,
	0x24,
	0x03,
	0x01,
	0x00,
	0x00,
};

const unsigned char JPGSIZE[] = 
{
	0x56,
	0x00,
	0x34,
	0x01,
	0x00
};

const unsigned char JPGREAD[] = 
{
	0x56,
	0x00,
	0x32,
	0x0C,
	0x00,
	0x0A,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x00,
	0x0A
};

/**
 * struct SerialCam - Struct SerialCam Grove Sensor Device
 */
struct SerialCam
{
	const void *class;
	struct Interface *inter;
	BYTE uart_module;
	BYTE uart_interface;
	BYTE compression;
	BYTE imageSize;
	BYTE baudrate;
#ifdef SDCARD
	FIL *image;
#endif
};



struct Interface *attachSensorToUartBus(void *,int,int,BYTE,BYTE);



/**
 * static void send_command (const unsigned char* com, int length,int uart_port) - Send command to the camera
 * \const unsigned char* com - command to be sent to the camera.
 *\int length - lenght of the command to be sent
 *\int uart_port - uart module used 
 * \return - void
*/
static void send_command (const unsigned char* com, int length,int uart_port)
{
	int i;
	for (i=0;i<length;i++)
		UARTWriteCh(uart_port,*(char*)(com + i));
}

/**
 * static int reset_camera(int uartPort) - Camera reset function
 *\int uartPort - UART port used for the Flyport - Camera communication
* \return - The status of the operation
 <UL>
	<LI><Breturn = 0:</B> the operation was successful.</LI> 
	<LI><B>return = -1:</B> the operation was unsuccessful.</LI> 
 </UL>
 */
static int reset_camera(int uartPort)
{
	unsigned char mess[4];
	int i;
	timer = TIMEOUT_CAM;
	UARTFlush(uartPort);
	send_command(RESET,4,uartPort);
	while((UARTBufferSize(uartPort)< 4) && timer)
		timer--;
	UARTRead(uartPort, (char *) mess,4);			
	for(i = 0; i < 4;i++)
	{
		if(mess[i] != ACK_RESET[i])
			return -1;
		else
			i++;
	}
	return 0;
}


 
/**
 * static int setImage(int uartPort,BYTE size) - Set image size function
 *\int uartPort - UART port used for the Flyport - Camera communication
 *\BYTE size - Image size set
* \return - The status of the operation
 <UL>
	<LI><Breturn = 0:</B> the operation was successful.</LI> 
	<LI><B>return = -1:</B> the operation was unsuccessful.</LI> 
 </UL>
 */
static int setImage(int uartPort,BYTE size)
{
	unsigned char mess[5];
	int i;
	timer = TIMEOUT_CAM;
	DelayMs(5);
	UARTFlush(uartPort);
	if(size)
		send_command(IMAGE640x480,9,uartPort);		
	else
		send_command(IMAGE320x240,9,uartPort);

	while((UARTBufferSize(uartPort)< 5) && timer)
		timer--;
	UARTRead(uartPort, (char *) mess,5);			
	for(i = 0; i < 5;i++)
	{
		if(mess[i] != ACK_IMAGESIZE[i])
			return -1;
		else
			i++;
	}
	return 0;
}

/**
 * static int setCompression(int uartPort,BYTE compr)- Set the compression rate of JPEG file
 *\int uartPort - UART port used for the Flyport - Camera communication
 * \BYTE compr - compression rate
 * \return - The status of the operation
 <UL>
	<LI><Breturn = 0:</B> the operation was successful.</LI> 
	<LI><B>return = -1:</B> the operation was unsuccessful.</LI> 
 </UL>
 */
static int setCompression(int uartPort,BYTE compr)
{
	unsigned char mess[5];
	int i;
	timer = TIMEOUT_CAM;
	BYTE *compression = &compr;
	UARTFlush(uartPort);
	send_command(COMPRESS,8,uartPort);
	send_command(compression,1,uartPort);

	while((UARTBufferSize(uartPort)< 5) && timer)
		timer--;
	UARTRead(uartPort, (char *) mess,5);			
	for(i = 0; i < 5;i++)
	{
		if(mess[i] != ACK_COMPRESS[i])
			return -1;
		else
			i++;
	}
	return 0;
}


/**
 * static int setBaud(int uartPort,BYTE baud)- Set the Baud rate of UART communication
 *\int uartPort - UART port used for the Flyport - Camera communication
 * \BYTE baud - Baud rate
 * \return - The status of the operation
 <UL>
	<LI><Breturn = 0:</B> the operation was successful.</LI> 
	<LI><B>return = -1:</B> the operation was unsuccessful.</LI> 
 </UL>
 */
static int setBaud(int uartPort,BYTE baud)
{
	unsigned char mess[5];
	int i;
	timer = TIMEOUT_CAM;
	long baudrate;
	BYTE BAUD_[2];
	switch(baud)
	{
		case _38400:
		baudrate = 38400;
		BAUD_[0] = 0x2A;
		BAUD_[1] = 0xf2;
		break;
		case _57600:
		baudrate = 57600;
		BAUD_[0] = 0x1C;
		BAUD_[1] = 0x4C;
		break;
		case _115200:
		baudrate = 115200;
		BAUD_[0] = 0x0D;
		BAUD_[1] = 0xA6;
	}
	UARTFlush(uartPort);
	send_command(BAUD,5,uartPort);		
	send_command(BAUD_,2,uartPort);		

	while((UARTBufferSize(uartPort)< 5) && timer)
		timer--;
	UARTRead(uartPort, (char *) mess,5);			
	for(i = 0; i < 5;i++)
	{
		if(mess[i] != ACK_BAUD[i])
			return -1;
		else
			i++;
	}
	UARTOff(uartPort);	
	UARTInit(uartPort,baudrate);
	UARTOn(uartPort);	
	return 0;
}


/**
 * static int setTakePic(int uartPort)- Take an image snaphot
 *\int uartPort - UART port used for the Flyport - Camera communication
 * \return - The status of the operation
 <UL>
	<LI><Breturn = 0:</B> the operation was successful.</LI> 
	<LI><B>return = -1:</B> the operation was unsuccessful.</LI> 
 </UL>
 */
static int setTakePic(int uartPort)
{
	unsigned char mess[4];
	int i;
	timer = TIMEOUT_CAM;
	DelayMs(5);
	UARTFlush(uartPort);

	send_command(TAKEPIC,5,uartPort);		

	while((UARTBufferSize(uartPort)< 4) && timer)
		timer--;
	UARTRead(uartPort, (char *) mess,4);			
	for(i = 0; i < 4;i++)
	{
		if(mess[i] != ACK_TAKEPIC[i])
			return -1;
		else
			i++;
	}
	return 0;
}


/**
 * static unsigned  int getSize(int uartPort)- Get the Image shot
 *\int uartPort - UART port used for the Flyport - Camera communication
 * \return - The size of image shot
 <UL>
	<LI><Breturn = 0:</B> the operation was unsuccessful.</LI> 
	<LI><B>return > 0 :</B> The size of image.</LI> 
 </UL>
 */
static unsigned int getSize(int uartPort)
{
	
	unsigned char mess[9];
	int i;
	timer = TIMEOUT_CAM;
	DelayMs(5);
	UARTFlush(uartPort);
	send_command(JPGSIZE,5,uartPort);		
	while((UARTBufferSize(uartPort)< 9) && timer)
		timer--;
	UARTRead(uartPort, (char *) mess,9);			
	for(i = 0; i < 5;i++)
	{
		if(mess[i] != ACK_JPGSIZE[i])
			return 0;
		else
			i++;
	}
	i++;
	return ((unsigned int)(mess[i]<<8)|mess[++i]);
}	



/**
 * static unsigned  int getChunck(unsigned char uart_port,BYTE *image,BYTE start, BYTE end)- 
 *\ - Get single image frame
 *\unsigned char uartPort - UART port used for the Flyport - Camera communication
 *\ BYTE *image - The image data sent back 
 *\ BYTE start - The start address from retriving image data
  *\BYTE end - The end address from retriving image data
  * \return - The status of the operation
 <UL>
	<LI><Breturn = 0:</B> the operation was successful.</LI> 
	<LI><B>return = -1 :</B> An error occurred with Camera transfer.</LI> 
	<LI><B>return = -2 :</B> The single image frame exceeded the 246 byte lenght.</LI> 
 </UL>
 */
static unsigned  int getChunck(int uart_port,char *image,unsigned int startAdd,BYTE len)
{
	int k;
	if(len > 0xF6)
		return -2;
	timer = TIMEOUT_CAM;
	
	BYTE tmp_buffer_write[10];

	unsigned char transfer_picture_command[16];
	for(k = 0;k<16;k++)
		transfer_picture_command[k] = JPGREAD[k];
													
	transfer_picture_command[8] = (startAdd>>8 & 0xFF);
    transfer_picture_command[9] = startAdd & 0xFF;
	transfer_picture_command[13] = len;			  
   	if(!startAdd)
		Delay10us(50);
	UARTFlush(uart_port);
  	send_command(transfer_picture_command,16,uart_port);
	vTaskDelay(1);
	while((UARTBufferSize(uart_port) < (len+10)) && timer)
		timer--;
 	UARTRead(uart_port,(char *)tmp_buffer_write,5); 
 	UARTRead(uart_port,(char *)image,len); 
 	UARTRead(uart_port,(char *)tmp_buffer_write+5,5); 
  	for(k=0;k<5;k++)
	{		
		if(tmp_buffer_write[k] != ACK_READIMAGE[k])
			return -1;
		if(tmp_buffer_write[k+5] != ACK_READIMAGE[k])
			return -1;
	}
	return 0;
}


/**
 * static int getImage(unsigned int size,unsigned char uart_port,FIL *imag)- Get the full image
 *\ unsigned int size - The image size 
 *\ unsigned char uartPort - UART port used for the Flyport - Camera communication
 *\ FIL *imag - The file pointer where to save the image 
  * \return - The status of the operation
 <UL>
	<LI><Breturn = 0:</B> the operation was successful.</LI> 
	<LI><B>return = -1 :</B> An error occurred with Camera transfer.</LI> 
	<LI><B>return = -2 :</B> An error occurred with the file saving.</LI> 
 </UL>
 */
#ifdef SDCARD

	static int getImage(unsigned int size,int uart_port,FIL *imag)
	{
	    int i,k;	
		int dataWritten;
		unsigned int count = size/PIC_BUF_LEN;
		int tail = size%PIC_BUF_LEN;
		unsigned int addr = 0;
		unsigned char tmp_buffer_write[10+PIC_BUF_LEN];
	
		unsigned char transfer_picture_command[16]; 
		for(i = 0;i<16;i++)
			transfer_picture_command[i] = JPGREAD[i];
		for(i = 0;i<(10+PIC_BUF_LEN);i++)
			tmp_buffer_write[i] = 0;
		// setting to start from first location of memory													
		transfer_picture_command[8] = 0;
	    transfer_picture_command[9] = 0;
		// set PIC_BUF_LEN as length of memory to get	
		transfer_picture_command[13] = PIC_BUF_LEN;		
	    for(i = 0; i < count; i++)
		{
			timer = TIMEOUT_CAM;
			send_command(transfer_picture_command,16,uart_port);
		    
			while((UARTBufferSize(uart_port) < (PIC_BUF_LEN+10)) && timer)
				timer--;
	  		UARTRead(uart_port,(char*) tmp_buffer_write,(unsigned char)sizeof(tmp_buffer_write)); 
			for(k=0;k<5;k++)
			{		
				if(tmp_buffer_write[k] != ACK_READIMAGE[k])
					return -1;
				if(tmp_buffer_write[k+PIC_BUF_LEN+5] != ACK_READIMAGE[k])
					return -1;
			}
			dataWritten = SDStreamWrite(imag,(char *)(tmp_buffer_write+5),PIC_BUF_LEN);
			if(dataWritten < PIC_BUF_LEN)
				return -2;
			addr += PIC_BUF_LEN;
			transfer_picture_command[8] = addr >> 8;
			transfer_picture_command[9] = addr & 0x00FF;
		}
		
		timer =TIMEOUT_CAM;
		transfer_picture_command[13] = tail;
		send_command(transfer_picture_command,16,uart_port);
	
		while((UARTBufferSize(uart_port) < (tail+10)) && timer)
			timer--;
	  	
		UARTRead(uart_port,(char *)tmp_buffer_write,10+tail);
		for(k=0;k<2;k++)
		{		
			if(tmp_buffer_write[k+tail+3] != ACK_READIMAGE[5+k])
				return -1;
		}
		dataWritten = SDStreamWrite(imag,(char *)(tmp_buffer_write+5),tail);
		if(dataWritten < tail)
			return -2;
		if(!SDStreamEOF(imag))
			return -2;
		if(!SDStreamClose(imag))
			return -2;
		else
			return 0;
		}	
	
#endif



/**
 * static void *SerialCam_ctor (void * _self, va_list *app)- Serial Camera grove device Constructor  
 * \param *_self - pointer to the Serial Camera grove device class.
 * \param *app 
 * \		1- uart module
* \return - Pointer to the Serial Camera devices instantiated
*/
static void *SerialCam_ctor (void * _self, va_list *app)
{
	struct SerialCam *self = _self;
	self->uart_module =  va_arg(*app, BYTE);
	switch(self->uart_module)
	{
		case 2 :
		{
			self->uart_interface = 7;
			break;
		}
		case 3 :
		{
			self->uart_interface = 9;
			break;
		}
		case 4 :
		{
			self->uart_interface = 11;
			break;
		}			
	}
	self->inter 	= NULL;
	self->imageSize = SIZE640x480;
	self->compression= 0x36 ;
	self->baudrate = _115200;
#ifdef SDCARD
	self->image = NULL;
#endif
	return self;

}	


/**
 * static void SerialCam_dtor (void * _sensor) - Serial Camera grove device Destructor  
 * \param *_sensor - pointer to the Serial Camera grove device class.
 * \return - None
*/
static void SerialCam_dtor (void * _sensor)
{
	struct SerialCam *sensor = _sensor;
	if(sensor->inter)
	{
	#ifdef SDCARD
		free(sensor->image);
	#endif
		free(sensor->inter->port);
		free(sensor->inter);
	}
}	


/**
 * static void *SerialCam_attach (void * _board,void *_sensor,int port) - attach a Serial Camera grove device to the GroveNest I2C port  
 * \param *_board - pointer to the GroveNest 
 * \param *_sensor - pointer to the Serial Camera grove device class.
 * \param ic2bus -  which DIG port the device is connected to
  * \return - The status of the operation
 <UL>
	<LI><Breturn = Pointer to the DIG interface created:</B> the operation was successful.</LI> 
	<LI><B>return = NULL:</B> the operation was unsuccessful.</LI> 
 </UL>
 */
static void *SerialCam_attach (void * _board,void *_sensor,int port)
{
	struct SerialCam *sensor = _sensor;
	sensor->inter = attachSensorToUartBus(_board,port,9600,sensor->uart_module,sensor->uart_interface);
	UARTInit(sensor->uart_module,115200);
	return sensor->inter;
}	


/**
 * static int SerialCam_config(void *_self,va_list *app) - Initilazed SD MEMORY CARD  
  * \return - The status of the operation
 <UL> 
	<LI><B>return = 0:</B> the operation was successful.</LI> 
	<LI><B>return = -1:</B> SD card is not initilazed.</LI> 
 </UL>
 */
static int SerialCam_config(void *_self,va_list *app)
{ 
	#ifdef SDCARD
	struct SerialCam *self = _self;
	if(!SDOnNest(SD_GROVE, 200))
			return -1;
		if(SDFreeSpace() < MINFREESPACE)
			return -1;
		self->image = (FIL *)malloc(sizeof(FIL));
	if(!self->image)
		return -1;
	else
		return 0;
	#endif
		return -1;
}	


/**
 *  static int SerialCam_set(void *_self,va_list *app) -  Set the camera parameters.
 * \BYTE param -  The parameter to be set.
 * \BYTE param1 -  The parameter data to be set
 * \return:
 <UL>
	<LI><Breturn = 0:</B> the operation was unsuccessful.</LI> 
	<LI><B>return > 0 :</B> The size of image.</LI> 
 </UL>
 */
static int SerialCam_set(void *_self,va_list *app)
{ 
	struct SerialCam *self = _self;
	BYTE param = va_arg(*app, BYTE);
	BYTE param1 = va_arg(*app, BYTE);
	int i = 0;
	int ret = 0;
	switch(param)
	{
		case BAUDRATE:
			self->baudrate = param1;
		break;
		case IMAGESIZE:
			self->imageSize = param1;
		break;
		case COMPRESSION:
			self->compression = param1;
		break;
		case TAKEPICTURE:
			UARTOn(self->uart_module);	
			for(;i<40;i++)
			{
				ret = reset_camera(self->uart_module);
				if(!ret)
					break;
			}
			if(ret)
				break;
			vTaskDelay(35);
			UARTOff(self->uart_module);
			UARTInit(self->uart_module,115200);
			UARTOn(self->uart_module);	
			ret = setBaud(self->uart_module,self->baudrate);							
			if(ret)
				break;
			ret = setImage(self->uart_module,self->imageSize);
			if(ret)
				break;
			ret = setCompression(self->uart_module,self->compression);
			if(ret)
				break;
			ret = setTakePic(self->uart_module);							
		break;
	}	
	UARTOff(self->uart_module);
	return ret;
}

/**
 * static float SerialCam_get(void * _self, va_list *app) -  Get the image data.
 * \BYTE param  - The function to be perfomed 
 <UL>
	<LI><B>param = SIZE   :</B> Return the image size</LI> 
	<LI><B>param = STREAM :</B> Return a image part  </LI> 
	<LI><B>param = SD     :</B> Return a image frame  </LI> 
 </UL>
 * \return:
 <UL>
	<LI><B>return = 0:</B> the operation was successful.</LI> 
	<LI><B>return = -1:</B> The camera transfer was unsuccessful</LI> 
	<LI><B>return = -2:</B> An error occurred with the file saving</LI> 
 </UL>
*/
static float SerialCam_get(void * _self, va_list *app)
{
	struct SerialCam *self = _self;
	UARTOn(self->uart_module);	
	BYTE param = va_arg(*app, BYTE);
	UARTFlush(self->uart_module);	
	unsigned int size;
	if(param != STREAM)
		size = getSize(self->uart_module);
	if((param == SIZE) || (size == 0))
	{
		UARTOff(self->uart_module);
		if(!size)
			return 0;
		else
			return (float) size;
	}
	char *text;
	text =  va_arg(*app, char *);
	int ret = 0;
	if(param == SD)
	{
		ret = -3;
	#ifdef SDCARD
		if(!SDFileCreate(text))
			ret = -2;
		else
		{
			if(!SDStreamOpen((FIL *)&self->image,text))
				ret = -2;
		else
				ret = getImage(size,self->uart_module,(FIL *)&self->image);
		}	
	#endif
	}
	
	if(param == STREAM)
	{
		unsigned int startAddr =  va_arg(*app, unsigned int);		
		BYTE lenght = va_arg(*app, BYTE);		
		ret = getChunck((int) self->uart_module, text,startAddr,lenght);
	}
	UARTOff(self->uart_module);	
	return ret;
}



static const struct SensorClass _SerialCam =
{	
	sizeof(struct SerialCam),
	SerialCam_ctor,
	SerialCam_dtor,
	SerialCam_attach,
	SerialCam_config,
	SerialCam_set,
	SerialCam_get,
};

const void *SerialCam = &_SerialCam;

