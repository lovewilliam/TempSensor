/* Name: hidtool.c
 * Project: hid-data example
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-11
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * This Revision: $Id$
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include "hiddata.h"
#include "../firmware/usbconfig.h"  /* for device VID, PID, vendor name and product name */

#define USB_MAX_RETRIES 3
#define DS18X20_ID_LENGTH 6
/* ------------------------------------------------------------------------- */

static char *usbErrorMessage(int errCode)
{
	static char buffer[80];

	switch(errCode){
		case USBOPEN_ERR_ACCESS:      return "Access to device denied";
		case USBOPEN_ERR_NOTFOUND:    return "The specified device was not found";
		case USBOPEN_ERR_IO:          return "Communication error with device";
		default:
			sprintf(buffer, "Unknown USB error %d", errCode);
			return buffer;
	}
	return NULL;    /* not reached */
}

static usbDevice_t  *openDevice(void)
{
	usbDevice_t     *dev = NULL;
	unsigned char   rawVid[2] = {USB_CFG_VENDOR_ID}, rawPid[2] = {USB_CFG_DEVICE_ID};
	char            vendorName[] = {USB_CFG_VENDOR_NAME, 0}, productName[] = {USB_CFG_DEVICE_NAME, 0};
	int             vid = rawVid[0] + 256 * rawVid[1];
	int             pid = rawPid[0] + 256 * rawPid[1];
	int             err;

	if((err = usbhidOpenDevice(&dev, vid, vendorName, pid, productName, 0)) != 0){
        	fprintf(stderr, "error finding %s: %s\n", productName, usbErrorMessage(err));
		return NULL;
	}
	return dev;
}

/* ------------------------------------------------------------------------- */

static void hexdump(char *buffer, int len)
{
	int     i;
	FILE    *fp = stdout;

	for(i = 0; i < len; i++){
        	if(i != 0){
			if(i % 16 == 0){
				fprintf(fp, "\n");
			}else{
				fprintf(fp, " ");
			}
		}
		//fprintf(fp, "0x%02x", buffer[i] & 0xff);
		fprintf(fp,"%d",buffer[i]);
	}
	if(i != 0)
		fprintf(fp, "\n");
}

/*static int  hexread(char *buffer, char *string, int buflen)
{
char    *s;
int     pos = 0;

    while((s = strtok(string, ", ")) != NULL && pos < buflen){
        string = NULL;
        buffer[pos++] = (char)strtol(s, NULL, 0);
    }
    return pos;
}*/

/* ------------------------------------------------------------------------- */

static void usage(char *myName)
{
	fprintf(stderr, "usage:\n");
	fprintf(stderr, "  %s all\n", myName);
	fprintf(stderr, "  %s temperature\n", myName);
	fprintf(stderr, "  %s humidity\n", myName);
	fprintf(stderr, "  %s raw [for dht11 raw data]\n", myName);
	fprintf(stderr, "  %s sensors [detect ds18b20]\n", myName);
	fprintf(stderr, "  %s temp <sensor_id>\n",myName);
}

int main(int argc, char **argv)
{
	usbDevice_t *dev;
	char buffer[6];
	int raw = 0;
	int err;

	if(argc < 2)
	{
		usage(argv[0]);
		exit(1);
	}
	for(raw = argc-1;raw>0;raw--)
	{
		if(strcasecmp(argv[raw], "raw")==0)
		{
			raw = 1;
			break;
		}
	}
	if((dev = openDevice()) == NULL)
		exit(1);
	if(strcmp(argv[1],"sensors")==0)
	{
		char attempt=0, success=0, error=0;
		do {
			attempt++;
			success=printSensors(dev);
			if (!success) {
				fprintf(stderr, "Querying USBTemp failed, attempt %d (of %d)\n", attempt, USB_MAX_RETRIES);
				sleep(2*attempt);
			}
		} while ((!success) && attempt < USB_MAX_RETRIES);
		if (error) {
			fprintf(stderr, "Permanent problem querying USBTemp device. Giving up.\n");
			exit(1);
		}
	}else if(strcasecmp(argv[1], "all")==0)
	{
		int len = 6;
		if((err = usbhidGetRaw(dev, 0, buffer, &len)) != 0)
		{
			fprintf(stderr, "error reading data: %s\n", usbErrorMessage(err));
		}else{
			if(raw!=0)
			{
				//hexdump(buffer +1 , sizeof(buffer) -1);
				printf("%d,%d\n",buffer[3],buffer[1]);
			}else
			{
				printf("Temperature is : %d Celsus\n",buffer[3]);
				printf("Humidity is : %d \n",buffer[1]);
			}
		}
	}else if(strcasecmp(argv[1], "temperature") == 0)
	{
        	int len = 2;
		if((err = usbhidGetTemp(dev, 0, buffer, &len)) != 0)
		{
			fprintf(stderr, "error reading data: %s\n", usbErrorMessage(err));
		}else{
			if(raw!=0)
			{
				printf("%d\n",buffer[1]);
			}else
			{
				printf("Temperature is : %d Celsus\n",buffer[1]);
			}
			//hexdump(buffer + 1, sizeof(buffer) - 1);
		}
	}else if(strcasecmp(argv[1], "humidity") == 0)
	{
		int len = 2;
		if((err = usbhidGetHumidity(dev, 0, buffer, &len)) != 0){
			fprintf(stderr, "error reading data: %s\n", usbErrorMessage(err));
		}else{
			if(raw!=0)
			{
				printf("%d\n",buffer[1]);
			}else
			{
				printf("Humidity is : %d\n",buffer[1]);
			}
			//hexdump(buffer + 1, sizeof(buffer) - 1);
		}
	} else if (strcmp(argv[1], "temp") == 0)
	{
		if (argc < 3 || strlen(argv[2]) != 2*DS18X20_ID_LENGTH)
		{
			fprintf(stderr, "usage: usbtemp temp <SensorID>\n");
			exit(-1);
		}
		char attempt=0, success=0, error = 0;
		do {
			attempt++;
			success=printTemperatureByName(dev,argv[2],raw);
			if (! success) {
				fprintf(stderr, "Querying USBTemp failed, attempt %d (of %d)\n", attempt, USB_MAX_RETRIES);
				sleep(2*attempt);
			}
		} while ((!success) && attempt < USB_MAX_RETRIES);
		if (error) {
			fprintf(stderr, "Permanent problem querying USBTemp device. Giving up.\n");
			exit(1);
		}
	}else if(raw!=0)
	{
		int len = 6;
		if((err = usbhidGetRaw(dev, 0, buffer, &len)) != 0)
		{
			fprintf(stderr, "error reading data: %s\n", usbErrorMessage(err));
		}else{
			hexdump(buffer +1 , sizeof(buffer) -1);
		}
	}else{
		usage(argv[0]);
		exit(1);
	}
	usbhidCloseDevice(dev);
	return 0;
}

/* ------------------------------------------------------------------------- */
