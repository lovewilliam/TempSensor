/* Name: hiddata.c
 * Author: Christian Starkjohann
 * Creation Date: 2008-04-11
 * Tabsize: 4
 * Copyright: (c) 2008 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: GNU GPL v2 (see License.txt), GNU GPL v3 or proprietary (CommercialLicense.txt)
 * This Revision: $Id$
 */

#include <stdio.h>
#include "hiddata.h"

#include <string.h>
#include <usb.h>

#define usbDevice   usb_dev_handle  /* use libusb's device structure */

/* ------------------------------------------------------------------------- */

#define DS18S20_ID 0x10
#define DS18B20_ID 0x28
#define DS18X20_FRACCONV 625
#define DS18X20_ID_LENGTH 6

#define SENSOR_ID_UNKNOWN 0
#define SENSOR_QUERY_OK 1
#define SENSOR_QUERY_FAILURE  0 

#define USBRQ_HID_GET_REPORT    0x01
#define USBRQ_HID_SET_REPORT    0x09

/* USB Temp & Humidity Sensor */
#define USBRQ_HID_GET_TEMPERATURE 0xd1
#define USBRQ_HID_GET_HUMIDITY    0xd2
#define USBRQ_HID_GET_RAW         0xd3

#define USB_HID_REPORT_TYPE_FEATURE 3

#define USBTEMP_CMD_TEST        0xe1
#define USBTEMP_CMD_NO_SENSORS  0xe2
#define USBTEMP_CMD_QUERY_SENSOR 0xe3
#define USBTEMP_CMD_GET_TEMP    0xe4

static int  usesReportIDs;

/* ------------------------------------------------------------------------- */

static int usbhidGetStringAscii(usb_dev_handle *dev, int index, char *buf, int buflen)
{
char    buffer[256];
int     rval, i;

    if((rval = usb_get_string_simple(dev, index, buf, buflen)) >= 0) /* use libusb version if it works */
        return rval;
    if((rval = usb_control_msg(dev, USB_ENDPOINT_IN, USB_REQ_GET_DESCRIPTOR, (USB_DT_STRING << 8) + index, 0x0409, buffer, sizeof(buffer), 5000)) < 0)
        return rval;
    if(buffer[1] != USB_DT_STRING){
        *buf = 0;
        return 0;
    }
    if((unsigned char)buffer[0] < rval)
        rval = (unsigned char)buffer[0];
    rval /= 2;
    /* lossy conversion to ISO Latin1: */
    for(i=1;i<rval;i++){
        if(i > buflen)              /* destination buffer overflow */
            break;
        buf[i-1] = buffer[2 * i];
        if(buffer[2 * i + 1] != 0)  /* outside of ISO Latin1 range */
            buf[i-1] = '?';
    }
    buf[i-1] = 0;
    return i-1;
}

int usbhidOpenDevice(usbDevice_t **device, int vendor, char *vendorName, int product, char *productName, int _usesReportIDs)
{
struct usb_bus      *bus;
struct usb_device   *dev;
usb_dev_handle      *handle = NULL;
int                 errorCode = USBOPEN_ERR_NOTFOUND;
static int          didUsbInit = 0;

    if(!didUsbInit){
        usb_init();
        didUsbInit = 1;
    }
    usb_find_busses();
    usb_find_devices();
    for(bus=usb_get_busses(); bus; bus=bus->next){
        for(dev=bus->devices; dev; dev=dev->next){
            if(dev->descriptor.idVendor == vendor && dev->descriptor.idProduct == product){
                char    string[256];
                int     len;
                handle = usb_open(dev); /* we need to open the device in order to query strings */
                if(!handle){
                    errorCode = USBOPEN_ERR_ACCESS;
                    fprintf(stderr, "Warning: cannot open USB device: %s\n", usb_strerror());
                    continue;
                }
                if(vendorName == NULL && productName == NULL){  /* name does not matter */
                    break;
                }
                /* now check whether the names match: */
                len = usbhidGetStringAscii(handle, dev->descriptor.iManufacturer, string, sizeof(string));
                if(len < 0){
                    errorCode = USBOPEN_ERR_IO;
                    fprintf(stderr, "Warning: cannot query manufacturer for device: %s\n", usb_strerror());
                }else{
                    errorCode = USBOPEN_ERR_NOTFOUND;
                    /* fprintf(stderr, "seen device from vendor ->%s<-\n", string); */
                    if(strcmp(string, vendorName) == 0){
                        len = usbhidGetStringAscii(handle, dev->descriptor.iProduct, string, sizeof(string));
                        if(len < 0){
                            errorCode = USBOPEN_ERR_IO;
                            fprintf(stderr, "Warning: cannot query product for device: %s\n", usb_strerror());
                        }else{
                            errorCode = USBOPEN_ERR_NOTFOUND;
                            /* fprintf(stderr, "seen product ->%s<-\n", string); */
                            if(strcmp(string, productName) == 0)
                                break;
                        }
                    }
                }
                usb_close(handle);
                handle = NULL;
            }
        }
        if(handle)
            break;
    }
    if(handle != NULL){
        errorCode = 0;
        *device = (void *)handle;
        usesReportIDs = _usesReportIDs;
    }
    return errorCode;
}

/* ------------------------------------------------------------------------- */

void    usbhidCloseDevice(usbDevice_t *device)
{
    if(device != NULL)
        usb_close((void *)device);
}

/* ------------------------------------------------------------------------- */

int usbhidSetReport(usbDevice_t *device, char *buffer, int len)
{
int bytesSent, reportId = buffer[0];

    if(!usesReportIDs){
        buffer++;   /* skip dummy report ID */
        len--;
    }
    bytesSent = usb_control_msg((void *)device, USB_TYPE_CLASS | USB_RECIP_DEVICE | USB_ENDPOINT_OUT, USBRQ_HID_SET_REPORT, USB_HID_REPORT_TYPE_FEATURE << 8 | (reportId & 0xff), 0, buffer, len, 5000);
    if(bytesSent != len){
        if(bytesSent < 0)
            fprintf(stderr, "Error sending message: %s\n", usb_strerror());
        return USBOPEN_ERR_IO;
    }
    return 0;
}

/* ------------------------------------------------------------------------- */

int usbhidGetReport(usbDevice_t *device, int reportNumber, char *buffer, int *len)
{
int bytesReceived, maxLen = *len;

    if(!usesReportIDs){
        buffer++;   /* make room for dummy report ID */
        maxLen--;
    }
    bytesReceived = usb_control_msg((void *)device, USB_TYPE_CLASS | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USBRQ_HID_GET_REPORT, USB_HID_REPORT_TYPE_FEATURE << 8 | reportNumber, 0, buffer, maxLen, 5000);
    if(bytesReceived < 0){
        fprintf(stderr, "Error sending message: %s\n", usb_strerror());
        return USBOPEN_ERR_IO;
    }
    *len = bytesReceived;
    if(!usesReportIDs){
        buffer[-1] = reportNumber;  /* add dummy report ID */
        (*len)++;
    }
    return 0;
}

int usbhidGetTemp(usbDevice_t *device, int reportNumber, char *buffer, int *len)
{
	int bytesReceived, maxLen = *len;

	if(!usesReportIDs){
		buffer++;   /* make room for dummy report ID */
		maxLen--;
	}
	bytesReceived = usb_control_msg((void *)device, USB_TYPE_CLASS | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USBRQ_HID_GET_TEMPERATURE, 0, 0, buffer, 1, 5000);
	if(bytesReceived < 0){
		fprintf(stderr, "Error sending message: %s\n", usb_strerror());
		return USBOPEN_ERR_IO;
	}
	*len = bytesReceived;
	if(!usesReportIDs){
		buffer[-1] = reportNumber;  /* add dummy report ID */
		(*len)++;
	}
	return 0;
}
int usbhidGetHumidity(usbDevice_t *device, int reportNumber, char *buffer, int *len)
{
	int bytesReceived, maxLen = *len;

	if(!usesReportIDs){
		buffer++;   /* make room for dummy report ID */
		maxLen--;
	}
	bytesReceived = usb_control_msg((void *)device, USB_TYPE_CLASS | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USBRQ_HID_GET_HUMIDITY, 0 , 0, buffer, 1, 5000);
	if(bytesReceived < 0){
		fprintf(stderr, "Error sending message: %s\n", usb_strerror());
		return USBOPEN_ERR_IO;
	}
	*len = bytesReceived;
	if(!usesReportIDs){
		buffer[-1] = reportNumber;  /* add dummy report ID */
		(*len)++;
	}
	return 0;
}
int usbhidGetRaw(usbDevice_t *device, int reportNumber, char *buffer, int *len)
{
	int bytesReceived, maxLen = *len;

	if(!usesReportIDs){
		buffer++;   /* make room for dummy report ID */
		maxLen--;
	}
	bytesReceived = usb_control_msg((void *)device, USB_TYPE_CLASS | USB_RECIP_DEVICE | USB_ENDPOINT_IN, USBRQ_HID_GET_RAW, 0 , 0, buffer, 5, 5000);
	if(bytesReceived < 0){
		fprintf(stderr, "Error sending message: %s\n", usb_strerror());
		return USBOPEN_ERR_IO;
	}
	*len = bytesReceived;
	if(!usesReportIDs){
		buffer[-1] = reportNumber;  /* add dummy report ID */
		(*len)++;
	}
	return 0;
}


int printSensors(usbDevice_t *device)
{
	unsigned char       buffer[8];
	int                 nBytes;
	unsigned char amount=0;
	printf("Checking number of sensors\n");
	if (getNumSensors((void*)device, &amount)) {
		fprintf(stderr, "%d sensor(s) found, querying\n", amount);
		int i;
		for (i=0; i<amount; i++) { 
		nBytes = usb_control_msg((void *)device, 
			USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
			USBTEMP_CMD_QUERY_SENSOR,	  // Command ID
			i,							  // Value 
			0,							  // Index 
			(char *) buffer, 
			sizeof(buffer), 
			5000);
		if(nBytes < 2){
			if(nBytes < 0) {
			fprintf(stderr, "USB error: %s\n", usb_strerror());
			}
			fprintf(stderr, "only %d bytes status received\n", nBytes);
			return SENSOR_QUERY_FAILURE;
		}
	  // TODO: Clean up, use struct to get clean response!
	  // buffer[0]: Type of sensor
	  // buffer[1-7]: 6-byte hardware id of sensor.
		fprintf(stderr, "sensor %d: ID ", i);
		int j;
		for(j=1; j<7; j++) {
			char id_part = buffer[j];
			printhex_byte(id_part);
		}
		fprintf(stderr, " type: ");
		if (buffer[0] == DS18S20_ID)
			fprintf(stderr, "(DS18S20)\n");
		else if (buffer[0] == DS18B20_ID)
			fprintf(stderr, "(DS18B20)\n");
		}
		return SENSOR_QUERY_OK;
	} else {
		return SENSOR_QUERY_FAILURE;
	}
}

int printTemperatureById(usbDevice_t *device,int sensorId,int raw_flag)
{
	unsigned char       buffer[8];
	unsigned char		  retval = SENSOR_QUERY_OK; // !=0 if an error occured.
	int                 nBytes;
	if(raw_flag==0)
		fprintf(stderr, "using sensor handle %u\n", sensorId);
	nBytes = usb_control_msg((void*)device, 
		USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
		USBTEMP_CMD_GET_TEMP,		  // Command ID
		sensorId,					  // sensor id 
		0,							  // Index 
		(char *) buffer, 
		sizeof(buffer), 
		5000);
	if(nBytes < 4){
		if(nBytes < 0) {
		fprintf(stderr, "USB error: %s\n", usb_strerror());
		}
		fprintf(stderr, "only %d bytes status received\n", nBytes);
		return SENSOR_QUERY_FAILURE;
	}
	unsigned char sensor = buffer[0];
	unsigned char subzero, cel, cel_frac_bits;
	subzero=buffer[1];
	cel=buffer[2];
	cel_frac_bits=buffer[3];
	if(raw_flag==0)
		fprintf(stderr, "reading sensor %d (°C): ", sensor);
	if (subzero)
		printf("-");
	else 
		printf("+");
	fprintf(stdout, "%d.", cel);
	fprintf(stdout, "%04d\n", cel_frac_bits*DS18X20_FRACCONV);
	return retval;
}

int printTemperatureByName(usbDevice_t *device, char* sensor_name,int raw_flag)
{
	int sensorId=0;
	if (getSensorById(device, sensor_name, &sensorId,raw_flag)) {
		fprintf(stderr, "Error: sensor not found.\n");
		return SENSOR_ID_UNKNOWN;
	}
	return printTemperatureById(device, sensorId, raw_flag);
}

unsigned char getNumSensors(usbDevice_t *device, unsigned char* amount)
{
	unsigned char       buffer[8];
	unsigned char           retval = SENSOR_QUERY_OK; // !=0 if an error occured.
	int                 nBytes;
	nBytes = usb_control_msg((void*)device,
			USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN,
			USBTEMP_CMD_NO_SENSORS,             // Command ID
			0,                                                          // Value 
			0,                                                          // Index 
			(char *) buffer,
			sizeof(buffer),
			5000);
	if(nBytes < 1){
		if(nBytes < 0) {
			fprintf(stderr, "USB error: %s\n", usb_strerror());
		}
		fprintf(stderr, "only %d bytes status received\n", nBytes);
		return USBOPEN_ERR_IO;
	}
	*amount=buffer[0];
	return retval;
}

int getSensorById(usbDevice_t *device, char* sensor_name, int* sensorId,int raw_flag)
{
	unsigned char       buffer[8];
	unsigned char		  retval = USBOPEN_SUCCESS; // !=0 if an error occured.
	int                 nBytes;
	if(raw_flag==0)
		fprintf(stderr, "searching sensor with id %s - ", sensor_name);
	unsigned char amount=0;
	if (getNumSensors(device, &amount)) {
		unsigned char found=1;
		*sensorId=SENSOR_ID_UNKNOWN; // make sure the value is defined.
		int i;
		for (i=0; i<amount; i++) {
			nBytes = usb_control_msg((void*)device,
				USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN, 
				USBTEMP_CMD_QUERY_SENSOR,	  // Command ID
				i,							  // Value 
				0,							  // Index 
				(char *) buffer,
				sizeof(buffer),
				5000);
			if(nBytes < 1+DS18X20_ID_LENGTH){
				if(nBytes < 0) {
					fprintf(stderr, "USB error: %s\n", usb_strerror());
					retval=USBOPEN_ERR_IO;
					return retval;
				}
				fprintf(stderr, "only %d bytes status received\n", nBytes);
			}
			found=1;
			int j;
			for(j=0; j<DS18X20_ID_LENGTH; j++) {
				char id_part = buffer[j+1];
				// Convert to string - nibble-wise hex representation.
				unsigned char lsn = id_part & 0x0f;
				if (lsn>9) lsn += 'A'-10;
				else lsn += '0';
				unsigned char msn = (id_part>>4) & 0x0f;
				if (msn>9) msn += 'A'-10;
				else msn += '0';
				// compare this byte to the given ID represenation
				if (! (msn == sensor_name[2*j] && lsn == sensor_name[2*j+1])) {
					found = 0;
					break;
				}
			}
			if (found) {
				*sensorId=i;
				break;
			}
		}
		return retval;
	} else {
		return SENSOR_QUERY_FAILURE;
	}
}

/* Developed by Martin Thomas, DS18X20 demo */
void printhex_nibble(const unsigned char b) {
	unsigned char  c = b & 0x0f;
	if (c>9) c += 'A'-10;
	else c += '0';
	fprintf(stderr, "%c", c);
} 

/* Developed by Martin Thomas, DS18X20 demo */
void printhex_byte(const unsigned char  b) {
	printhex_nibble(b>>4);
	printhex_nibble(b);
} 


