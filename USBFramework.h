#include "DeviceConstants.h"

#ifndef USBFRAMEWORK_H
#define USBFRAMEWORK_H

// Buffer descriptor status bits
#define UOWN                (0x80)
#define DTS                 (0x40)
#define DTSEN               (0x08)
#define BSTALL              (0x04)

// standard descriptor types
#define	USB_DEVICE_DESCRIPTOR_TYPE		1
#define	USB_CONFIGURATION_DESCRIPTOR_TYPE	2
#define	USB_STRING_DESCRIPTOR_TYPE		3
#define	USB_INTERFACE_DESCRIPTOR_TYPE	4
#define	USB_ENDPOINT_DESCRIPTOR_TYPE	5

// offsets from the beginning of the setup data record
#define	bmRequestType_OFFSET	0x00
#define	bRequest_OFFSET         0x01
#define	wValueL_OFFSET		0x02
#define	wValueH_OFFSET	0x03
#define	wIndexL_OFFSET		0x04
#define	wIndexH_OFFSET	0x05
#define	wLengthL_OFFSET		0x06
#define	wLengthH_OFFSET	0x07

// Offsets in configuration descriptor
#define wTotalLengthL_OFFSET       (2)
#define wTotalLengthH_OFFSET       (3)

// bmRequestType bits
#define HOST_TO_DEVICE      (0x80)

// Offsets in device descriptor
#define bLength_OFFSET             (0)
#define bDescriptorType_OFFSET     (1)

// Endpoint configuration bits
#define EPOUTEN     (0x04)
#define EPINEN      (0x02)

// Standard USB requests
#define NO_REQUEST		0xFF
#define	GET_STATUS		0x00
#define	CLEAR_FEATURE           0x01
#define	SET_FEATURE		0x03
#define	SET_ADDRESS		0x05
#define	GET_DESCRIPTOR          0x06
#define SET_DESCRIPTOR          0x07
#define	GET_CONFIGURATION	0x08
#define	SET_CONFIGURATION	0x09
#define	GET_INTERFACE           0x0A
#define	SET_INTERFACE           0x0B
#define SYNCH_FRAME		0x0C

#define USB_DEVICE_REQUEST      0x00
#define USB_INTERFACE_REQUEST   0x01
#define USB_ENDPOINT_REQUEST    0x02

#define DEVICE_REMOTE_WAKEUP	0x01
#define TEST_MODE               0x02
#define ENDPOINT_HALT           0x00

typedef enum _USBDeviceState {
    USBDeviceStateReset,
    USBDeviceStateInitialized,
    USBDeviceStateAddressed,
    USBDeviceStateConfigured
} USBDeviceState;

typedef enum _USBDeviceStatus {
    USBDeviceStatusTestMode =           0x01,
    USBDeviceStatusDeviceRemoteWakeup = 0x02,
} USBDeviceStatus;

typedef enum _USBEngineStatus {
    USBEngineStatusReset = 0x00,
    USBEngineStatusAddressing = 0x01,
    USBEngineStatusSendingDescriptor = 0x02
} USBEngineStatus;

typedef enum _USBToken {
    USBTokenSETUP = 0b1101,
    USBTokenIN =    0b1001,
    USBTokenOUT =   0b0001
} USBToken;

typedef struct _USBBufferDescriptor {
	unsigned char status;
	unsigned char count;
	unsigned char *address;
} USBBufferDescriptor;

typedef struct _USBTransaction {
    unsigned char endpoint;
    USBToken token;
    USBBufferDescriptor *bd;
} USBTransaction;

#pragma udata __USB_BUFFER_DESCRIPTOR_TABLE__=0x0200
far USBBufferDescriptor _USBBD0O;
far USBBufferDescriptor _USBBD0I;
far USBBufferDescriptor _USBBD1O;
far USBBufferDescriptor _USBBD1I;
far USBBufferDescriptor _USBBD2O;
far USBBufferDescriptor _USBBD2I;
far USBBufferDescriptor _USBBD3O;
far USBBufferDescriptor _USBBD3I;
far USBBufferDescriptor _USBBD4O;
far USBBufferDescriptor _USBBD4I;
far USBBufferDescriptor _USBBD5O;
far USBBufferDescriptor _USBBD5I;
far USBBufferDescriptor _USBBD6O;
far USBBufferDescriptor _USBBD6I;
far USBBufferDescriptor _USBBD7O;
far USBBufferDescriptor _USBBD7I;

#pragma udata __USB_ENDPOINT_BUFFERS__=0x0280
far unsigned char _USBEP0OutBuffer[EP0_SIZE];
far unsigned char _USBEP0InBuffer[EP0_SIZE];
