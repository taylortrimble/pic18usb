#include "DeviceConstants.h"

#define	NUM_CONFIGURATIONS	1
#define	NUM_INTERFACES		1
#define NUM_STRINGS			2
#define MAX_PACKET_SIZE		8	// maximum packet size for low-speed peripherals is 8 bytes, for full-speed peripherals it can be 8, 16, 32, or 64 bytes

// Define the states for Control EndPoints
#define	EP_IDLE_STATE		0x00
#define	EP_SETUP_STATE		0x01
#define	EP_DISABLED_STATE	0xff

#define	ENDPT_DISABLED		0x00
#define ENDPT_IN_ONLY		0x12
#define ENDPT_OUT_ONLY		0x14
#define ENDPT_CONTROL		0x16	// enable for in, out and setup
#define ENDPT_NON_CONTROL	0x1E	// enable for in, and out

#define INT_STAT_MASK_RESET	0x01
#define INT_STAT_MASK_ERROR	0x02
#define INT_STAT_MASK_TOKEN_DONE	0x04
#define INT_STAT_MASK_SLEEP	0x08
#define INT_STAT_MASK_STALL	0x10

// Buffer descriptor status bits
#define UOWN                (0x80)
#define DTS                 (0x40)
#define DTSEN               (0x08)
#define BSTALL              (0x04)

#define TOKEN_OUT	(0x01<<2)
#define TOKEN_ACK	(0x02<<2)
#define TOKEN_IN	(0x09<<2)
#define TOKEN_SETUP	(0x0D<<2)

// standard descriptor types
#define	DEVICE		1
#define	CONFIGURATION	2
#define	STRING		3
#define	INTERFACE	4
#define	ENDPOINT	5

// HID class descriptor types
#define HID			0x21
#define REPORT		0x22
#define PHYSICAL	0x23

// offsets from the beginning of the setup data record
#define	bmRequestType_OFFSET	0x00
#define	bRequest_OFFSET         0x01
#define	wValueL_OFFSET		0x02
#define	wValueH_OFFSET	0x03
#define	wIndexL_OFFSET		0x04
#define	wIndexH_OFFSET	0x05
#define	wLengthL_OFFSET		0x06
#define	wLengthH_OFFSET	0x07

// Offsets in device descriptor
#define bLength_OFFSET             (0)
#define bDescriptorType_OFFSET     (1)

// Standard USB requests
#define NO_REQUEST		0xFF
#define	GET_STATUS		0x00
#define	CLEAR_FEATURE	0x01
#define	SET_FEATURE		0x03
#define	SET_ADDRESS		0x05
#define	GET_DESCRIPTOR	0x06
#define SET_DESCRIPTOR	0x07
#define	GET_CONFIGURATION	0x08
#define	SET_CONFIGURATION	0x09
#define	GET_INTERFACE	0x0A
#define	SET_INTERFACE	0x0B
#define SYNCH_FRAME		0x0C

// HID Class requests
#define GET_REPORT		0x01
#define GET_IDLE		0x02
#define GET_PROTOCOL	0x03
#define SET_REPORT		0x09
#define SET_IDLE		0x0A
#define SET_PROTOCOL	0x0B

#define	GET_STRING_DESCRIPTOR	0x66
#define	HID_SET_REPORT		0x21
#define	VEND_SET_MEMORY		0x80

#define	SVCUSBINT		0x01 << 2
#define	SVCTOKENDONE		0x02 << 2
#define	SVCRESET		0x03 << 2
#define	SVCSLEEP		0x04 << 2
#define	SVCSTALL		0x05 << 2
#define	SVCERROR		0x06 << 2
#define	SVCACTIVITY		0x07 << 2
#define	TOKENOUT		0x08 << 2
#define	TOKENIN			0x09 << 2
#define	TOKENSETUP		0x0A << 2
#define	CLEARFEATURE		0x0B << 2
#define	GETCONFIG		0x0C << 2
#define	GETDESCRIPTOR		0x0D << 2
#define	GETINTERFACE		0x0E << 2
#define	GETSTATUS		0x0F << 2
#define	SETADDRESS		0x10 << 2
#define	SETCONFIG		0x11 << 2
#define	SETFEATURE		0x12 << 2
#define	SETINTERFACE		0x13 << 2
#define	FINISHSETADDRESS	0x14 << 2
#define	COPYDESC2EP0		0x15 << 2
#define	COPYSTRINGDESC2EP0	0x16 << 2
#define	ZEROLENPACKET		0x17 << 2

#define EP0				0x00 << 3
#define EP1				0x01 << 3
#define EP2				0x02 << 3

#define STANDARD		0x00 << 5
#define CLASS			0x01 << 5
#define VENDOR			0x02 << 5

#define RECIPIENT_DEVICE	0x00
#define RECIPIENT_INTERFACE	0x01
#define RECIPIENT_ENDPOINT	0x02

#define DEVICE_REMOTE_WAKEUP	0x01
#define ENDPOINT_HALT			0x00

typedef enum _USBDeviceState {
    USBDeviceStateReset,
    USBDeviceStateInitialized,
    USBDeviceStateAddressed,
    USBDeviceStateConfigured
} USBDeviceState;

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

#pragma udata USB_BUFFER_DESCRIPTOR_TABLE=0x0200
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

#pragma udata USB_BUFFERS=0x0280
far unsigned char _USBEP0OutBuffer[MAX_PACKET_SIZE];
far unsigned char _USBEP0InBuffer[MAX_PACKET_SIZE];

void ProcessSetupToken(void);
void ProcessInToken(void);
void ProcessOutToken(void);
void StandardRequests(void);
void ClassRequests(void);
void VendorRequests(void);
void SendDescriptorPacket(void);
