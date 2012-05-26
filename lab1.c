// 
// Author: Bradley A. Minch
// Organization: Franklin W. Olin College of Engineering
// Revision History: 
//     01/19/2006 - Added wait for initial SE0 condition to clear at the end
//                  of InitUSB().
//     01/13/2006 - Fixed problem with DATA OUT transfers (only worked properly
//                  with class requests) in ProcessSetupToken.  Added code to
//                  disable all EPs except EP0 on a valid SET_CONFIGURATION
//                  request.  Changed code to use BSTALL instead of EPSTALL for
//                  Request Error on EP0.  Changed CLEAR_FEATURE, SET_FEATURE,
//                  and GET_STATUS requests to use BSTALL instead of EPSTALL.  
//                  Eliminated initial for loop in main().
//     11/14/2005 - Initial port to C for the Microchip C18 compiler complete
//     06/22/2005 - Added code to disable all endpoints on USRTIF and to mask
//                  bits 0, 1, and 7 of USTAT on TRNIF in ServiceUSB.
//     04/21/2005 - Initial public release.
//
// ============================================================================
// 
// Peripheral Description:
// 
// This peripheral enumerates as a vendor-specific device. The main event loop 
// blinks an LED connected to RA1 on and off at about 2 Hz and the peripheral 
// responds to a pair of vendor-specific requests that turn on or off an LED
// connected to RA0.  The firmware is configured to use an external 4-MHz 
// crystal, to operate as a low-speed USB device, and to use the internal 
// pull-up resistor.
// 
// ============================================================================
//
// Software Licence Agreement:
// 
// THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION.  NO WARRANTIES, WHETHER 
// EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED 
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY 
// TO THIS SOFTWARE. THE AUTHOR SHALL NOT, UNDER ANY CIRCUMSTANCES, BE LIABLE 
// FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
// 
#include <p18f14k50.h>
#include "USBFramework.h"

// Configuration bits
#pragma config CPUDIV = NOCLKDIV
#pragma config USBDIV = OFF
#pragma config FOSC   = HS
#pragma config PLLEN  = ON
#pragma config FCMEN  = OFF
#pragma config IESO   = OFF
#pragma config PWRTEN = OFF
#pragma config BOREN  = OFF
#pragma config BORV   = 30
#pragma config WDTEN  = OFF
#pragma config WDTPS  = 32768
#pragma config MCLRE  = ON
#pragma config HFOFST = OFF
#pragma config STVREN = ON
#pragma config LVP    = OFF
#pragma config XINST  = OFF
#pragma config BBSIZ  = OFF
#pragma config CP0    = OFF
#pragma config CP1    = OFF
#pragma config CPB    = OFF
#pragma config WRT0   = OFF
#pragma config WRT1   = OFF
#pragma config WRTB   = OFF
#pragma config WRTC   = OFF
#pragma config EBTR0  = OFF
#pragma config EBTR1  = OFF
#pragma config EBTRB  = OFF
#pragma config DEBUG  = OFF

#define SHOW_ENUM_STATUS

#define SET_RA0         0x01        // vendor-specific request to set RA0 to high
#define CLR_RA0         0x02        // vendor-specific request to set RA0 to low

#pragma udata
USBBufferDescriptor gCurrentBufferDescriptor;
unsigned char gBufferData[8];
unsigned char gErrorCondition;
unsigned char gCurrentConfiguration;
unsigned char gUSBDeviceStatus;
unsigned char _USBEngineStatus;
unsigned char gUSBPendingAddress;
unsigned char rom *gDescriptorToSend;
unsigned char gUSBBytesLeft;
unsigned char gUSBPacketLength;
unsigned char gCachedUSTAT;
USBDeviceState _USBDeviceState;

#pragma romdata
rom const unsigned char gkDevice[] = {          // Device descriptor
    0x12,               // bLength
    DEVICE,             // bDescriptorType
    0x10,               // bcdUSB (low byte)
    0x01,               // bcdUSB (high byte)
    0x00,               // bDeviceClass
    0x00,               // bDeviceSubClass
    0x00,               // bDeviceProtocol
    MAX_PACKET_SIZE,    // bMaxPacketSize
    0xD8,               // idVendor (low byte)
    0x04,               // idVendor (high byte)
    0x01,               // idProduct (low byte)
    0x00,               // idProduct (high byte)
    0x00,               // bcdDevice (low byte)
    0x00,               // bcdDevice (high byte)
    0x01,               // iManufacturer
    0x02,               // iProduct
    0x00,               // iSerialNumber (none)
    NUM_CONFIGURATIONS  // bNumConfigurations
};

rom const unsigned char gkConfiguration1[] = {  // Single configuration descriptor (index 0)
    0x09,               // bLength
    CONFIGURATION,      // bDescriptorType
    0x12,               // wTotalLength (low byte)
    0x00,               // wTotalLength (high byte)
    NUM_INTERFACES,     // bNumInterfaces
    0x01,               // bConfigurationValue
    0x00,               // iConfiguration (none)
    0xA0,               // bmAttributes
    0x32,               // bMaxPower (100 mA)
    0x09,               // bLength (Interface1 descriptor starts here)
    INTERFACE,          // bDescriptorType
    0,                  // bInterfaceNumber
    0x00,               // bAlternateSetting
    0x00,               // bNumEndpoints (excluding EP0)
    0xFF,               // bInterfaceClass (vendor specific class code)
    0x00,               // bInterfaceSubClass
    0xFF,               // bInterfaceProtocol (vendor specific protocol used)
    0x00                // iInterface (none)
};

rom const unsigned char gkString0[] = {         // LangID (special string at index 0)
    0x04,               // bLength
    STRING,             // bDescriptorType
    0x09,               // wLANGID[0] (low byte)
    0x04                // wLANGID[0] (high byte)
};

rom const unsigned char gkString1[] = {         // Manufacturer string
    0x36,               // bLength
    STRING,             // bDescriptorType
    'M', 0x00, 'i', 0x00, 'c', 0x00, 'r', 0x00, 'o', 0x00, 'c', 0x00, 'h', 0x00, 'i', 0x00, 'p', 0x00, ' ', 0x00,
    'T', 0x00, 'e', 0x00, 'c', 0x00, 'h', 0x00, 'n', 0x00, 'o', 0x00, 'l', 0x00, 'o', 0x00, 'g', 0x00, 'y', 0x00, ',', 0x00, ' ', 0x00,
    'I', 0x00, 'n', 0x00, 'c', 0x00, '.', 0x00
};

rom const unsigned char gkString2[] = {         // Device string
    0x44,               // bLength
    STRING,             // bDescriptorType
    'E', 0x00, 'N', 0x00, 'G', 0x00, 'R', 0x00, ' ', 0x00, '2', 0x00, '2', 0x00, '1', 0x00, '0', 0x00, ' ', 0x00,
    'P', 0x00, 'I', 0x00, 'C', 0x00, '1', 0x00, '8', 0x00, 'F', 0x00, '2', 0x00, '4', 0x00, '5', 0x00, '5', 0x00, ' ', 0x00,
    'U', 0x00, 'S', 0x00, 'B', 0x00, ' ', 0x00,
    'F', 0x00, 'i', 0x00, 'r', 0x00, 'm', 0x00, 'w', 0x00, 'a', 0x00, 'r', 0x00, 'e', 0x00
};

#pragma code
void InitUSB(void);
void ServiceUSB(void);
void HandleError(void);
void ProcessSetupToken(void);
void ProcessInToken(void);
void ProcessOutToken(void);
void ProcessStandardRequests(void);
void ProcessClassRequests(void);
void ProcessVendorRequests(void);
void SendDescriptorPacket(void);

void InitUSB(void) {
    UIE = 0x00;                 // mask all USB interrupts
    UIR = 0x00;                 // clear all USB interrupt flags
    UCFG = 0x14;                // configure USB for low-speed transfers and to use the on-chip transciever and pull-up resistor
    UCON = 0x08;                // enable the USB module and its supporting circuitry
    gCurrentConfiguration = 0x00;
    _USBDeviceState = USBDeviceStateReset;         // default to powered state
    gUSBDeviceStatus = 0x01;
    _USBEngineStatus = USBEngineStatusReset;   // No device requests in process
#ifdef SHOW_ENUM_STATUS
    TRISC = 0x00;               // set all bits of LATC as outputs
    LATC = 0x01;                // set bit zero to indicate Powered status
#endif
    while (UCONbits.SE0);       // wait for the first SE0 to end
}

void ServiceUSB(void) {
    USBBufferDescriptor *currentBufferDescriptor;

    if (UIRbits.UERRIF) {
        UEIR = 0x00;
    } else if (UIRbits.SOFIF) {
        UIRbits.SOFIF = 0;
    } else if (UIRbits.IDLEIF) {
        UIRbits.IDLEIF = 0;
        UCONbits.SUSPND = 1;
#ifdef SHOW_ENUM_STATUS
        LATC &= 0xE0;
        LATCbits.LATC4 = 1;
#endif
    } else if (UIRbits.ACTVIF) {
        UIRbits.ACTVIF = 0;
        UCONbits.SUSPND = 0;
#ifdef SHOW_ENUM_STATUS
        LATC &= 0xE0;
        LATC |= 0x01<<_USBDeviceState;
#endif
    } else if (UIRbits.STALLIF) {
        UIRbits.STALLIF = 0;
    } else if (UIRbits.URSTIF) {
        gCurrentConfiguration = 0x00;
        UIRbits.TRNIF = 0;      // clear TRNIF four times to clear out the USTAT FIFO
        UIRbits.TRNIF = 0;
        UIRbits.TRNIF = 0;
        UIRbits.TRNIF = 0;
        UEP0 = 0x00;                // clear all EP control registers to disable all endpoints
        UEP1 = 0x00;
        UEP2 = 0x00;
        UEP3 = 0x00;
        UEP4 = 0x00;
        UEP5 = 0x00;
        UEP6 = 0x00;
        UEP7 = 0x00;
        _USBBD0O.bytecount = MAX_PACKET_SIZE;
        _USBBD0O.address = _USBEP0OutBuffer;  // EP0 OUT gets a buffer
        _USBBD0O.status = 0x88;             // set UOWN bit (USB can write)
        _USBBD0I.address = _USBEP0InBuffer;   // EP0 IN gets a buffer
        _USBBD0I.status = 0x08;             // clear UOWN bit (MCU can write)
        UADDR = 0x00;               // set USB Address to 0
        UIR = 0x00;             // clear all the USB interrupt flags
        UEP0 = ENDPT_CONTROL;   // EP0 is a control pipe and requires an ACK
        UEIE = 0xFF;            // enable all error interrupts
        _USBDeviceState = USBDeviceStateInitialized;
        gUSBDeviceStatus = 0x01;   // self powered, remote wakeup disabled
#ifdef SHOW_ENUM_STATUS
        LATC &= 0xE0;
        LATCbits.LATC1 = 1;     // set bit 1 of LATC to indicate Powered state
#endif
    } else if (UIRbits.TRNIF) {
        currentBufferDescriptor = (USBBufferDescriptor *)((unsigned char *)(&_USBBD0O)+(USTAT&0x7C));  // mask out bits 0, 1, and 7 of USTAT for offset into the buffer descriptor table
        gCurrentBufferDescriptor.status = currentBufferDescriptor->status;
        gCurrentBufferDescriptor.bytecount = currentBufferDescriptor->bytecount;
        gCurrentBufferDescriptor.address = currentBufferDescriptor->address;
        gCachedUSTAT = USTAT;      // save the USB status register
        UIRbits.TRNIF = 0;      // clear TRNIF interrupt flag
#ifdef SHOW_ENUM_STATUS
        switch (gCachedUSTAT&0x18) {   // toggle bit 5, 6, or 7 of LATC to reflect EP0, EP1, or EP2 activity
            case EP0:
                LATC ^= 0x20;
                break;
            case EP1:
                LATC ^= 0x40;
                break;
            case EP2:
                LATC ^= 0x80;
        }
#endif
        gErrorCondition = 0; // clear USB error flags
        switch (gCurrentBufferDescriptor.status&0x3C) {  // extract PID bits
            case TOKEN_SETUP:
                ProcessSetupToken();
                break;
            case TOKEN_IN:
                ProcessInToken();
                break;
            case TOKEN_OUT:
                ProcessOutToken();
        }
        if (gErrorCondition) {     // if there was a Request Error...
            _USBBD0O.bytecount = MAX_PACKET_SIZE;   // ...get ready to receive the next Setup token...
            _USBBD0I.status = 0x84;
            _USBBD0O.status = 0x84;                 // ...and issue a protocol stall on EP0
        }
    }
}

void HandleError()
{
    gErrorCondition = 1;
}

void ProcessSetupToken(void) {
    unsigned char n;

    for (n = 0; n<8; n++) {
        gBufferData[n] = gCurrentBufferDescriptor.address[n];
    }
    _USBBD0O.bytecount = MAX_PACKET_SIZE;   // reset the EP0 OUT byte count
    _USBBD0I.status = 0x08;         // return the EP0 IN buffer to us (dequeue any pending requests)
    _USBBD0O.status = (!(gBufferData[bmRequestType]&0x80) && (gBufferData[wLength] || gBufferData[wLengthHigh])) ? 0xC8:0x88;   // set EP0 OUT UOWN back to USB and DATA0/DATA1 packet according to the request type
    UCONbits.PKTDIS = 0;            // assuming there is nothing to dequeue, clear the packet disable bit
    _USBEngineStatus = USBEngineStatusReset;       // clear the device request in process
    switch (gBufferData[bmRequestType]&0x60) {  // extract request type bits
        case STANDARD:
            StandardRequests();
            break;
        case CLASS:
            ClassRequests();
            break;
        case VENDOR:
            VendorRequests();
            break;
        default:
            HandleError();    // set Request Error Flag
    }
}

void StandardRequests(void) {
    unsigned char *UEP;
    unsigned char n;
    USBBufferDescriptor *currentBufferDescriptor;

    switch (gBufferData[bRequest]) {
        case GET_STATUS:
            switch (gBufferData[bmRequestType]&0x1F) {  // extract request recipient bits
                case RECIPIENT_DEVICE:
                    _USBBD0I.address[0] = gUSBDeviceStatus;
                    _USBBD0I.address[1] = 0x00;
                    _USBBD0I.bytecount = 0x02;
                    _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
                    break;
                case RECIPIENT_INTERFACE:
                    switch (_USBDeviceState) {
                        case USBDeviceStateAddressed:
                            HandleError();    // set Request Error Flag
                            break;
                        case USBDeviceStateConfigured:
                            if (gBufferData[wIndex]<NUM_INTERFACES) {
                                _USBBD0I.address[0] = 0x00;
                                _USBBD0I.address[1] = 0x00;
                                _USBBD0I.bytecount = 0x02;
                                _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
                            } else {
                                HandleError();    // set Request Error Flag
                            }
                    }
                    break;
                case RECIPIENT_ENDPOINT:
                    switch (_USBDeviceState) {
                        case USBDeviceStateAddressed:
                            if (!(gBufferData[wIndex]&0x0F)) {  // get EP, strip off direction bit and see if it is EP0
                                _USBBD0I.address[0] = (((gBufferData[wIndex]&0x80) ? _USBBD0I.status:_USBBD0O.status)&0x04)>>2; // return the BSTALL bit of EP0 IN or OUT, whichever was requested
                                _USBBD0I.address[1] = 0x00;
                                _USBBD0I.bytecount = 0x02;
                                _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
                            } else {
                                HandleError();    // set Request Error Flag
                            }
                            break;
                        case USBDeviceStateConfigured:
                            UEP = (unsigned char *)&UEP0;
                            n = gBufferData[wIndex]&0x0F;   // get EP and strip off direction bit for offset from UEP0
                            currentBufferDescriptor = &_USBBD0O+((n<<1)|((gBufferData[wIndex]&0x80) ? 0x01:0x00)); // compute pointer to the buffer descriptor for the specified EP
                            if (UEP[n]&((gBufferData[wIndex]&0x80) ? 0x02:0x04)) { // if the specified EP is enabled for transfers in the specified direction...
                                _USBBD0I.address[0] = ((currentBufferDescriptor->status)&0x04)>>2; // ...return the BSTALL bit of the specified EP
                                _USBBD0I.address[1] = 0x00;
                                _USBBD0I.bytecount = 0x02;
                                _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
                            } else {
                                HandleError();    // set Request Error Flag
                            }
                            break;
                        default:
                            HandleError();    // set Request Error Flag
                    }
                    break;
                default:
                    HandleError();    // set Request Error Flag
            }
            break;
        case CLEAR_FEATURE:
        case SET_FEATURE:
            switch (gBufferData[bmRequestType]&0x1F) {  // extract request recipient bits
                case RECIPIENT_DEVICE:
                    switch (gBufferData[wValue]) {
                        case DEVICE_REMOTE_WAKEUP:
                            if (gBufferData[bRequest]==CLEAR_FEATURE)
                                gUSBDeviceStatus &= 0xFE;
                            else
                                gUSBDeviceStatus |= 0x01;
                            _USBBD0I.bytecount = 0x00;      // set EP0 IN byte count to 0
                            _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
                            break;
                        default:
                            HandleError();    // set Request Error Flag
                    }
                    break;
                case RECIPIENT_ENDPOINT:
                    switch (_USBDeviceState) {
                        case USBDeviceStateAddressed:
                            if (!(gBufferData[wIndex]&0x0F)) {  // get EP, strip off direction bit, and see if its EP0
                                _USBBD0I.bytecount = 0x00;      // set EP0 IN byte count to 0
                                _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
                            } else {
                                HandleError();    // set Request Error Flag
                            }
                            break;
                        case USBDeviceStateConfigured:
                            UEP = (unsigned char *)&UEP0;
                            if (n = gBufferData[wIndex]&0x0F) { // get EP and strip off direction bit for offset from UEP0, if not EP0...
                                currentBufferDescriptor = &_USBBD0O+((n<<1)|((gBufferData[wIndex]&0x80) ? 0x01:0x00)); // compute pointer to the buffer descriptor for the specified EP
                                if (gBufferData[wIndex]&0x80) { // if the specified EP direction is IN...
                                    if (UEP[n]&0x02) {  // if EPn is enabled for IN transfers...
                                        currentBufferDescriptor->status = (gBufferData[bRequest]==CLEAR_FEATURE) ? 0x00:0x84;
                                    } else {
                                        HandleError();    // set Request Error Flag
                                    }
                                } else {    // ...otherwise the specified EP direction is OUT, so...
                                    if (UEP[n]&0x04) {  // if EPn is enabled for OUT transfers...
                                        currentBufferDescriptor->status = (gBufferData[bRequest]==CLEAR_FEATURE) ? 0x88:0x84;
                                    } else {
                                        HandleError();    // set Request Error Flag
                                    }
                                }
                            }
                            if (!(gErrorCondition)) {  // if there was no Request Error...
                                _USBBD0I.bytecount = 0x00;
                                _USBBD0I.status = 0xC8;     // ...send packet as DATA1, set UOWN bit
                            }
                            break;
                        default:
                            HandleError();    // set Request Error Flag
                    }
                    break;
                default:
                    HandleError();    // set Request Error Flag
            }
            break;
        case SET_ADDRESS:
            if (gBufferData[wValue]>0x7F) { // if new device address is illegal, send Request Error
                HandleError();    // set Request Error Flag
            } else {
                _USBEngineStatus |= USBEngineStatusAddressing;  // processing a SET_ADDRESS request
                gUSBPendingAddress = gBufferData[wValue];  // save new address
                _USBBD0I.bytecount = 0x00;      // set EP0 IN byte count to 0
                _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
            }
            break;
        case GET_DESCRIPTOR:
            _USBEngineStatus |= USBEngineStatusSendingDescriptor;   // readying a GET_DESCRIPTOR request
            switch (gBufferData[wValueHigh]) {
                case DEVICE:
                    gDescriptorToSend = gkDevice;
                    gUSBBytesLeft = gDescriptorToSend[0];
                    if ((gBufferData[wLengthHigh]==0x00) && (gBufferData[wLength]<gUSBBytesLeft)) {
                        gUSBBytesLeft = gBufferData[wLength];
                    }
                    SendDescriptorPacket();
                    break;
                case CONFIGURATION:
                    switch (gBufferData[wValue]) {
                        case 0:
                            gDescriptorToSend = gkConfiguration1;
                            break;
                        default:
                            HandleError();    // set Request Error Flag
                    }
                    if (!(gErrorCondition)) {
                        gUSBBytesLeft = gDescriptorToSend[2];   // wTotalLength at an offset of 2
                        if ((gBufferData[wLengthHigh]==0x00) && (gBufferData[wLength]<gUSBBytesLeft)) {
                            gUSBBytesLeft = gBufferData[wLength];
                        }
                        SendDescriptorPacket();
                    }
                    break;
                case STRING:
                    switch (gBufferData[wValue]) {
                        case 0:
                            gDescriptorToSend = gkString0;
                            break;
                        case 1:
                            gDescriptorToSend = gkString1;
                            break;
                        case 2:
                            gDescriptorToSend = gkString2;
                            break;
                        default:
                            HandleError();    // set Request Error Flag
                    }
                    if (!(gErrorCondition)) {
                        gUSBBytesLeft = gDescriptorToSend[0];
                        if ((gBufferData[wLengthHigh]==0x00) && (gBufferData[wLength]<gUSBBytesLeft)) {
                            gUSBBytesLeft = gBufferData[wLength];
                        }
                        SendDescriptorPacket();
                    }
                    break;
                default:
                    HandleError();    // set Request Error Flag
            }
            break;
        case GET_CONFIGURATION:
            _USBBD0I.address[0] = gCurrentConfiguration;  // copy current device configuration to EP0 IN buffer
            _USBBD0I.bytecount = 0x01;
            _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
            break;
        case SET_CONFIGURATION:
            if (gBufferData[wValue]<=NUM_CONFIGURATIONS) {
                UEP1 = 0x00;    // clear all EP control registers except for EP0 to disable EP1-EP15 prior to setting configuration
                UEP2 = 0x00;
                UEP3 = 0x00;
                UEP4 = 0x00;
                UEP5 = 0x00;
                UEP6 = 0x00;
                UEP7 = 0x00;
                switch (gCurrentConfiguration = gBufferData[wValue]) {
                    case 0:
                        _USBDeviceState = USBDeviceStateAddressed;
#ifdef SHOW_ENUM_STATUS
                        LATC &= 0xE0;
                        LATCbits.LATC2 = 1;
#endif
                        break;
                    default:
                        _USBDeviceState = USBDeviceStateConfigured;
#ifdef SHOW_ENUM_STATUS
                        LATC &= 0xE0;
                        LATCbits.LATC3 = 1;
#endif
                }
                _USBBD0I.bytecount = 0x00;      // set EP0 IN byte count to 0
                _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
            } else {
                HandleError();    // set Request Error Flag
            }
            break;
        case GET_INTERFACE:
            switch (_USBDeviceState) {
                case USBDeviceStateConfigured:
                    if (gBufferData[wIndex]<NUM_INTERFACES) {
                        _USBBD0I.address[0] = 0x00; // always send back 0 for bAlternateSetting
                        _USBBD0I.bytecount = 0x01;
                        _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
                    } else {
                        HandleError();    // set Request Error Flag
                    }
                    break;
                default:
                    HandleError();    // set Request Error Flag
            }
            break;
        case SET_INTERFACE:
            switch (_USBDeviceState) {
                case USBDeviceStateConfigured:
                    if (gBufferData[wIndex]<NUM_INTERFACES) {
                        switch (gBufferData[wValue]) {
                            case 0:     // currently support only bAlternateSetting of 0
                                _USBBD0I.bytecount = 0x00;      // set EP0 IN byte count to 0
                                _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
                                break;
                            default:
                                HandleError();    // set Request Error Flag
                        }
                    } else {
                        HandleError();    // set Request Error Flag
                    }
                    break;
                default:
                    HandleError();    // set Request Error Flag
            }
            break;
        case SET_DESCRIPTOR:
        case SYNCH_FRAME:
        default:
            HandleError();    // set Request Error Flag
    }
}

void ClassRequests(void) {
    switch (gBufferData[bRequest]) {
        default:
            HandleError();    // set Request Error Flag
    }
}

void VendorRequests(void) {
    switch (gBufferData[bRequest]) {
        case SET_RA0:
            PORTAbits.RA0 = 1;      // set RA0 high
            _USBBD0I.bytecount = 0x00;      // set EP0 IN byte count to 0
            _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
            break;
        case CLR_RA0:
            PORTAbits.RA0 = 0;      // set RA0 low
            _USBBD0I.bytecount = 0x00;      // set EP0 IN byte count to 0
            _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
            break;
        default:
            HandleError();    // set Request Error Flag
    }
}

void ProcessInToken(void) {
    switch (gCachedUSTAT&0x18) {   // extract the EP bits
        case EP0:
            if (_USBEngineStatus & USBEngineStatusAddressing) {
                switch (UADDR = gUSBPendingAddress) {
                    case 0:
                        _USBDeviceState = USBDeviceStateInitialized;
#ifdef SHOW_ENUM_STATUS
                        LATC &= 0xE0;
                        LATCbits.LATC1 = 1;
#endif
                        break;
                    default:
                        _USBDeviceState = USBDeviceStateAddressed;
#ifdef SHOW_ENUM_STATUS
                        LATC &= 0xE0;
                        LATCbits.LATC2 = 1;
#endif
                }
                _USBEngineStatus &= ~USBEngineStatusAddressing;
            } else if (_USBEngineStatus & USBEngineStatusSendingDescriptor) {
                SendDescriptorPacket();
            } else {
                HandleError();
            }
            break;
        case EP1:
            break;
        case EP2:
            break;
    }
}

void ProcessOutToken(void) {
    switch (gCachedUSTAT&0x18) {   // extract the EP bits
        case EP0:
            _USBBD0O.bytecount = MAX_PACKET_SIZE;
            _USBBD0O.status = 0x88;
            _USBBD0I.bytecount = 0x00;      // set EP0 IN byte count to 0
            _USBBD0I.status = 0xC8;     // send packet as DATA1, set UOWN bit
            break;
        case EP1:
            break;
        case EP2:
            break;
    }
}

void SendDescriptorPacket(void) {
    unsigned char n;

    if (gUSBBytesLeft<MAX_PACKET_SIZE) {
        _USBEngineStatus &= ~USBEngineStatusSendingDescriptor;   // sending a short packet, no longer sending after this
        gUSBPacketLength = gUSBBytesLeft;
        gUSBBytesLeft = 0x00;
    } else {
        gUSBPacketLength = MAX_PACKET_SIZE;
        gUSBBytesLeft -= MAX_PACKET_SIZE;
    }
    for (n = 0; n<gUSBPacketLength; n++) {
        _USBBD0I.address[n] = *gDescriptorToSend++;
    }
    _USBBD0I.bytecount = gUSBPacketLength;
    _USBBD0I.status = ((_USBBD0I.status^0x40)&0x40)|0x88; // toggle the DATA01 bit, clear the PIDs bits, and set the UOWN and DTS bits
}

void main(void) {
    InitUSB();          // initialize the USB registers and serial interface engine
    while (1) {
        ServiceUSB();
    }
}
