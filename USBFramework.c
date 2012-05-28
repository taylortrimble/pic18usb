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
#include <delays.h>
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
#define UPDATE_ENUM_STATUS(status) LATC=(1<<status);

#pragma udata
unsigned char _USBCurrentConfiguration;
USBDeviceStatus _USBDeviceStatus;
unsigned char _USBEngineStatus;
unsigned char _USBPendingDeviceAddress;
rom unsigned char *_USBDescriptorToSend;
unsigned char _USBDescriptorBytesLeft;
USBDeviceState _USBDeviceState;

#pragma romdata
rom const unsigned char _USBDeviceDescriptor[] = {
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

rom const unsigned char _USBConfigurationDescriptor[] = {
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

rom const rom const unsigned char *_USBConfigurationDescriptors[] = {
    _USBConfigurationDescriptor
};

rom const unsigned char _USBLangIDStringDescriptor[] = {    // LangID (special string descriptor at index 0)
    0x04,               // bLength
    STRING,             // bDescriptorType
    0x09,               // wLANGID[0] (low byte)
    0x04                // wLANGID[0] (high byte)
};

rom const unsigned char _USBManufacturerStringDescriptor[] = {
    0x36,               // bLength
    STRING,             // bDescriptorType
    'M', 0x00, 'i', 0x00, 'c', 0x00, 'r', 0x00, 'o', 0x00, 'c', 0x00, 'h', 0x00, 'i', 0x00, 'p', 0x00, ' ', 0x00,
    'T', 0x00, 'e', 0x00, 'c', 0x00, 'h', 0x00, 'n', 0x00, 'o', 0x00, 'l', 0x00, 'o', 0x00, 'g', 0x00, 'y', 0x00, ',', 0x00, ' ', 0x00,
    'I', 0x00, 'n', 0x00, 'c', 0x00, '.', 0x00
};

rom const unsigned char _USBDeviceStringDescriptor[] = {
    0x44,               // bLength
    STRING,             // bDescriptorType
    'E', 0x00, 'N', 0x00, 'G', 0x00, 'R', 0x00, ' ', 0x00, '2', 0x00, '2', 0x00, '1', 0x00, '0', 0x00, ' ', 0x00,
    'P', 0x00, 'I', 0x00, 'C', 0x00, '1', 0x00, '8', 0x00, 'F', 0x00, '2', 0x00, '4', 0x00, '5', 0x00, '5', 0x00, ' ', 0x00,
    'U', 0x00, 'S', 0x00, 'B', 0x00, ' ', 0x00,
    'F', 0x00, 'i', 0x00, 'r', 0x00, 'm', 0x00, 'w', 0x00, 'a', 0x00, 'r', 0x00, 'e', 0x00
};

rom const rom const unsigned char *_USBStringDescriptors[] = {
    _USBLangIDStringDescriptor,
    _USBManufacturerStringDescriptor,
    _USBDeviceStringDescriptor
};

#pragma code
void HighPriorityISR(void);
void LowPriorityISR(void);
USBTransaction _USBGetCurrentTransaction(void);
void _USBHandleControlError(void);
void _USBResetEP0InBuffer(void);
void _USBWriteValue(unsigned value, unsigned char bytes);
void _USBSendDescriptor(void);
void _USBWriteDescriptor(rom const unsigned char *descriptor, unsigned char bytes);
void _USBWriteSingleDescriptor(rom const unsigned char *descriptor);
void _USBProcessEP0(USBTransaction *transaction);
void _USBEngineReset(void);
void _USBConfigureBufferDescriptors(void);
void _USBSetup(void);

// Interrupt service routines
#pragma code __HIGH_PRIORITY_ISR__=0x0008
void _high_priority_isr(void) {_asm goto HighPriorityISR _endasm}
#pragma code __LOW_PRIORITY_ISR__=0x0018
void _low_priority_isr(void) {_asm goto LowPriorityISR _endasm}

#pragma interrupt HighPriorityISR
void HighPriorityISR(void)
{
    // Suspend when idle
    if (UIRbits.IDLEIF) {
        UCONbits.SUSPND = 1;
        UIRbits.IDLEIF = 0;
    }

    // Unsuspend when bus active
    if (UIRbits.ACTVIF) {
        UCONbits.SUSPND = 0;
        while (UIRbits.ACTVIF) {
            UIRbits.ACTVIF = 0;
        }
    }

    // Start of Frame - ignored
    if (UIRbits.SOFIF) {
        // Ignored
        UIRbits.SOFIF = 0;
    }

    // Bus stall - ignored
    if (UIRbits.STALLIF) {
        // Ignored
        UIRbits.STALLIF = 0;
    }

    // Error handling - ignored
    if (UIRbits.UERRIF) {
        // Ignored
        UEIR = 0x00;            // Clear all USB err flags
        UIRbits.UERRIF = 0;
    }

    // Handle USB reset
    if (UIRbits.URSTIF) {
        _USBEngineReset();
        UIRbits.URSTIF = 0;
    }

    // Handle USB transaction
    if (UIRbits.TRNIF) {
        USBTransaction transaction = _USBGetCurrentTransaction();
        // Route transaction to endpoint
        switch (transaction.endpoint) {
            case 0:
                _USBProcessEP0(&transaction);
                break;
            default:
                break;
        }
        // Clear flags
        UIRbits.TRNIF = 0;
    }
    
    PIR2bits.USBIF = 0;
}

#pragma interruptlow LowPriorityISR
void LowPriorityISR(void)
{
    // Low interrupt service routine
}

USBTransaction _USBGetCurrentTransaction(void)
{
    USBTransaction transaction;
    USBBufferDescriptor *bd;

    bd = (USBBufferDescriptor *)(0x200+USTAT);
    transaction.endpoint = (USTAT&0x38)>>3;         // Transaction endpoint is recorded in USTAT<5:3>
    transaction.token = (bd->status & 0x3C) >> 2;   // Transaction PID is recorded in BDnSTAT<5:2>
    transaction.bd = bd;                            // Point to the USB SIE buffer descriptor

    return transaction;
}

void _USBHandleControlError(void)
{
    // Reset EP0 OUT buffer capacity
    _USBBD0O.count = EP0_SIZE;

    // Stall endpoint 0
    _USBBD0O.status = (UOWN|BSTALL);
    _USBBD0I.status = (UOWN|BSTALL);
}

void _USBResetEP0InBuffer(void)
{
    _USBBD0I.count = 0;                    // Not sending a byte
    _USBBD0I.status = (UOWN|DTS|DTSEN);    // Set UOWN, send byte as DATA1
}

void _USBWriteValue(unsigned value, unsigned char bytes)
{
    unsigned char index;

    // Copy value to EP0 IN buffer 8bits at a time
    for (index = 0; index < bytes; index++) {
        _USBBD0I.address[index] = (value>>(index*8))&0x00FF;
    }

    // Return data to SIE through buffer descriptor
    _USBBD0I.count = bytes;
    _USBBD0I.status = (UOWN|DTS|DTSEN);    // Set UOWN, send as DATA1
}

void _USBSendDescriptor(void)
{
    unsigned char index;

    if (_USBDescriptorBytesLeft <= EP0_SIZE) {
        // Copy remaining bytes to EP0 IN buffer
        for (index = 0; index < _USBDescriptorBytesLeft; index++) {
            _USBBD0I.address[index] = *_USBDescriptorToSend++;
        }

        // Return data to SIE through buffer descriptor
        _USBBD0I.count = _USBDescriptorBytesLeft;
        _USBBD0I.status = _USBBD0I.status^DTS;                // Toggle the DATAx bit
        _USBBD0I.status = (_USBBD0I.status&DTS)|(UOWN|DTSEN); // Preserve DATAx, clear rest, set UOWN and DTSEN

        // Not sending a descriptor any more
        _USBEngineStatus &= ~USBEngineStatusSendingDescriptor;
        _USBDescriptorBytesLeft = 0;
    } else {
        // Copy a full length of bytes to EP0 IN buffer
        for (index = 0; index < EP0_SIZE; index++) {
            _USBBD0I.address[index] = *_USBDescriptorToSend++;
        }

        // Return data to SIE through buffer descriptor
        _USBBD0I.count = EP0_SIZE;
        _USBBD0I.status = _USBBD0I.status^DTS;                // Toggle the DATAx bit
        _USBBD0I.status = (_USBBD0I.status&DTS)|(UOWN|DTSEN); // Preserve DATAx, clear rest, set UOWN and DTSEN

        // Inform IN token processing to keep sending descriptor
        _USBEngineStatus |= USBEngineStatusSendingDescriptor;
        _USBDescriptorBytesLeft -= EP0_SIZE;
    }
}

void _USBWriteDescriptor(rom const unsigned char *descriptor, unsigned char bytes)
{
    // Simply setup system for global descriptor-sending function
    _USBDescriptorToSend = descriptor;
    _USBDescriptorBytesLeft = bytes;

    // Call the function that sends the first set of EP0_SIZE bytes
    _USBSendDescriptor();
}

void _USBWriteSingleDescriptor(rom const unsigned char *descriptor)
{
    _USBWriteDescriptor(descriptor, descriptor[bLength_OFFSET]);
}

void _USBProcessEP0(USBTransaction *transaction)
{
    unsigned char *transactionData;
    unsigned char bRequest;
    unsigned char bmRequestType;
    unsigned wValue;
    unsigned wIndex;

    transactionData = transaction->bd->address;

    switch (transaction->token) {  // extract PID bits
        case USBTokenSETUP:
            // SETUP token processing
            bRequest = transactionData[bRequest_OFFSET];
            bmRequestType = transactionData[bmRequestType_OFFSET];
            wValue = transactionData[wValueL_OFFSET] + (transactionData[wValueH_OFFSET])*0x100;
            wIndex = transactionData[wIndexL_OFFSET] + (transactionData[wIndexH_OFFSET])*0x100;
            
            // Reset OUT endpoint, dequeue any IN packets, and re-enable packet transfers
            _USBBD0O.status = ((!(bmRequestType&HOST_TO_DEVICE)&&wValue) ? (UOWN|DTS|DTSEN) : (UOWN|DTSEN)); // Choose correct DATA0/DATA1
            _USBBD0O.count = EP0_SIZE;
            _USBBD0I.status = DTSEN;   // steal UOWN to dequeue IN packets
            UCONbits.PKTDIS = 0;
            _USBEngineStatus = USBEngineStatusReset;
            
            // Move more up after testing/commit. The above is a combination of my code and Brad Minch's.
            switch (bRequest) {
                case GET_STATUS:
                    switch (bmRequestType) {
                        case USB_DEVICE_REQUEST:
                            _USBWriteValue(_USBDeviceStatus, 2);
                            break;

                        case USB_INTERFACE_REQUEST:
                            if ((_USBDeviceState == USBDeviceStateConfigured) && (wIndex<NUMBER_OF_USB_INTERFACES)) {
                                _USBWriteValue(0, 2);   // Always 0
                            } else {
                                _USBHandleControlError();
                            }
                            break;

                        case USB_ENDPOINT_REQUEST:
                            // Caluculate buffer descriptor status register and move BSTALL bit to bit <0>
                            if (_USBDeviceState == USBDeviceStateAddressed) {
                                _USBWriteValue((*(unsigned char *)(0x200+((wIndex&0x000F)*0x08)+((wIndex&0x0080)*0x04))&BSTALL)>>2, 2);
                            } else if (_USBDeviceState == USBDeviceStateConfigured) {
                                if (*(&UEP0+(wIndex&0x000F)) & (wIndex&0x80?EPINEN:EPOUTEN)) {  // If endpoint is enabled in specified directiong
                                    _USBWriteValue((*(unsigned char *)(0x200+((wIndex&0x000F)*0x08)+((wIndex&0x0080)*0x04))&BSTALL)>>2, 2);
                                } else {
                                    _USBResetEP0InBuffer();
                                }
                            } else {
                                _USBResetEP0InBuffer();
                            }
                            break;

                        default:
                            _USBHandleControlError();
                            break;
                    }
                    break;
                case CLEAR_FEATURE:
                    switch (bmRequestType) {
                        case USB_DEVICE_REQUEST:
                            switch (wValue) {
                                case DEVICE_REMOTE_WAKEUP:
                                    _USBDeviceStatus &= ~USBDeviceStatusDeviceRemoteWakeup;
                                    _USBResetEP0InBuffer();
                                    break;

                                case TEST_MODE:
                                    _USBDeviceStatus &= ~USBDeviceStatusTestMode;
                                    _USBResetEP0InBuffer();
                                    break;

                                default:
                                    _USBHandleControlError();
                                    break;
                            }
                            break;

                        case USB_ENDPOINT_REQUEST:
                            if (_USBDeviceState == USBDeviceStateAddressed) {
                                _USBResetEP0InBuffer();
                            } else if (_USBDeviceState == USBDeviceStateConfigured) {
                                if (*(&UEP0+(wIndex&0x000F)) & (wIndex&0x80?EPINEN:EPOUTEN)) {  // If endpoint is enabled in specified directiong
                                    *(unsigned char *)(0x200+((wIndex&0x000F)*0x08)+((wIndex&0x0080)*0x04)) = 0x00; // Clear endpoint descriptor
                                } else {
                                    _USBResetEP0InBuffer();
                                }
                            } else {
                                _USBHandleControlError();
                            }
                            break;

                        case USB_INTERFACE_REQUEST: // Should never happen
                        default:
                            _USBHandleControlError();
                            break;
                    }
                    break;
                    
                case SET_FEATURE:
                    switch (bmRequestType) {
                        case USB_DEVICE_REQUEST:
                            switch (wValue) {
                                case DEVICE_REMOTE_WAKEUP:
                                    _USBDeviceStatus |= USBDeviceStatusDeviceRemoteWakeup;
                                    _USBResetEP0InBuffer();
                                    break;

                                case TEST_MODE:
                                    _USBDeviceStatus |= USBDeviceStatusTestMode;
                                    _USBResetEP0InBuffer();
                                    break;

                                default:
                                    _USBHandleControlError();
                                    break;
                            }
                            break;

                        case USB_ENDPOINT_REQUEST:
                            if (_USBDeviceState == USBDeviceStateAddressed) {
                                _USBResetEP0InBuffer();
                            } else if (_USBDeviceState == USBDeviceStateConfigured) {
                                if (*(&UEP0+(wIndex&0x000F)) & (wIndex&0x80?EPINEN:EPOUTEN)) {  // If endpoint is enabled in specified directiong
                                    *(unsigned char *)(0x200+((wIndex&0x000F)*0x08)+((wIndex&0x0080)*0x04)) = (UOWN|BSTALL);    // Stall endpoint
                                } else {
                                    _USBResetEP0InBuffer();
                                }
                            } else {
                                _USBHandleControlError();
                            }
                            break;

                        case USB_INTERFACE_REQUEST: // Should never happen
                        default:
                            _USBHandleControlError();
                            break;
                    }
                    break;
                case SET_ADDRESS:
                    // Device only
                    if (wValue <= 127) {
                        _USBEngineStatus |= USBEngineStatusAddressing;
                        _USBPendingDeviceAddress = wValue;
                        _USBResetEP0InBuffer();
                    } else {
                        _USBHandleControlError();
                    }
                    break;

                case GET_DESCRIPTOR:
                    // Device only (bmRequestType)
                    switch ((wValue>>8)&0x00FF) { // High byte contains descriptor type
                        case DEVICE:
                            _USBWriteSingleDescriptor(_USBDeviceDescriptor);
                            break;
                        case CONFIGURATION:
                            _USBWriteDescriptor(_USBConfigurationDescriptors[wValue&0x00FF], (_USBConfigurationDescriptors[wValue&0x00FF])[wTotalLengthL_OFFSET]+((_USBConfigurationDescriptors[wValue&0x00FF])[wTotalLengthH_OFFSET])*0x100);
                            break;
                        case STRING:
                            _USBWriteSingleDescriptor(_USBStringDescriptors[wValue]);
                            break;
                        default:
                            _USBHandleControlError();
                            break;
                    }
                    break;
                case GET_CONFIGURATION:
                    // Device only
                    _USBWriteValue(_USBCurrentConfiguration, 1);
                    break;
                case SET_CONFIGURATION:
                    // Device only
                    if (wValue <= NUMBER_OF_CONFIGURATIONS) {
                        _USBCurrentConfiguration = wValue;
                        // Disable all endpoints except 0
#warning must update so that other endpoints are to work later
                        UEP1 = 0x00;
                        UEP2 = 0x00;
                        UEP3 = 0x00;
                        UEP4 = 0x00;
                        UEP5 = 0x00;
                        UEP6 = 0x00;
                        UEP7 = 0x00;
                        if (_USBCurrentConfiguration == 0) {
                            _USBDeviceState |= USBDeviceStateAddressed;
                            LATC = 0b0100;
                        } else {
                            _USBDeviceState = USBDeviceStateConfigured;
                            LATC = 0b1000;
                        }
                        _USBResetEP0InBuffer();
                    } else {
                        _USBHandleControlError();
                    }
                    break;
                case GET_INTERFACE:
                    // Interface only
                    if ((_USBDeviceState == USBDeviceStateConfigured) && (wIndex < NUMBER_OF_USB_INTERFACES)) {
                        if (wValue == 0) {  // Only support bAlternateSetting of 0 for EP0
                            _USBWriteValue(0, 1);
                        } else {
                            _USBHandleControlError();
                        }
                    } else {
                        _USBHandleControlError();
                    }
                    break;

                case SET_INTERFACE:
                    // Interface only
                    if ((_USBDeviceState == USBDeviceStateConfigured) && (wIndex < NUMBER_OF_USB_INTERFACES)) {
                        if (wValue == 0) {  // Only support bAlternateSetting of 0 for EP0
                            _USBResetEP0InBuffer();
                        } else {
                            _USBHandleControlError();
                        }
                    } else {
                        _USBHandleControlError();
                    }
                    break;
                case SET_DESCRIPTOR:    // Useless
                case SYNCH_FRAME:       // Unused
                default:
                    _USBHandleControlError();
                    break;
            }
            break;
        case USBTokenIN:
            // A pending USB address is committed on an IN token
            if (_USBEngineStatus & USBEngineStatusAddressing) {
                UADDR = _USBPendingDeviceAddress;
                switch (UADDR) {
                    case 0:
                        _USBDeviceState |= USBDeviceStateInitialized;
                        UPDATE_ENUM_STATUS(_USBDeviceState);
                        break;
                        
                    default:
                        _USBDeviceState = USBDeviceStateAddressed;
                        UPDATE_ENUM_STATUS(_USBDeviceState);
                        break;
                }
                _USBEngineStatus &= ~USBEngineStatusAddressing;
            }
            
            // Continue sending any unfinished descriptor transmission
            else if (_USBEngineStatus & USBEngineStatusSendingDescriptor) {
                _USBSendDescriptor();
            }
            break;
        case USBTokenOUT:
            // Any OUT token means no more IN tokens are necessary
            _USBResetEP0InBuffer();
            
            // Also reset the OUT Buffer; no OUT tokens required for EP0
            _USBBD0O.status = (UOWN|DTSEN); // SIE owns BD, data toggle enabled
            _USBBD0O.count = EP0_SIZE;      // Allow to receive full packet
            break;
        default:
            _USBHandleControlError();
            break;
    }
}

void _USBEngineReset(void)
{
    unsigned char *index;

    // Clear USTAT FIFO
    UIRbits.TRNIF = 0;
    UIRbits.TRNIF = 0;
    UIRbits.TRNIF = 0;
    UIRbits.TRNIF = 0;

    // Disable all endpoints
    for (index = (unsigned char *)&UEP0;
         index <= (unsigned char *)&UEP7;
         index++) {
        *index = 0x00;
    }

    // Reset endpoint buffer descriptors
    _USBConfigureBufferDescriptors();

    // Reset general USB
    _USBDeviceState = USBDeviceStateReset;
    _USBEngineStatus = USBEngineStatusReset;
    _USBDeviceStatus = 0x01;
    _USBCurrentConfiguration = 0;
    UADDR = 0;      // Reset USB bus address

    // Clear all USB interrupts, enable all USB error interrupts
    UEIE = 0x00;
    UEIR = 0x00;
    UIR = 0x00;
    UEIE = 0xFF;

    // Enable EP0
    UEP0 = 0x16;    // SETUP endpoint, not stalled
}

void _USBConfigureBufferDescriptors(void)
{
    _USBBD0I.status = (DTSEN);
    // IN buffer count is left alone
    _USBBD0I.address = _USBEP0InBuffer;
    _USBBD0O.status = (UOWN|DTSEN); // SIE owns BD, data toggle enabled
    _USBBD0O.count = EP0_SIZE;      // Allow to receive full packet
    _USBBD0I.address = _USBEP0OutBuffer;
}

void _USBSetup()
{
    // Setup
    LATC = 0x00;
    TRISC = 0xF0;

    // status = 0x01; for all disabled?????
    // Set device variables
    _USBDeviceState = USBDeviceStateReset;         // Reset USB state
    _USBEngineStatus = USBEngineStatusReset;
    _USBDeviceStatus = 0x01;
    _USBCurrentConfiguration = 0;
    UPDATE_ENUM_STATUS(_USBDeviceState);

    // General USB setup
    UEIE = 0x00;    // Disable and clear all USB interrupts
    UEIR = 0x00;
    UIE = 0x00;
    UIR = 0x00;
    UCON = 0x00;    // Resume signaling off, suspend mode off, USB off
    UCFG = 0x14;    // Eye off, pull ups enabled, full speed, ping pong all
    UADDR = 0x00;   // Reset USB address

    // Configure endpoint buffer descriptors
    _USBConfigureBufferDescriptors();

    // Configure endpoints
    UEP0 = 0x16;    // SETUP endpoint, not stalled

    // Enable USB
    Delay1KTCYx(24);        // Wait 2ms for clock to stabilize
    UCONbits.USBEN = 1;     // then enable USB
    while (UCONbits.SE0);   // Wait for first SE0 to end (marks bus power)

    // Configure USB interrupts
    UIEbits.ACTVIE = 1;
    UIEbits.IDLEIE = 1;
    UIEbits.SOFIE = 1;
    UIEbits.STALLIE = 1;
    UIEbits.URSTIE = 1;
    UIEbits.TRNIE = 1;  // Enable transaction complete interrupts
    PIE2bits.USBIE = 1; // Enable USB interrupts

    // Configure USB error interrupts
    UEIEbits.BTOEE = 1;
    UEIEbits.BTSEE = 1;
    UEIEbits.CRC16EE = 1;
    UEIEbits.CRC5EE = 1;
    UEIEbits.DFN8EE = 1;
    UEIEbits.PIDEE = 1;
    UIEbits.UERRIE = 1; // Enable USB error interrupts

    // Enable global interrupts
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;

    _USBDeviceState = USBDeviceStateInitialized;  // Set USB state to initialized
    UPDATE_ENUM_STATUS(_USBDeviceState);
}

void Setup()
{
    _USBSetup();
}

void main(void) {
    Setup();
    while (1) {
        // Main run loop
    }
}
