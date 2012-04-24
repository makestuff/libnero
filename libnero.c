/*
 * Copyright (C) 2009-2012 Chris McClelland
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <makestuff.h>
#include <libusbwrap.h>
#ifdef WIN32
	#include <lusb0_usb.h>
#else
	#include <usb.h>
#endif
#include <liberror.h>
#include <vendorCommands.h>
#include "libnero.h"

// -------------------------------------------------------------------------------------------------
// Declaration of private types & functions
// -------------------------------------------------------------------------------------------------

typedef enum {
	SEND_ZEROS,
	SEND_ONES,
	SEND_DATA,
	SEND_MASK
} SendType;

enum {
	IS_RESPONSE_NEEDED = 0,
	IS_LAST = 1,
	SEND_TYPE = 2
};

static NeroStatus beginShift(
	struct NeroHandle *handle, uint32 numBits, SendType sendType, bool isLast,
	bool isResponseNeeded, const char **error
) WARN_UNUSED_RESULT;

static NeroStatus doSend(
	struct NeroHandle *handle, const uint8 *sendPtr, uint16 chunkSize, const char **error
) WARN_UNUSED_RESULT;

static NeroStatus doReceive(
	struct NeroHandle *handle, uint8 *receivePtr, uint16 chunkSize, const char **error
) WARN_UNUSED_RESULT;

static NeroStatus setEndpointSize(
	struct NeroHandle *handle, const char **error
) WARN_UNUSED_RESULT;

static NeroStatus setJtagMode(
	struct NeroHandle *handle, bool enable, const char **error
) WARN_UNUSED_RESULT;

// -------------------------------------------------------------------------------------------------
// Public functions
// -------------------------------------------------------------------------------------------------

// Find the NeroJTAG device, open it.
//
NeroStatus neroInitialise(
	struct usb_dev_handle *device, struct NeroHandle *handle, const char **error)
{
	NeroStatus returnCode, nStatus;
	handle->device = device;
	nStatus = setEndpointSize(handle, error);
	CHECK_STATUS(nStatus, "neroInitialise()", NERO_ENDPOINTS);
	nStatus = setJtagMode(handle, true, error);
	CHECK_STATUS(nStatus, "neroInitialise()", NERO_ENABLE);
	return NERO_SUCCESS;
cleanup:
	handle->device = NULL;
	handle->endpointSize = 0;
	return returnCode;
}

// Close the cable...drop the USB connection.
//
NeroStatus neroClose(struct NeroHandle *handle, const char **error) {
	NeroStatus returnCode, nStatus;
	if ( handle->device ) {
		nStatus = setJtagMode(handle, false, error);
		CHECK_STATUS(nStatus, "neroClose()", NERO_ENABLE);
	}
	returnCode = NERO_SUCCESS;
cleanup:
	handle->device = NULL;
	handle->endpointSize = 0;
	return returnCode;
}

// Shift data into and out of JTAG chain.
//   In pointer may be ZEROS (shift in zeros) or ONES (shift in ones).
//   Out pointer may be NULL (not interested in data shifted out of the chain).
//
NeroStatus neroShift(
	struct NeroHandle *handle, uint32 numBits, const uint8 *inData, uint8 *outData, bool isLast,
	const char **error)
{
	NeroStatus returnCode, nStatus;
	uint32 numBytes;
	uint16 chunkSize;
	SendType sendType;
	bool isResponseNeeded;

	if ( inData == ZEROS ) {
		sendType = SEND_ZEROS;
	} else if ( inData == ONES ) {
		sendType = SEND_ONES;
	} else {
		sendType = SEND_DATA;
	}
	if ( outData ) {
		isResponseNeeded = true;
	} else {
		isResponseNeeded = false;
	}
	nStatus = beginShift(handle, numBits, sendType, isLast, isResponseNeeded, error);
	CHECK_STATUS(nStatus, "neroShift()", NERO_BEGIN_SHIFT);
	numBytes = bitsToBytes(numBits);
	while ( numBytes ) {
		chunkSize = (numBytes>=handle->endpointSize) ? handle->endpointSize : (uint16)numBytes;
		if ( sendType == SEND_DATA ) {
			nStatus = doSend(handle, inData, chunkSize, error);
			CHECK_STATUS(nStatus, "neroShift()", NERO_SEND);
			inData += chunkSize;
		}
		if ( isResponseNeeded ) {
			nStatus = doReceive(handle, outData, chunkSize, error);
			CHECK_STATUS(nStatus, "neroShift()", NERO_RECEIVE);
			outData += chunkSize;
		}
		numBytes -= chunkSize;
	}
	return NERO_SUCCESS;
cleanup:
	return returnCode;
}

// Apply the supplied bit pattern to TMS, to move the TAP to a specific state.
//
NeroStatus neroClockFSM(
	struct NeroHandle *handle, uint32 bitPattern, uint8 transitionCount, const char **error)
{
	NeroStatus returnCode;
	const uint32 lePattern = littleEndian32(bitPattern);
	int uStatus = usb_control_msg(
		handle->device,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		CMD_JTAG_CLOCK_FSM,       // bRequest
		(uint16)transitionCount,  // wValue
		0x0000,                   // wIndex
		(char*)&lePattern,
		4,                        // wLength
		5000                      // timeout (ms)
	);
	if ( uStatus < 0 ) {
		errRender(error, "neroClockFSM(): %s (%d)", usb_strerror(), uStatus);
		FAIL(NERO_CLOCKFSM);
	}
	return NERO_SUCCESS;
cleanup:
	return returnCode;
}

// Cycle the TCK line for the given number of times.
//
NeroStatus neroClocks(struct NeroHandle *handle, uint32 numClocks, const char **error) {
	NeroStatus returnCode;
	int uStatus = usb_control_msg(
		handle->device,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		CMD_JTAG_CLOCK,    // bRequest
		numClocks&0xFFFF,  // wValue
		numClocks>>16,     // wIndex
		NULL,
		0,                 // wLength
		5000               // timeout (ms)
	);
	if ( uStatus < 0 ) {
		errRender(error, "neroClocks(): %s (%d)", usb_strerror(), uStatus);
		FAIL(NERO_CLOCKS);
	}
	return NERO_SUCCESS;
cleanup:
	return returnCode;
}

// -------------------------------------------------------------------------------------------------
// Implementation of private functions
// -------------------------------------------------------------------------------------------------

// Kick off a shift operation on the micro. This will be followed by a bunch of sends and receives.
//
static NeroStatus beginShift(
	struct NeroHandle *handle, uint32 numBits, SendType sendType, bool isLast,
	bool isResponseNeeded, const char **error)
{
	NeroStatus returnCode;
	const uint32 leNumBits = littleEndian32(numBits);
	uint16 wValue = 0x0000;
	int uStatus;
	if ( isLast ) {
		wValue |= (1<<IS_LAST);
	}
	if ( isResponseNeeded ) {
		wValue |= (1<<IS_RESPONSE_NEEDED);
	}
	wValue |= sendType << SEND_TYPE;
	uStatus = usb_control_msg(
		handle->device,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		CMD_JTAG_CLOCK_DATA,  // bRequest
		wValue,               // wValue
		0x0000,               // wIndex
		(char*)&leNumBits,      // send bit count
		4,                    // wLength
		5000                  // timeout (ms)
	);
	if ( uStatus < 0 ) {
		errRender(error, "beginShift(): %s (%d)", usb_strerror(), uStatus);
		FAIL(NERO_BEGIN_SHIFT);
	}
	return NERO_SUCCESS;
cleanup:
	return returnCode;
}

// Send a chunk of data to the micro.
//
static NeroStatus doSend(
	struct NeroHandle *handle, const uint8 *sendPtr, uint16 chunkSize, const char **error)
{
	NeroStatus returnCode;
	int uStatus = usb_bulk_write(
		handle->device,
		USB_ENDPOINT_OUT | 2,    // write to endpoint 2
		(char *)sendPtr,         // write from send buffer
		chunkSize,               // write this many bytes
		5000                     // timeout in milliseconds
	);
	if ( uStatus < 0 ) {
		errRender(error, "doSend(): %s (%d)", usb_strerror(), uStatus);
		FAIL(NERO_SEND);
	}
	return NERO_SUCCESS;
cleanup:
	return returnCode;
}

// Receive a chunk of data from the micro.
//
static NeroStatus doReceive(
	struct NeroHandle *handle, uint8 *receivePtr, uint16 chunkSize, const char **error)
{
	NeroStatus returnCode;
	int uStatus = usb_bulk_read(
		handle->device,
		USB_ENDPOINT_IN | 4,    // read from endpoint 4
		(char *)receivePtr,     // read into the receive buffer
		chunkSize,              // read this many bytes
		5000                    // timeout in milliseconds
	);
	if ( uStatus < 0 ) {
		errRender(error, "doReceive(): %s (%d)", usb_strerror(), uStatus);
		FAIL(NERO_RECEIVE);
	}
	return NERO_SUCCESS;
cleanup:
	return returnCode;
}

// Find the size of the EP2OUT & EP4IN bulk endpoints (they must be the same)
//
static NeroStatus setEndpointSize(struct NeroHandle *handle, const char **error) {
	NeroStatus returnCode;
	int uStatus;
	char descriptorBuffer[1024];  // TODO: Fix by doing two queries
	char *ptr = descriptorBuffer;
	uint8 endpointNum;
	uint16 ep2size = 0;
	uint16 ep4size = 0;
	struct usb_config_descriptor *configDesc;
	struct usb_interface_descriptor *interfaceDesc;
	struct usb_endpoint_descriptor *endpointDesc;
	handle->endpointSize = 0;
	uStatus = usb_control_msg(
		handle->device,
		USB_ENDPOINT_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
		USB_REQ_GET_DESCRIPTOR,    // bRequest
		0x0200,                    // wValue
		0x0000,     // wIndex
		descriptorBuffer,
		1024,                 // wLength
		5000               // timeout (ms)
	);
	if ( uStatus < 0 ) {
		errRender(
			error, "setEndpointSize(): Failed to get config descriptor: %s (%d)", usb_strerror(), uStatus);
		FAIL(NERO_ENDPOINTS);
	}
	if ( uStatus > 0 ) {
		configDesc = (struct usb_config_descriptor *)ptr;
		ptr += configDesc->bLength;
		interfaceDesc = (struct usb_interface_descriptor *)ptr;
		ptr += interfaceDesc->bLength;			
		endpointNum = interfaceDesc->bNumEndpoints;
		while ( endpointNum-- ) {
			endpointDesc = (struct usb_endpoint_descriptor *)ptr;
			if ( endpointDesc-> bmAttributes == 0x02 ) {
				if ( endpointDesc->bEndpointAddress == 0x02 ) {
					ep2size = littleEndian16(endpointDesc->wMaxPacketSize);
				} else if ( endpointDesc->bEndpointAddress == 0x84 ) {
					ep4size = littleEndian16(endpointDesc->wMaxPacketSize);
				}
			}
			ptr += endpointDesc->bLength;
		}
	}
	if ( !ep2size ) {
		errRender(
			error, "setEndpointSize(): EP2OUT not found or not configured as a bulk endpoint!");
		FAIL(NERO_ENDPOINTS);
	}
	if ( !ep4size ) {
		errRender(
			error, "setEndpointSize(): EP4IN not found or not configured as a bulk endpoint!");
		FAIL(NERO_ENDPOINTS);
	}
	if ( ep2size != ep4size ) {
		errRender(
			error, "setEndpointSize(): EP2OUT's wMaxPacketSize differs from that of EP4IN");
		FAIL(NERO_ENDPOINTS);
	}
	handle->endpointSize = ep2size;
	return NERO_SUCCESS;
cleanup:
	return returnCode;
}

// Put the device in jtag mode (i.e drive or tristate the JTAG lines)
//
static NeroStatus setJtagMode(struct NeroHandle *handle, bool enable, const char **error) {
	NeroStatus returnCode;
	int uStatus = usb_control_msg(
		handle->device,
		USB_ENDPOINT_OUT | USB_TYPE_VENDOR | USB_RECIP_DEVICE,
		CMD_MODE_STATUS,          // bRequest
		enable ? MODE_JTAG : 0,   // wValue
		MODE_JTAG,                // wMask
		NULL,
		0,                        // wLength
		5000                      // timeout (ms)
	);
	if ( uStatus < 0 ) {
		errRender(
			error, "setJtagMode(): Unable to %s JTAG mode: %s (%d)",
			enable ? "enable" : "disable", usb_strerror(), uStatus);
		FAIL(NERO_ENABLE);
	}
	return NERO_SUCCESS;
cleanup:
	return returnCode;
}
