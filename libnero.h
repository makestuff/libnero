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
#ifndef LIBNERO_H
#define LIBNERO_H

#include <makestuff.h>

#ifdef __cplusplus
extern "C" {
#endif

	// Possible return codes
	typedef enum {
		NERO_SUCCESS,
		NERO_USB_INIT,
		NERO_SYNC,
		NERO_ENDPOINTS,
		NERO_CLOCKFSM,
		NERO_CLOCKS,
		NERO_BEGIN_SHIFT,
		NERO_SEND,
		NERO_RECEIVE,
		NERO_ENABLE
	} NeroStatus;

	// Return the number of bytes necessary to store x number of bits
	#define bitsToBytes(x) ((x>>3) + (x&7 ? 1 : 0))

	// Context struct
	struct usb_dev_handle;
	struct NeroHandle {
		struct usb_dev_handle *device;
		uint16 endpointSize;
	};
	
	// Initialise the connection to the device implementing the NeroJTAG protocol
	NeroStatus neroInitialise(
		struct usb_dev_handle *device, struct NeroHandle *handle, const char **error
	) WARN_UNUSED_RESULT;
	
	// Drop the connection to the device implementing the NeroJTAG protocol
	NeroStatus neroClose(
		struct NeroHandle *handle, const char **error
	) WARN_UNUSED_RESULT;
	
	// Shift "numBits" bits from "inData" into TDI, at the same time shifting the same number of
	// bits from TDO into "outData". If "isLast" is true, leave Shift-DR state on final bit. If you
	// want inData to be all zeros or all ones, you can use ZEROS or ONES respectively. This is more
	// efficient than physically sending an array containing all zeros or all 0xFFs.
	NeroStatus neroShift(
		struct NeroHandle *handle, uint32 numBits, const uint8 *inData, uint8 *outData, bool isLast,
		const char **error
	) WARN_UNUSED_RESULT;

	// Special values for inData parameter of neroShift() declared above
	#define ZEROS (const uint8*)NULL
	#define ONES (const uint8*)-1
	
	// Clock "transitionCount" bits from "bitPattern" into TMS, starting with the LSB.
	NeroStatus neroClockFSM(
		struct NeroHandle *handle, uint32 bitPattern, uint8 transitionCount, const char **error
	) WARN_UNUSED_RESULT;
	
	// Toggle TCK "numClocks" times.
	NeroStatus neroClocks(
		struct NeroHandle *handle, uint32 numClocks, const char **error
	) WARN_UNUSED_RESULT;
	
#ifdef __cplusplus
}
#endif

#endif
