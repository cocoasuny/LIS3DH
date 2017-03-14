/************************************************************************************
* This header file is provided as part of the interface to the Freescale 802.15.4
* MAC and PHY layer.
*
* The file gives access to the generic function library used by the MAC/PHY.
*
* (c) Copyright 2007, Freescale, Inc.  All rights reserved.
*
* No part of this document must be reproduced in any form - including copied,
* transcribed, printed or by any electronic means - without specific written
* permission from Freescale Semiconductor.
*
************************************************************************************/

#ifndef _FUNCTION_LIB_H_
#define _FUNCTION_LIB_H_

#include "stdint.h"
#include "TS_Kernel.h"

/************************************************************************************
*************************************************************************************
* Public macros
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/************************************************************************************
* Copy upto 255 bytes from one buffer to another. The buffers should not overlap.
*   
* Interface assumptions:
*   None
*   
* Return value:
*   None
* 
************************************************************************************/
void FLib_MemCpy
  (
  void *pDst, // Destination buffer
  void *pSrc, // Source buffer
  uint8_t n   // Byte count
  );

/************************************************************************************
* Copy upto 65535 bytes from one buffer to another. The buffers should not overlap.
*   
* Interface assumptions:
*   None
*   
* Return value:
*   None
* 
************************************************************************************/
void FLib_MemCpy16
  (
  void *pDst, // Destination buffer
  void *pSrc, // Source buffer
  uint16_t n   // Byte count
  );


/************************************************************************************
* Copy 2 bytes from one buffer to another. The buffers should not overlap.
*   
* Interface assumptions:
*   None
*   
* Return value:
*   None
* 
************************************************************************************/
void FLib_MemCpy2Bytes
  (
  void *pDst, // Destination buffer
  void *pSrc  // Source buffer
  );

/************************************************************************************
* Copy upto 255 bytes from one buffer to another. The buffers should not overlap.
* The function can copy in either direction. If 'dir' is TRUE, then the function
* works like FLib_MemCpy(). If FALSE, the function swaps the buffer pointers
* before copying.
*   
* Interface assumptions:
*   None
*   
* Return value:
*   None
* 
************************************************************************************/
void FLib_MemCpyDir
  (
  void *pBuf1, // Dst/Src buffer
  void *pBuf2, // Src/Dst buffer
  bool_t dir,  // Direction: TRUE: pBuf1<-pBuf2, FALSE: pBuf2<-pBuf1
  uint8_t n    // Byte count
  );

/************************************************************************************
* Copy up to 255 bytes, possibly into the same overlapping memory as it is taken from
*   
* Interface assumptions:
*   None
*   
* Return value:
*   None
* 
************************************************************************************/
void FLib_MemInPlaceCpy
  (
  void *pDst, // Destination buffer
  void *pSrc, // Source buffer
  uint8_t n   // Byte count
  );

/************************************************************************************
* Copy up to 255 bytes. The byte at index i from the source buffer is copied to index
* ((n-1) - i) in the destination buffer (and vice versa).
*   
* Interface assumptions:
*   None
*   
* Return value:
*   None
* 
************************************************************************************/
void FLib_MemCpyReverseOrder
  (
  void *pDst, // Destination buffer
  void *pSrc, // Source buffer
  uint8_t n   // Byte count
  );

/************************************************************************************
* Compare upto 255 bytes.
*   
* Interface assumptions:
*   None
*   
* Return value:
*   TRUE if content of buffers is equal, and FALSE otherwise.
* 
************************************************************************************/
uint8_t FLib_MemCmp
  (
  void *pCmp1,  // Buffer with to be compared with pCmp2
  void *pCmp2,  // Buffer with to be compared with pCmp1
  uint8_t n     // Byte count
  );

/************************************************************************************
* Compare two bytes.
*   
* Interface assumptions:
*   None
*   
* Return value:
*   TRUE if content of buffers is equal, and FALSE otherwise.
* 
************************************************************************************/
bool_t FLib_Cmp2Bytes
  (
  void *pCmp1, // Buffer with to be compared with pCmp2
  void *pCmp2  // Buffer with to be compared with pCmp1
  );

/************************************************************************************
* Reset buffer contents to a single value.
*   
* Interface assumptions:
*   None
*   
* Return value:
*   None.
* 
************************************************************************************/
void FLib_MemSet
  (
  void *pDst,    // Buffer to be reset
  uint8_t value, // Byte value
  uint8_t cnt    // Byte count
  );

/************************************************************************************
* Reset buffer contents to a single value. 16bit buffer length.
*   
* Interface assumptions:
*   None
*   
* Return value:
*   None.
* 
************************************************************************************/
void FLib_MemSet16
  (
  void *pDst,    // Buffer to be reset
  uint8_t value, // Byte value
  uint16_t cnt    // Byte count
  );

/************************************************************************************
* Add an offset to a pointer. The offset can be in the range 0-255.
* 
* Interface assumptions:
*   None
*   
* Return value:
*    None
* 
************************************************************************************/
void FLib_AddOffsetToPointer
  (
  void **pPtr,    // IN/OUT: Pointer to the pointer to add the offset to
  uint8_t offset  // IN: Offset to add to the supplied pointe´r
  );
#define FLib_AddOffsetToPtr(pPtr,offset) FLib_AddOffsetToPointer((void**)pPtr,offset)

/************************************************************************************
* Returns the maximum value of arguments a and b.
* 
* Interface assumptions:
*   The primitive should must be implemented as a macro, as it should be possible to
*   evaluate the result on compile time if the arguments are constants.
*   
* Return value:
*   The maximum value of arguments a and b
* 
************************************************************************************/
#define FLib_GetMax(a,b)    (((a) > (b)) ? (a) : (b))

/************************************************************************************
* Returns the minimum value of arguments a and b.
* 
* Interface assumptions:
*   The primitive should must be implemented as a macro, as it should be possible to
*   evaluate the result on compile time if the arguments are constants.
*   
* Return value:
*   The minimum value of arguments a and b
* 
************************************************************************************/
#define FLib_GetMin(a,b)    (((a) < (b)) ? (a) : (b))

#endif /* _FUNCTION_LIB_H_ */
