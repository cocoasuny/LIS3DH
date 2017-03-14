/************************************************************************************
* This module contains various common functions like copy and compare routines.
*
* (c) Copyright 2004, Freescale, Inc.  All rights reserved.
*
* Freescale Confidential Proprietary
* Digianswer Confidential
*
* No part of this document must be reproduced in any form - including copied,
* transcribed, printed or by any electronic means - without specific written
* permission from Freescale.
*
************************************************************************************/

#include "stdint.h"
#include "TS_Kernel.h"

/************************************************************************************
*************************************************************************************
* Private macros
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private prototypes
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private type definitions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Private memory declarations
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Public functions
*************************************************************************************
************************************************************************************/

/************************************************************************************
* Copy upto 256 bytes from one buffer to another. The buffers should not overlap.
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
  )
{
    // Code looks weird, but is quite optimal on a HCS08 CPU
  while(n) {
    *((uint8_t *)pDst) = *((uint8_t *)pSrc);
    pDst=((uint8_t *)pDst)+1;
    pSrc=((uint8_t *)pSrc)+1;
    n--;
  }
}

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
  )
{
    // Code looks weird, but is quite optimal on a HCS08 CPU
  while(n) {
    *((uint8_t *)pDst) = *((uint8_t *)pSrc);
    pDst=((uint8_t *)pDst)+1;
    pSrc=((uint8_t *)pSrc)+1;
    n--;
  }
}

/************************************************************************************
* Copy upto 256 bytes from one buffer to another. The buffers should not overlap.
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
  )
{
  if(dir)
    FLib_MemCpy(pBuf1, pBuf2, n);
  else
    FLib_MemCpy(pBuf2, pBuf1, n);
}


/************************************************************************************
* Copy up to 256 bytes, possibly into the same overlapping memory as it is taken from
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
  )
{
  if (pDst != pSrc) { // Do nothing if copying to same position
    if (pDst < pSrc) { // If dst is before src in memory copy forward
        // Code looks weird, but is quite optimal on a HCS08 CPU
      while(n) {
        *((uint8_t *)pDst) = *((uint8_t *)pSrc);
        pDst=((uint8_t *)pDst)+1;
        pSrc=((uint8_t *)pSrc)+1;
        n--;
      }
    }
    else { // If dst is after src in memory copy backward
      while(n) {
        n--;
        ((uint8_t *)pDst)[n] = ((uint8_t *)pSrc)[n];
      }
    }
  }
}

/************************************************************************************
* Copy up to 256 bytes. The byte at index i from the source buffer is copied to index
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
  )
{
  pDst = (uint8_t *)pDst + (uint8_t)(n-1);
    // Code looks weird, but is quite optimal on a HCS08 CPU
  while(n) {
    *((uint8_t *)pDst) = *((uint8_t *)pSrc);
    pDst=(uint8_t *)pDst-1;
    pSrc=(uint8_t *)pSrc+1;
    n--;
  }
}


/************************************************************************************
* Compare upto 256 bytes.
*   
* Interface assumptions:
*   None
*   
* Return value:
*   TRUE if content of buffers is equal, and FALSE otherwise.
* 
************************************************************************************/
bool_t FLib_MemCmp
  (
  void *pCmp1,  // Buffer with to be compared with pCmp2
  void *pCmp2,  // Buffer with to be compared with pCmp1
  uint8_t n     // Byte count
  )
{
  while(n) {
    if( *((uint8_t *)pCmp1) != *((uint8_t *)pCmp2) )
      return FALSE;

    pCmp2=(uint8_t *)pCmp2+1;
    pCmp1=(uint8_t *)pCmp1+1;
    n--;
  }
  return TRUE;
}


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
  )
{
  return (((uint8_t *)pCmp1)[0] == ((uint8_t *)pCmp2)[0]) &&
         (((uint8_t *)pCmp1)[1] == ((uint8_t *)pCmp2)[1]);
}

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
  )
{
  while(cnt) {
    ((uint8_t *)pDst)[--cnt] = value;
  }
}

/************************************************************************************
* Reset buffer contents to a single value.
*   
************************************************************************************/
void FLib_MemSet16
  (
  void *pDst,    // Buffer to be reset
  uint8_t value, // Byte value
  uint16_t cnt    // Byte count
  )
{
  while(cnt) {
    ((uint8_t *)pDst)[--cnt] = value;
  }
}

/************************************************************************************
* Convert an 802.15.4 address mode to a length in bytes, Input values must be 0, 2,
* or 3. Other values will return either 0 or 2.
* 
************************************************************************************/
uint8_t FLib_AddrModeToLength
  (
  uint8_t addrMode // IN: 802.15.4 address mode (0, 2, or 3)
  )
{
  addrMode &= 0x03;
  if(addrMode == 3)
    return 8;
  return addrMode & 0x02;
}


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
  )
{
  (*pPtr) = ((uint8_t *)*pPtr) + offset;
}

/************************************************************************************
* Copy a number of bytes to a destination array and increment destination pointer 
* accordingly.
* 
* Interface assumptions:
*   None
*   
* Return value:
*   None
* 
************************************************************************************/
void FLib_ArrayAddArray
  (
  uint8_t **ppDestArray, //OUT: Address of destination array pointer
  uint8_t *pArray, // IN: The byte array to be added
  uint8_t length   // IN: Number of bytes to be added
  )
{
  while (length) {
    *(*ppDestArray) = *pArray;
    (*ppDestArray) = (*ppDestArray) + 1;
    pArray = pArray + 1;
    length--;
  }
}

/************************************************************************************
* Add one byte to the destination array and increment destination array pointer
* accordingly
* 
* Interface assumptions:
*   None
*   
* Return value:
*   None
* 
************************************************************************************/
void FLib_ArrayAddByte
  (
  uint8_t **ppDestArray, //OUT: Address of destination array pointer
  uint8_t byte // IN: Byte to add to the Array
  )
{
   *(*ppDestArray) = byte;
   (*ppDestArray) = (*ppDestArray) + 1;
}


/************************************************************************************
*************************************************************************************
* Private functions
*************************************************************************************
************************************************************************************/

/************************************************************************************
*************************************************************************************
* Module debug stuff
*************************************************************************************
************************************************************************************/

