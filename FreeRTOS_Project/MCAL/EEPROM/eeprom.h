/*
 * adc.h
 *
 *  Created on: Jun 24, 2024
 *      Author: mosta
 */

#ifndef MCAL_ERPROM_ERPROM_H_
#define MCAL_ERPROM_ERPROM_H_

#include "utilities.h"
#include "tm4c123gh6pm_registers.h"

//*****************************************************************************
//
// Values returned by EEPROMInit.
//
//*****************************************************************************

//
//! This value may be returned from a call to EEPROMInit().  It indicates that
//! no previous write operations were interrupted by a reset event and that the
//! EEPROM peripheral is ready for use.
//
#define EEPROM_INIT_OK      0

//
//! This value may be returned from a call to EEPROMInit().  It indicates that
//! a previous data or protection write operation was interrupted by a reset
//! event and that the EEPROM peripheral was unable to clean up after the
//! problem.  This situation may be resolved with another reset or may be fatal
//! depending upon the cause of the problem.  For example, if the voltage to
//! the part is unstable, retrying once the voltage has stabilized may clear
//! the error.
//
#define EEPROM_INIT_ERROR   2

//*****************************************************************************
//
// Error indicators returned by various EEPROM API calls.  These will be ORed
// together into the final return code.
//
//*****************************************************************************

//
//! This return code bit indicates that an attempt was made to read from
//! the EEPROM while a write operation was in progress.
//
#define EEPROM_RC_WRBUSY            0x00000020

//
//! This return code bit indicates that an attempt was made to write a
//! value but the destination permissions disallow write operations.  This
//! may be due to the destination block being locked, access protection set
//! to prohibit writes or an attempt to write a password when one is already
//! written.
//
#define EEPROM_RC_NOPERM            0x00000010

//
//! This return code bit indicates that the EEPROM programming state machine
//! is currently copying to or from the internal copy buffer to make room for
//! a newly written value.  It is provided as a status indicator and does not
//! indicate an error.
//
#define EEPROM_RC_WKCOPY            0x00000008

//
//! This return code bit indicates that the EEPROM programming state machine
//! is currently erasing the internal copy buffer.  It is provided as a
//! status indicator and does not indicate an error.
//
#define EEPROM_RC_WKERASE           0x00000004

//
//! This return code bit indicates that the EEPROM programming state machine
//! is currently working.  No new write operations should be attempted until
//! this bit is clear.
//
#define EEPROM_RC_WORKING           0x00000001

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEDONE register.
//
//*****************************************************************************
#define EEPROM_EEDONE_WORKING   0x00000001  // EEPROM Working
#define EEPROM_EEDONE_WRBUSY    0x00000020  // Write Busy
#define EEPROM_EEDONE_NOPERM    0x00000010  // Write Without Permission

//*****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EESUPP register.
//
//*****************************************************************************
#define EEPROM_EESUPP_ERETRY    0x00000004  // Erase Must Be Retried
#define EEPROM_EESUPP_PRETRY    0x00000008  // Programming Must Be Retried

//*****************************************************************************
//
// Useful macro to extract the offset from a linear address.
//
//*****************************************************************************
#define OFFSET_FROM_ADDR(x) (((x) >> 2) & 0x0F)

//*****************************************************************************
//
//! Returns the EEPROM block number containing a given offset address.
//!
//! \param ui32Addr is the linear, byte address of the EEPROM location whose
//! block number is to be returned.  This is a zero-based offset from the start
//! of the EEPROM storage.
//!
//! This macro may be used to translate an EEPROM address offset into a
//! block number suitable for use in any of the driver's block protection
//! functions.  The address provided is expressed as a byte offset from the
//! base of the EEPROM.
//!
//! \return Returns the zero-based block number which contains the passed
//! address.
//
//*****************************************************************************
#define EEPROMBlockFromAddr(ui32Addr) ((ui32Addr) >> 6)


//*****************************************************************************
//
// The following are defines for the EEPROM register offsets.
//
//*****************************************************************************
#define EEPROM_EESIZE                       0x400AF000  // EEPROM Size Information
#define EEPROM_EEBLOCK                      0x400AF004  // EEPROM Current Block
#define EEPROM_EEOFFSET                     0x400AF008  // EEPROM Current Offset
#define EEPROM_EERDWR                       0x400AF010  // EEPROM Read-Write
#define EEPROM_EERDWRINC                    0x400AF014  // EEPROM Read-Write with Increment
#define EEPROM_EEDONE                       0x400AF018  // EEPROM Done Status
#define EEPROM_EESUPP                       0x400AF01C  // EEPROM Support Control and
                                            // Status
#define EEPROM_EEUNLOCK                     0x400AF020  // EEPROM Unlock
#define EEPROM_EEPROT                       0x400AF030  // EEPROM Protection
#define EEPROM_EEPASS0                      0x400AF034  // EEPROM Password
#define EEPROM_EEPASS1                      0x400AF038  // EEPROM Password
#define EEPROM_EEPASS2                      0x400AF03C  // EEPROM Password
#define EEPROM_EEINT                        0x400AF040  // EEPROM Interrupt
#define EEPROM_EEHIDE0                      0x400AF050  // EEPROM Block Hide 0
#define EEPROM_EEHIDE                       0x400AF050  // EEPROM Block Hide
#define EEPROM_EEHIDE1                      0x400AF054  // EEPROM Block Hide 1
#define EEPROM_EEHIDE2                      0x400AF058  // EEPROM Block Hide 2
#define EEPROM_EEDBGME                      0x400AF080  // EEPROM Debug Mass Erase
#define EEPROM_PP                           0x400AFFC0  // EEPROM Peripheral Properties

#define SYSCTL_PERIPH_EEPROM0               0x400FE558


/************************************
 * Extern Variables
 ************************************/

extern char dr_hl_c1;
extern char dr_hl_c2;
extern char dr_hl_c3;
extern char dr_hl_c4;
extern char dr_err_c1;
extern char dr_err_c2;
extern char dr_err_c3;
extern char dr_err_c4;

extern char ps_hl_c1;
extern char ps_hl_c2;
extern char ps_hl_c3;
extern char ps_hl_c4;
extern char ps_err_c1;
extern char ps_err_c2;
extern char ps_err_c3;
extern char ps_err_c4;

/************************************
 * Functions Declarations
 ************************************/
uint32 EEPROMInit(void);
void EEPROMRead(uint32 *pui32Data, uint32 ui32Address, uint32 ui32Count);
uint32 EEPROMProgram(uint32 *pui32Data, uint32 ui32Address, uint32 ui32Count);
void time_split_write(uint32 time,uint32 *array_ptr);
uint32 time_concate_read(uint32 *array_ptr);
void string_split_write( uint8 *levelptr,uint8 *errorptr, uint32 *array_ptr);
void string_concate_read(uint32 *array_ptr1,uint32 *array_ptr2);




#endif /* MCAL_ERPROM_ERPROM_H_ */
