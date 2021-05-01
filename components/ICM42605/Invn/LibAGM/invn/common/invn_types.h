//----------------------------------------------------------------------------- 
/*
    Copyright © 2014-2015 InvenSense Inc. Portions Copyright © 2014-2015 Movea. All rights reserved.

    This software, related documentation and any modifications thereto (collectively “Software”) is subject
    to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
    and other intellectual property rights laws.

    InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
    and any use, reproduction, disclosure or distribution of the Software without an express license agreement
    from InvenSense is strictly prohibited.
*/
//-----------------------------------------------------------------------------

#ifndef INVN_COMMON_INVN_TYPES_H_
#define INVN_COMMON_INVN_TYPES_H_

/**
 *  @defgroup invn_types Types
 *  @brief  Motion Library - Type definitions.
 *  \details Definition of codes and error codes used within the MPL and
 *          returned to the user.
 *          Every function tries to return a meaningful error code basing
 *          on the occuring error condition. The error code is numeric.
 *
 *          The available error codes and their associated values are:
 *          - (0)               INV_SUCCESS
 *          - (32)              INV_ERROR
 *          - (22 / EINVAL)     INV_ERROR_INVALID_PARAMETER
 *          - (1  / EPERM)      INV_ERROR_FEATURE_NOT_ENABLED
 *          - (36)              INV_ERROR_FEATURE_NOT_IMPLEMENTED
 *          - (64)              INV_ERROR_FIFO_READ_COUNT
 * \todo Clean up the details documentation in order to use only the \\def one.
 * \todo Add documentation to all the definitions
 * \ingroup Common
 * @file invn_types.h
 */

//=======================================//
//========= Integer Definition  =========//
//=======================================// 
#ifdef _MSC_VER
#  include "invn/common/msvc/inttypes.h"
#else
#  include <stdint.h>
#endif

//=======================================//
//======= Fixed Point Conversion  =======//
//=======================================// 

//! \def INVN_FLT_TO_FXP
//! Convert the \a value from float to QN value. \ingroup invn_macro 
#define INVN_FLT_TO_FXP(value, shift)	( (int32_t)  ((float)(value)*(1ULL << (shift)) + ( (value>=0)-0.5f )) ) 
//! \def INVN_DBL_TO_FXP
//! Convert the \a value from double to QN value. \ingroup invn_macro 
#define INVN_DBL_TO_FXP(value, shift)	( (int32_t)  ((double)(value)*(1ULL << (shift)) + ( (value>=0)-0.5 )) ) 
//! \def INVN_FLT_TO_UFXP
//! Convert the \a value from float to unsigned QN value. \ingroup invn_macro 
#define INVN_FLT_TO_UFXP(value, shift)	( (uint32_t) ((float)(value)*(1ULL << (shift)) + 0.5f) ) 
//! \def INVN_DBL_TO_UFXP
//! Convert the \a value from double to unsigned QN value. \ingroup invn_macro 
#define INVN_DBL_TO_UFXP(value, shift)	( (uint32_t) ((double)(value)*(1ULL << (shift)) + 0.5) ) 
//! \def INVN_FXP_TO_FLT
//! Convert the \a value from QN value to float. \ingroup invn_macro 
#define INVN_FXP_TO_FLT(value, shift)	( (float)  (int32_t)(value) / (float)(1ULL << (shift)) ) 
//! \def INVN_FXP_TO_DBL
//! Convert the \a value from QN value to double. \ingroup invn_macro 
#define INVN_FXP_TO_DBL(value, shift)	( (double)  (int32_t)(value) / (double)(1ULL << (shift)) ) 
//! \def INVN_UFXP_TO_FLT
//! Convert the \a value from unsigned QN value to float. \ingroup invn_macro
#define INVN_UFXP_TO_FLT(value, shift)	( (float) (uint32_t)(value) / (float)(1ULL << (shift)) ) 
//! \def INVN_UFXP_TO_DBL
//! Convert the \a value from unsigned QN value to double. \ingroup invn_macro 
#define INVN_UFXP_TO_DBL(value, shift)	( (double) (uint32_t)(value) / (double)(1ULL << (shift)) ) 
//! \def INVN_CONVERT_FLT_TO_FXP
//!	Macro to convert float values from an address into QN values, and copy them to another address. \ingroup invn_macro
#define INVN_CONVERT_FLT_TO_FXP(fltptr, fixptr, length, shift)	{ int32_t i; for(i=0; i<(length); ++i) (fixptr)[i] = INVN_FLT_TO_FXP((fltptr)[i], shift); }
//! \def INVN_CONVERT_FLT_TO_UFXP
//!	Macro to convert float values from an address into unsigned QN values, and copy them to another address. \ingroup invn_macro
#define INVN_CONVERT_FLT_TO_UFXP(fltptr, fixptr, length, shift)	{ int32_t i; for(i=0; i<(length); ++i) (fixptr)[i] = INVN_FLT_TO_UFXP((fltptr)[i], shift); }
//! \def INVN_CONVERT_DBL_TO_FXP
//!	Macro to convert double values from an address into QN  values, and copy them to another address. \ingroup invn_macro
#define INVN_CONVERT_DBL_TO_FXP(fltptr, fixptr, length, shift)	{ int32_t i; for(i=0; i<(length); ++i) (fixptr)[i] = INVN_DBL_TO_FXP((fltptr)[i], shift); }
//! \def INVN_CONVERT_DBL_TO_UFXP
//!	Macro to convert double values from an address into unsigned QN values, and copy them to another address. \ingroup invn_macro
#define INVN_CONVERT_DBL_TO_UFXP(fltptr, fixptr, length, shift)	{ int32_t i; for(i=0; i<(length); ++i) (fixptr)[i] = INVN_DBL_TO_UFXP((fltptr)[i], shift); }
//! \def INVN_CONVERT_FXP_TO_FLT
//!	Macro to convert QN values from an address into float values, and copy them to another address. \ingroup invn_macro
#define INVN_CONVERT_FXP_TO_FLT(fixptr, fltptr, length, shift)	{ int32_t i; for(i=0; i<(length); ++i) (fltptr)[i] = INVN_FXP_TO_FLT((fixptr)[i], shift); }
//! \def INVN_CONVERT_UFXP_TO_FLT
//!	Macro to convert unsigned QN values from an address into float values, and copy them to another address. \ingroup invn_macro
#define INVN_CONVERT_UFXP_TO_FLT(fixptr, fltptr, length, shift)	{ int32_t i; for(i=0; i<(length); ++i) (fltptr)[i] = INVN_UFXP_TO_FLT((fixptr)[i], shift); }
//! \def INVN_CONVERT_FXP_TO_DBL
//!	Macro to convert QN values from an address into double values, and copy them to another address. \ingroup invn_macro
#define INVN_CONVERT_FXP_TO_DBL(fixptr, fltptr, length, shift)	{ int32_t i; for(i=0; i<(length); ++i) (fltptr)[i] = INVN_FXP_TO_DBL((fixptr)[i], shift); }
//! \def INVN_CONVERT_UFXP_TO_DBL 
//! \brief Macro to convert unsigned QN values from an address into double values, and copy them to another address. \ingroup invn_macro
#define INVN_CONVERT_UFXP_TO_DBL(fixptr, fltptr, length, shift)	{ int32_t i; for(i=0; i<(length); ++i) (fltptr)[i] = INVN_UFXP_TO_DBL((fixptr)[i], shift); }


//=====================================//
//========= Error Definition  =========//
//=====================================// 

#ifndef REMOVE_INV_ERROR_T
	typedef int32_t inv_error_t;	/*!< Type used for error definitions. \ingroup invn_types */
#endif

//typedef int32_t mpu_error_t;
typedef int64_t mpu_time_t;			/*!< Type used for mpu time. \ingroup invn_types */

// Typically I2C addresses are 8-bit, but some specifications allow for a 10-bit address
// This definition allows the length to be optimally defined for the platform
typedef uint8_t inv_i2c_addr_t;		/*!< Type used for I2C adresses. \ingroup invn_types */

#ifdef __IAR_SYSTEMS_ICC__
	// These are defined in standard C errno.h
	#define EINVAL                                  (22)
	#define EPERM                                   (1)
	#define ENOMEM                                  (12)
#else
	#include "errno.h"
#endif

#define INVN_SUCCESS							(0) 					/*!< Constant definition for success. \ingroup invn_types */
#define INVN_ERROR_BASE							(0x20)					/*!< Constant definition for basic error. Value is \b 32 \ingroup invn_types */
#define INVN_ERROR								(INVN_ERROR_BASE)		/*!< Constant definition for error. Value is \b 32 \ingroup invn_types */
#define INVN_ERROR_FEATURE_NOT_ENABLED          (EPERM)					/*!< Constant definition for feature not enabled error. \ingroup invn_types */
#define INVN_ERROR_FEATURE_NOT_IMPLEMENTED      (INVN_ERROR_BASE + 4)	/*!< Constant definition for feature not implemented error. \ingroup invn_types */
#define INVN_ERROR_INVALID_PARAMETER            (EINVAL)				/*!< Constant definition for invalid parameter error. \ingroup invn_types */
#define INVN_ERROR_FILE_OPEN                    (INVN_ERROR_BASE + 14)	/*!< Constant definition for opening file error. \ingroup invn_types */
#define INVN_ERROR_FILE_READ                    (INVN_ERROR_BASE + 15)	/*!< Constant definition for reading file error. \ingroup invn_types */
#define INVN_ERROR_FILE_WRITE                   (INVN_ERROR_BASE + 16)	/*!< Constant definition for writing file error. \ingroup invn_types */
#define INVN_ERROR_INVALID_CONFIGURATION        (INVN_ERROR_BASE + 17)	/*!< Constant definition for invalid configuration error. \ingroup invn_types */
/* Serial Communication */
#define INVN_ERROR_SERIAL_OPEN_ERROR            (INVN_ERROR_BASE + 21)	/*!< Constant definition for serial open error. \ingroup invn_types */
#define INVN_ERROR_SERIAL_READ                  (INVN_ERROR_BASE + 22)	/*!< Constant definition for serial read error. \ingroup invn_types */
#define INVN_ERROR_SERIAL_WRITE                 (INVN_ERROR_BASE + 23)	/*!< Constant definition for serial write error. \ingroup invn_types */
/* Fifo */
#define INVN_ERROR_FIFO_OVERFLOW                (INVN_ERROR_BASE + 30)	/*!< Constant definition for fifo overflow error. \ingroup invn_types */
#define INVN_ERROR_FIFO_FOOTER                  (INVN_ERROR_BASE + 31)	/*!< Constant definition for fifo footer error. \ingroup invn_types */
#define INVN_ERROR_FIFO_READ_COUNT              (INVN_ERROR_BASE + 32)	/*!< Constant definition for fifo read count error. \ingroup invn_types */
#define INVN_ERROR_FIFO_READ_DATA               (INVN_ERROR_BASE + 33)	/*!< Constant definition for fifo read data error. \ingroup invn_types */
/* OS interface errors */
#define INVN_ERROR_OS_BAD_HANDLE                (INVN_ERROR_BASE + 61)	/*!< Constant definition for OS bad handle error. \ingroup invn_types */
#define INVN_ERROR_OS_CREATE_FAILED             (INVN_ERROR_BASE + 62)	/*!< Constant definition for OS create failed error. \ingroup invn_types */
#define INVN_ERROR_OS_LOCK_FAILED               (INVN_ERROR_BASE + 63)	/*!< Constant definition for OS lock failed error. \ingroup invn_types */
/* Warning */
#define INVN_WARNING_SEMAPHORE_TIMEOUT          (INVN_ERROR_BASE + 86)	/*!< Constant definition for semaphore timeout warning. \ingroup invn_types */


#endif // INVN_COMMON_INVN_TYPES_H_
