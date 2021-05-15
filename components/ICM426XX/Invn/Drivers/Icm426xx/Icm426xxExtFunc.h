/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

/** @defgroup DriverIcm426xxExt Icm426xx driver extern functions
 *  @brief    Extern functions for Icm426xx devices
 *  @ingroup  DriverIcm426xx
 *  @{
 */

/** @file Icm426xxExtFunc.h
 * Extern functions for Icm426xx devices
 */

#ifndef _INV_ICM426XX_EXTFUNC_H_
#define _INV_ICM426XX_EXTFUNC_H_


#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


/** @brief Hook for low-level high res system sleep() function to be implemented by upper layer
 *  ~100us resolution is sufficient
 *  @param[in] us number of us the calling thread should sleep
 */
extern void inv_icm426xx_sleep_us(uint32_t us);


/** @brief Hook for low-level high res system get_time() function to be implemented by upper layer
 *  Timer should be on 64bit with a 1 us resolution
 *  @param[out] The current time in us
 */
extern uint64_t inv_icm426xx_get_time_us(void);


#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM426XX_EXTFUNC_H_ */

/** @} */
