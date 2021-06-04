#ifndef __MALLOC_LOCAL_H__
#define __MALLOC_LOCAL_H__

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "esp_heap_caps.h"

/**
 * Edit these functions if you want to change malloc of the component you include this into
 * 
 */

#define malloc_local(size) heap_caps_malloc(size, MALLOC_CAP_8BIT)
#define realloc_local(ptr, size) heap_caps_realloc(ptr, size, MALLOC_CAP_8BIT)
#define free_local(ptr) heap_caps_free(ptr)

#endif //__MALLOC_LOCAL_H__