#if !defined(_MODULE_MOTION)
#define _MODULE_MOTION

#include <stdio.h>

#include "sdkconfig.h"

#include "duktape.h"
#include "duktape_utils.h"
#include "dukf_utils.h"


extern void motion_module_register(duk_context *ctx);

#endif