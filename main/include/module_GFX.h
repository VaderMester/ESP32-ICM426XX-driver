#if !defined(_MODULE_GFX)
#define _MODULE_GFX

#include <stdio.h>
#include "sdkconfig.h"
#include "duktape.h"
#include "duktape_utils.h"
#include "dukf_utils.h"
//#include <gfxfont.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cubeGFX.h>
//-------------- BASIC C funcs ---------------------

void module_gfx_putFontToList(uint index, GFXfont *font);
void gfx_module_register(duk_context *ctx);
void gfx_module_init(void);

#endif