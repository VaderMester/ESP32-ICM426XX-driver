#if !defined(_MODULE_CUBEOS)
#define _MODULE_CUBEOS

#include <stdio.h>

#include "sdkconfig.h"

#include "duktape.h"
#include "duktape_utils.h"
#include "dukf_utils.h"

/**
 * Attach the debugger.
 */
//static duk_ret_t js_dukf_debug(duk_context *ctx);

//Ask JS to perform a gabrage collection.
//static duk_ret_t js_dukf_gc(duk_context *ctx);

// Return the global object.
//static duk_ret_t js_dukf_global(duk_context *ctx);


/*
 * Log the heap size with a tag.
 * [0] - tag
 */
//static duk_ret_t js_dukf_logHeap(duk_context *ctx);


/*
 * Set the named file as the start file.  In our logic, after initialization, we
 * check the value of the NVS esp32duktape->start for existence and for a string
 * file name.  If this key has a value then we try and run that script.
 *
 * [0] - string - fileName to try and run at startup.
 */

//static duk_ret_t js_dukf_setStartFile(duk_context *ctx);

//static duk_ret_t js_dukf_sleep(duk_context *ctx);

extern void cubeos_module_register(duk_context *ctx);

/*
const char *cubeosHello =
   "\n  ______             __\n"
    " /      \\           |  \\\n"           
    "|  $$$$$$\\ __    __ | $$____    ______\n"
    "| $$   \\$$|  \\  |  \\| $$    \\  /      \\\n"
    "| $$      | $$  | $$| $$$$$$$\\|  $$$$$$\\\n"
    "| $$   __ | $$  | $$| $$  | $$| $$    $$\n"
    "| $$__/  \\| $$__/ $$| $$__/ $$| $$$$$$$$\n"
    " \\$$    $$ \\$$    $$| $$    $$ \\$$     \\\n"
    "  \\$$$$$$   \\$$$$$$  \\$$$$$$$   \\$$$$$$$\n"
    "___________________________\n"  
    "|\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\n"
    "||\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\n"
    "|||\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\n"
    "|||+--------------------------+\n"
    "||||oooooooooooooooooooooooooo|\n"
    "||||oooooooooooooooooooooooooo|\n"
    "||||ooo ______    _______  ooo|\n"
    "||||o  /      \\  /       \\ ooo|\n"
    "||||o |  $$$$$$\\|  $$$$$$$  oo|\n"
    "||||o | $$  | $$ \\$$    \\  ooo|\n"
    "||||o | $$__/ $$ _\\$$$$$$\\ ooo|\n"
    "||||oo \\$$    $$|       $$  oo|\n"
    "||||ooo \\$$$$$$  \\$$$$$$$  ooo|\n"
    "\\|||oooo       V0.1       oooo|\n"
    "_\\||oooooooooooooooooooooooooo|\n"
    "__\\|oooooooooooooooooooooooooo|\n"
    "___+--------------------------+\n"             
    "Based on Duktape JavaScript Engine\n"
    "http://duktape.org\n"
    "\n"
    "Developed by Bence, Bujtor\n"
    "https://github.com/VaderMester\n"
    "\n"
    "\n"
    "Build: " __DATE__ " " __TIME__ "\n"
    "\n";
*/

#endif