#include <stdio.h>
#include "esp_log.h"
#include "sdkconfig.h"

#include "duktape.h"
#include "dukf_utils.h"

//#include <btterminal.h>

#include <cubeGFX.h>
#include "module_GFX.h"
#include <fonts.h>

#define LOGNAME "cubeOS"
#define ERROR(fmt, ...)   ESP_LOGE(LOGNAME, fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...)   ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...)   ESP_LOGV(LOGNAME, fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)   ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)

#define MS2TICKS(ms)              ((ms) / portTICK_PERIOD_MS)
#define osSleep(ms)               vTaskDelay(MS2TICKS(ms));

// ------------------ CONTROL API --------------------
//void cube_gfx_setRotation(uint8_t r);
//void cube_gfx_invertDisplay(bool i);

//esp_err_t cube_gfx_startWrite(void);
static duk_ret_t gfx_start(duk_context *ctx);
static duk_ret_t gfx_end(duk_context *ctx);
static duk_ret_t gfx_setFrameTime(duk_context *ctx);
static duk_ret_t gfx_getFrameTime(duk_context *ctx);
static duk_ret_t gfx_pauseRender(duk_context *ctx);
static duk_ret_t gfx_resumeRender(duk_context *ctx);

// ------------------END OF CONTROL API --------------------

// ------------------- BASIC DRAW API ----------------------
static duk_ret_t gfx_colorMake(duk_context *ctx);
static duk_ret_t gfx_drawPixel(duk_context *ctx);
static duk_ret_t gfx_getPixel(duk_context *ctx);
static duk_ret_t gfx_drawRect(duk_context *ctx);
static duk_ret_t gfx_fillRect(duk_context *ctx);
static duk_ret_t gfx_drawCircle(duk_context *ctx);
//static duk_ret_t gfx_drawCircleHelper(duk_context *ctx);
static duk_ret_t gfx_fillCircle(duk_context *ctx);
//static duk_ret_t gfx_fillCircleHelper(duk_context *ctx);
static duk_ret_t gfx_drawTriangle(duk_context *ctx);
static duk_ret_t gfx_fillTriangle(duk_context *ctx);
static duk_ret_t gfx_drawRoundRect(duk_context *ctx);
static duk_ret_t gfx_fillRoundRect(duk_context *ctx);
static duk_ret_t gfx_drawFastVLine(duk_context *ctx);
static duk_ret_t gfx_drawFastHLine(duk_context *ctx);
static duk_ret_t gfx_drawLine(duk_context *ctx);
static duk_ret_t gfx_fillScreen(duk_context *ctx);

// ---------------- END OF BASIC DRAW API -------------------




// ------------------- BITMAP API ----------------------
// TODO
/*
#if 0
//void cube_gfx_drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);
//void cube_gfx_drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color, uint16_t bg);
void cube_gfx_drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
//void cube_gfx_drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg);
void cube_gfx_drawXBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);
//void cube_gfx_drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h);
void cube_gfx_drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h);
//void cube_gfx_drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[], const uint8_t mask[], int16_t w, int16_t h);
void cube_gfx_drawGrayscaleBitmap_opaque(int16_t x, int16_t y, uint8_t *bitmap, uint8_t *mask, int16_t w, int16_t h);
//void cube_gfx_drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], int16_t w, int16_t h);
void cube_gfx_drawRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h);
//void cube_gfx_drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], const uint8_t mask[], int16_t w, int16_t h);
void cube_gfx_drawRGBBitmap_opaque(int16_t x, int16_t y, uint16_t *bitmap, uint8_t *mask, int16_t w, int16_t h);
#endif
*/
// ------------------- END OF BITMAP API ----------------------

// ------------------- TEXT API ----------------------
//void writeChar(int16_t x, int16_t y, unsigned char c, uint16_t color, int16_t bg, uint8_t size_x, uint8_t size_y);

//void cube_gfx_ClearAll(void);
static duk_ret_t gfx_clearAll(duk_context *ctx);
//void cube_gfx_ClearBuf(void);
static duk_ret_t gfx_clearBuf(duk_context *ctx);



//static duk_ret_t gfx_drawCharBg(duk_context *ctx);
//static duk_ret_t gfx_drawChar(duk_context *ctx);

static duk_ret_t gfx_getTextBounds(duk_context *ctx);
static duk_ret_t gfx_setTextSize(duk_context *ctx);
static duk_ret_t gfx_setFont(duk_context *ctx);
static duk_ret_t gfx_setCursor(duk_context *ctx);

static duk_ret_t gfx_setTextColor(duk_context *ctx);
static duk_ret_t gfx_setTextColorBg(duk_context *ctx);
static duk_ret_t gfx_setTextWrap(duk_context *ctx);
static duk_ret_t gfx_printText(duk_context *ctx);
static duk_ret_t gfx_getCursor(duk_context *ctx);


//int16_t cube_gfx_width(void);
//int16_t cube_gfx_height(void);
//uint8_t cube_gfx_getRotation(void);

static TaskHandle_t initTask;

static GFXfont *fontlist[10];

/*
	To be used in CubeOS envt. to assign a font value to a created font list.
	A font array must have been created beforehand.
*/
/*void module_gfx_setFontInList(GFXfont *listitem, const GFXfont *font) {
	listitem = font;
}
*/
void module_gfx_putFontToList(uint index, GFXfont *font) {
	fontlist[index] = font;
}

void gfx_module_init(void)
{
	module_gfx_putFontToList(0, &kongtext4pt7b);
	module_gfx_putFontToList(1, &FreeMonoBold9pt7b);
	xTaskCreate(sGFXinitTask, "gfx_init", 8000, NULL, 5, &initTask);
}

static duk_function_list_entry gfx_funcs[] = {
	{"start",       	gfx_start,      	0},
	{"end",      		gfx_end,     		0},
	{"setFrameTime",    gfx_setFrameTime,   1},
	{"getFrameTime",    gfx_getFrameTime,   0},
	{"clearAll",    	gfx_clearAll,   	0},
	{"clearBuf",    	gfx_clearBuf,   	0},
	{"pauseRender",    	gfx_pauseRender,   	0},
	{"resumeRender",  	gfx_resumeRender,   0},
	{"colorMake",		gfx_colorMake,		4},
	{"drawPixel",		gfx_drawPixel,		3},
	{"getPixel",		gfx_getPixel,		2},
	{"drawRect",		gfx_drawRect,		5},
	{"fillRect",		gfx_fillRect,		5},
	{"drawCircle",		gfx_drawCircle,		4},
	{"fillCircle",		gfx_fillCircle,		4},
	{"drawTriangle",	gfx_drawTriangle,	7},
	{"fillTriangle",	gfx_fillTriangle,	7},
	{"drawRoundRect",	gfx_drawRoundRect,	6},
	{"fillRoundRect",	gfx_fillRoundRect,	6},
	{"drawVLine",		gfx_drawFastVLine,	4},
	{"drawHLine",		gfx_drawFastHLine,	4},
	{"drawLine",		gfx_drawLine,		5},
	{"fillScreen",		gfx_fillScreen,		1},
	{"getTextBounds",	gfx_getTextBounds,	3},
	{"setTextSize",		gfx_setTextSize,	1},
	{"setFont",			gfx_setFont,		1},
	{"setCursor",		gfx_setCursor,		2},
	{"getCursor",		gfx_getCursor,		0},
	{"setTextColor",	gfx_setTextColor,	1},
	{"setTextColorBg",	gfx_setTextColorBg,	2},
	{"setTextWrap",		gfx_setTextWrap,	1},
	{"printText",		gfx_printText,		4},
	{ NULL, NULL, 0 }
};

void gfx_module_register(duk_context *ctx) {
	/* Set global 'cubeOS'. */
	duk_push_global_object(ctx);
	duk_push_object(ctx);
	duk_put_function_list(ctx, -1, gfx_funcs);
	duk_put_prop_string(ctx, -2, "GFX");
	duk_pop(ctx);
}

// ------------------ CONTROL API --------------------
//void cube_gfx_setRotation(uint8_t r);
//void cube_gfx_invertDisplay(bool i);
//esp_err_t cube_gfx_startWrite(void);

static duk_ret_t gfx_start(duk_context *ctx) {
	if(cube_gfx_startWrite() != ESP_OK)
	{
		duk_push_error_object(ctx, DUK_ERR_TYPE_ERROR, "Failed to Start Display writing sequence");
    	return duk_throw(ctx);
	}
	return 0;
}

//void cube_gfx_endWrite(void);
static duk_ret_t gfx_end(duk_context *ctx)
{
	cube_gfx_endWrite();
	return 0;
}

/*void cube_gfx_setFrameTime(unsigned int millisec);
nArgs: 1
0: millisec
*/
static duk_ret_t gfx_setFrameTime(duk_context *ctx)
{
	uint ms = duk_get_uint(ctx, 0);
	if( ms != 0) {
		cube_gfx_setFrameTime(ms);
	}
	return 0;
}

static duk_ret_t gfx_pauseRender(duk_context *ctx)
{	
	cube_gfx_pauseRender();
	return 0;
}

static duk_ret_t gfx_resumeRender(duk_context *ctx)
{
	cube_gfx_resumeRender();
	return 0;
}

/*unsigned int cube_gfx_getFrameTime(void);
*/
static duk_ret_t gfx_getFrameTime(duk_context *ctx)
{
	duk_push_uint(ctx, (duk_uint_t)cube_gfx_getFrameTime());
	return 1;
}

//void cube_gfx_ClearAll(void);
static duk_ret_t gfx_clearAll(duk_context *ctx)
{
	cube_gfx_ClearAll();
	return 0;
}
//void cube_gfx_ClearBuf(void);
static duk_ret_t gfx_clearBuf(duk_context *ctx)
{
	cube_gfx_ClearBuf();
	return 0;
}
// ------------------END OF CONTROL API --------------------

// ------------------- BASIC DRAW API ----------------------

/*Makes an ARGB4444 of 8 bit R, G, B and Alpha values
nArgs: 
0: Red
1: Green
2: Blue
3: Alpha
*/
static duk_ret_t gfx_colorMake(duk_context *ctx)
{
	uint16_t r = (uint16_t)duk_require_uint(ctx, 0);
	uint16_t g = (uint16_t)duk_require_uint(ctx, 1);
	uint16_t b = (uint16_t)duk_require_uint(ctx, 2);
	uint16_t a = (uint16_t)duk_require_uint(ctx, 3);
	uint16_t color = (((a & 0xF) << 12) + ((r & 0xF) << 8) + ((g & 0xF) << 4) + (b & 0xF));
	duk_push_uint(ctx, color);
	return 1;
}

/*void cube_gfx_drawPixel(int16_t x, int16_t y, uint16_t color);
nArgs: 3
0: x
1: y
2: color
*/
static duk_ret_t gfx_drawPixel(duk_context *ctx)
{
	uint16_t x = (uint16_t)duk_require_uint(ctx, 0);
	uint16_t y = (uint16_t)duk_require_uint(ctx, 1);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 2);
	cube_gfx_drawPixel(x, y, color);
	return 0;
}

/*uint16_t cube_gfx_getPixel(int16_t x, int16_t y);
nArgs: 2
0: x
1: y
*/
static duk_ret_t gfx_getPixel(duk_context *ctx)
{
	uint16_t x = (uint16_t)duk_require_uint(ctx, 0);
	uint16_t y = (uint16_t)duk_require_uint(ctx, 1);
	uint16_t color = cube_gfx_getPixel(x, y);
	duk_push_uint(ctx, color);
	return 1;
}

/*void cube_gfx_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
nArgs: 5
IDX:
0: x
1: y
2: width
3: height
4: color
*/
static duk_ret_t gfx_drawRect(duk_context *ctx)
{
	uint16_t x = (uint16_t)duk_require_uint(ctx, 0);
	uint16_t y = (uint16_t)duk_require_uint(ctx, 1);
	uint16_t w = (uint16_t)duk_require_uint(ctx, 2);
	uint16_t h = (uint16_t)duk_require_uint(ctx, 3);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 4);
	cube_gfx_drawRect(x, y, w, h, color);
	return 0;
}

/*void cube_gfx_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
nArgs: 5
IDX:
0: x
1: y
2: width
3: height
4: color
*/
static duk_ret_t gfx_fillRect(duk_context *ctx)
{
	uint16_t x = (uint16_t)duk_require_uint(ctx, 0);
	uint16_t y = (uint16_t)duk_require_uint(ctx, 1);
	uint16_t w = (uint16_t)duk_require_uint(ctx, 2);
	uint16_t h = (uint16_t)duk_require_uint(ctx, 3);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 4);
	cube_gfx_fillRect(x, y, w, h, color);
	return 0;
}

/*void cube_gfx_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
nArgs: 4
IDX:
0: x
1: y
2: r
3: color
*/
static duk_ret_t gfx_drawCircle(duk_context *ctx)
{
	uint16_t x = (uint16_t)duk_require_uint(ctx, 0);
	uint16_t y = (uint16_t)duk_require_uint(ctx, 1);
	uint16_t r = (uint16_t)duk_require_uint(ctx, 2);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 3);
	cube_gfx_drawCircle(x, y, r, color);
	return 0;
}

/*void cube_gfx_drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
nArgs: 6
IDX:
0: x0
1: y0
2: r
3: cornername
4: color
*/
/* Actually not needed function
static duk_ret_t gfx_drawCircleHelper(duk_context *ctx)
{
	uint16_t x0 = (uint16_t)duk_require_uint(ctx, 0);
	uint16_t y0 = (uint16_t)duk_require_uint(ctx, 1);
	uint16_t r = (uint16_t)duk_require_uint(ctx, 2);
	uint8_t cornername = (uint8_t)duk_require_uint(ctx, 3);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 4);
	cube_gfx_drawCircleHelper(x0, y0, r, cornername, color);
	return 0;
}
*/
/*void cube_gfx_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
nArgs: 4
IDX:
0: x
1: y
2: r
3: color
*/
static duk_ret_t gfx_fillCircle(duk_context *ctx)
{
	uint16_t x = (uint16_t)duk_require_uint(ctx, 0);
	uint16_t y = (uint16_t)duk_require_uint(ctx, 1);
	uint16_t r = (uint16_t)duk_require_uint(ctx, 2);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 3);
	cube_gfx_fillCircle(x, y, r, color);
	return 0;
}

/*void cube_gfx_fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
nArgs: 6
IDX:
0: x0
1: y0
2: r
3: cornername
4: delta
5: color
*/
/*
static duk_ret_t gfx_fillCircleHelper(duk_context *ctx)
{

	uint16_t x0 = (uint16_t)duk_require_uint(ctx, 0);
	uint16_t y0 = (uint16_t)duk_require_uint(ctx, 1);
	uint16_t r = (uint16_t)duk_require_uint(ctx, 2);
	uint8_t cornername = (uint8_t)duk_require_uint(ctx, 3);
	int16_t delta = (int16_t)duk_require_int(ctx, 4);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 5);
	cube_gfx_fillCircleHelper(x0, y0, r, cornername, delta, color);
	return 0;
}
*/
/*void cube_gfx_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
nArgs: 7
IDX:
0: x0
1: y0
2: x1
3: y1
4: x2
5: y2
6: color
*/
static duk_ret_t gfx_drawTriangle(duk_context *ctx)
{
	int16_t x0 = (int16_t)duk_require_int(ctx, 0);
	int16_t y0 = (int16_t)duk_require_int(ctx, 1);
	int16_t x1 = (int16_t)duk_require_int(ctx, 2);
	int16_t y1 = (int16_t)duk_require_int(ctx, 3);
	int16_t x2 = (int16_t)duk_require_int(ctx, 4);
	int16_t y2 = (int16_t)duk_require_int(ctx, 5);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 6);
	cube_gfx_drawTriangle(x0, y0, x1, y1, x2, y2, color);
	return 0;
}

/*void cube_gfx_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
nArgs: 7
IDX:
0: x0
1: y0
2: x1
3: y1
4: x2
5: y3
6: color
*/
static duk_ret_t gfx_fillTriangle(duk_context *ctx)
{
	int16_t x0 = (int16_t)duk_require_int(ctx, 0);
	int16_t y0 = (int16_t)duk_require_int(ctx, 1);
	int16_t x1 = (int16_t)duk_require_int(ctx, 2);
	int16_t y1 = (int16_t)duk_require_int(ctx, 3);
	int16_t x2 = (int16_t)duk_require_int(ctx, 4);
	int16_t y2 = (int16_t)duk_require_int(ctx, 5);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 6);
	cube_gfx_fillTriangle(x0, y0, x1, y1, x2, y2, color);
	return 0;
}

/*void cube_gfx_drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
nArgs: 6
IDX:
0: x
1: y
2: width
3: height
4: corner radius
5: color
*/
static duk_ret_t gfx_drawRoundRect(duk_context *ctx)
{
		int16_t x = (int16_t)duk_require_int(ctx, 0);
	int16_t y = (int16_t)duk_require_int(ctx, 1);
	int16_t w = (int16_t)duk_require_int(ctx, 2);
	int16_t h = (int16_t)duk_require_int(ctx, 3);
	int16_t r = (int16_t)duk_require_int(ctx, 4);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 5);
	cube_gfx_drawRoundRect(x, y, w, h, r, color);
	return 0;
}

/*void cube_gfx_fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
nArgs: 6
IDX:
0: x
1: y
2: width
3: height
4: corner radius
5: color
*/
static duk_ret_t gfx_fillRoundRect(duk_context *ctx)
{
	int16_t x = (int16_t)duk_require_int(ctx, 0);
	int16_t y = (int16_t)duk_require_int(ctx, 1);
	int16_t w = (int16_t)duk_require_int(ctx, 2);
	int16_t h = (int16_t)duk_require_int(ctx, 3);
	int16_t r = (int16_t)duk_require_int(ctx, 4);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 5);
	cube_gfx_drawRoundRect(x, y, w, h, r, color);
	return 0;
}

/*void cube_gfx_drawFastVLine(int16_t x, int16_t y, int16_t l, uint16_t color);
nArgs: 4
IDX:
0: x
1: y
2: Length
3: color
*/
static duk_ret_t gfx_drawFastVLine(duk_context *ctx)
{
	int16_t x = (int16_t)duk_require_int(ctx, 0);
	int16_t y = (int16_t)duk_require_int(ctx, 1);
	int16_t l = (int16_t)duk_require_int(ctx, 2);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 3);
	cube_gfx_drawFastVLine(x, y, l, color);
	return 0;
}

/*void cube_gfx_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
nArgs: 4
IDX:
0: x
1: y
2: height
3: color
*/
static duk_ret_t gfx_drawFastHLine(duk_context *ctx)
{
	int16_t x = (int16_t)duk_require_int(ctx, 0);
	int16_t y = (int16_t)duk_require_int(ctx, 1);
	int16_t h = (int16_t)duk_require_int(ctx, 2);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 3);
	cube_gfx_drawFastHLine(x, y, h, color);
	return 0;
}

/*void cube_gfx_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
nArgs: 5
IDX:
0: x0
1: y0
2: x1
3: y1
4: color
*/
static duk_ret_t gfx_drawLine(duk_context *ctx)
{
	int16_t x0 = (int16_t)duk_require_int(ctx, 0);
	int16_t y0 = (int16_t)duk_require_int(ctx, 1);
	int16_t x1 = (int16_t)duk_require_int(ctx, 2);
	int16_t y1 = (int16_t)duk_require_int(ctx, 3);
	uint16_t color = (uint16_t)duk_require_uint(ctx, 4);
	cube_gfx_drawLine(x0, y0, x1, y1, color);
	return 0;
}

/*void cube_gfx_fillScreen(int16_t color);
nArgs: 1
IDX:
0: color

If color < 0, fill random
*/
static duk_ret_t gfx_fillScreen(duk_context *ctx)
{
	int16_t color = (int16_t)duk_require_int(ctx, 0);
	cube_gfx_fillScreen(color);
	return 0;
}

// ---------------- END OF BASIC DRAW API -------------------




// ------------------- BITMAP API ----------------------
// TODO
/*
#if 0
//void cube_gfx_drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);
//void cube_gfx_drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color, uint16_t bg);
void cube_gfx_drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
//void cube_gfx_drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg);
void cube_gfx_drawXBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color);
//void cube_gfx_drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h);
void cube_gfx_drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h);
//void cube_gfx_drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[], const uint8_t mask[], int16_t w, int16_t h);
void cube_gfx_drawGrayscaleBitmap_opaque(int16_t x, int16_t y, uint8_t *bitmap, uint8_t *mask, int16_t w, int16_t h);
//void cube_gfx_drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], int16_t w, int16_t h);
void cube_gfx_drawRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, int16_t w, int16_t h);
//void cube_gfx_drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], const uint8_t mask[], int16_t w, int16_t h);
void cube_gfx_drawRGBBitmap_opaque(int16_t x, int16_t y, uint16_t *bitmap, uint8_t *mask, int16_t w, int16_t h);
#endif
*/
// ------------------- END OF BITMAP API ----------------------

// ------------------- TEXT API ----------------------
//void writeChar(int16_t x, int16_t y, unsigned char c, uint16_t color, int16_t bg, uint8_t size_x, uint8_t size_y);
//void cube_gfx_ClearAll(void);
//void cube_gfx_ClearBuf(void);

/*void cube_gfx_drawChar_bg(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
nArgs: 6
IDX:
0: x
1: y
2: character
3: char. color
4: background color
5: size
*/
//static duk_ret_t gfx_drawCharBg(duk_context *ctx);

/*void cube_gfx_drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint8_t size);
nArgs: 5
IDX:
0: x
1: y
2: character
3: char. color
4: size
*/
//static duk_ret_t gfx_drawChar(duk_context *ctx);


/*void cube_gfx_getTextBounds(const char *string, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
nArgs: 3
IDX:
0: JavaScript String
1: X coordinate of cursor (Cursor is not set to this coordinate)
2: Y coordinate of cursor (Cursor is not actually set to this coordinate)

Return a JavaScript array with a size of 4.
Returned array indexes:  [0] X coordinate of text boundary from which width and height is referenced
                [1] Y coordinate of text boundary from which width and height is referenced
                [3] Text Width
                [4] Text Height
*/
static duk_ret_t gfx_getTextBounds(duk_context *ctx)
{
	const char *str = duk_safe_to_string(ctx, -1);
	int16_t x = (int16_t) duk_require_int(ctx, 1);
	int16_t y = (int16_t) duk_require_int(ctx, 2);

	int16_t x1, y1 = 0;
	uint16_t w, h = 0;
	cube_gfx_getTextBounds(str, x, y, &x1, &y1, &w, &h);

	duk_idx_t arr_idx;
	arr_idx = duk_push_array(ctx);
	duk_push_int(ctx, x1);
	duk_put_prop_index(ctx, arr_idx, 0);
	duk_push_int(ctx, y1);
	duk_put_prop_index(ctx, arr_idx, 1);
	duk_push_uint(ctx, w);
	duk_put_prop_index(ctx, arr_idx, 2);
	duk_push_uint(ctx, h);
	duk_put_prop_index(ctx, arr_idx, 3);
	//duk_pop(ctx);
	return 1;
}

/*uint16_t cube_gfx_getTextHeight(const char *str, int16_t x, int16_t y);
nArgs: 6
IDX:
0: String to be checked
1: X coordinate of cursor (Cursor is not set to this coordinate)
2: Y coordinate of cursor (Cursor is not actually set to this coordinate)

Return: Text
*/
//uint16_t cube_gfx_getTextWidth(const char *str, int16_t x, int16_t y);
//uint16_t cube_gfx_getTextCornerX(const char *str, int16_t x, int16_t y);
//uint16_t cube_gfx_getTextCornerY(const char *str, int16_t x, int16_t y);

/*void cube_gfx_setTextSize(uint8_t s);
nArgs: 1
IDX:
0: Size to set the text to (only Integer values are accepted)
*/
static duk_ret_t gfx_setTextSize(duk_context *ctx)
{
	uint8_t s = (uint8_t)duk_require_uint(ctx, 0);
	cube_gfx_setTextSize(s);
	return 0;
}

/*void cube_gfx_setFont(const GFXfont *f);
nArgs: 1
IDX:
0: Index of the font from the font list
*/
static duk_ret_t gfx_setFont(duk_context *ctx)
{
	uint index = duk_require_uint(ctx, 0);
	if(index >= ((sizeof(fontlist))/sizeof(GFXfont))) {
		duk_push_error_object(ctx, DUK_ERR_TYPE_ERROR, "invalid index of Font Array: %d", (uint) index);
    	return duk_throw(ctx);
	}
	cube_gfx_setFont(fontlist[index]);
	return 0;
}

/*void cube_gfx_setCursor(int16_t x, int16_t y);
nArgs: 2
IDX:
0: Cursor X
1: Cursor Y
*/
static duk_ret_t gfx_setCursor(duk_context *ctx)
{
	int16_t x = (int16_t)duk_require_int(ctx, 0);
	int16_t y = (int16_t)duk_require_int(ctx, 1);
	cube_gfx_setCursor(x,y);
	return 0;
}

//void cube_gfx_charBounds(unsigned char c, int16_t *x, int16_t *y, int16_t *minx, int16_t *miny, int16_t *maxx, int16_t *maxy);

/*void cube_gfx_setTextColor(uint16_t c);
nArgs: 1
IDX:
0: Text Color
*/
static duk_ret_t gfx_setTextColor(duk_context *ctx)
{
	uint16_t color = (uint16_t) duk_require_uint(ctx, 0);
	cube_gfx_setTextColor(color);
	return 0;
}

/*void cube_gfx_setTextColor_bg(uint16_t c, uint16_t bg);
nArgs: 2
IDX:
0: Text Color
1: background color
*/
static duk_ret_t gfx_setTextColorBg(duk_context *ctx)
{
	uint16_t tc = (uint16_t)duk_require_uint(ctx, 0);
	uint16_t bg = (uint16_t)duk_require_uint(ctx, 1);
	cube_gfx_setTextColor_bg(tc, bg);
	return 0;
}

/*void cube_gfx_setTextWrap(bool w);
nArgs: 1
IDX:
0: Bool - True: Text wraps at display edge, false: Not
*/
static duk_ret_t gfx_setTextWrap(duk_context *ctx)
{
	bool w = duk_require_boolean(ctx, 0);
	cube_gfx_setTextWrap(w);
	return 0;
}

/*void cube_gfx_Print(int16_t x, int16_t y, uint16_t color, const char *string);
nArgs: 4
IDX:
0: String to print
1: X Coordinate of cursor
2: Y Coordinate of cursor
3: Color - If NULL, color is set separatelly by GFX.setTextColor() or GFX.setTextColorBg()
*/
static duk_ret_t gfx_printText(duk_context *ctx)
{
	//duk_push_string(ctx, " ");
	//duk_insert(ctx, 0);
	//duk_join(ctx, duk_get_top(ctx) - 1);
	const char *str = duk_safe_to_string(ctx, 0);
	int16_t x = (int16_t)duk_require_int(ctx, 1);
	int16_t y = (int16_t)duk_require_int(ctx, 2);
	int16_t c = (int16_t)duk_get_int_default(ctx, 3, -1);
	cube_gfx_Print(x, y, c, str);
	return 0;
}

/*int16_t cube_gfx_getCursorX(void);
int16_t cube_gfx_getCursorY(void);
nArgs: 0

Return a JavaScript array with a size of 2.
Returned array indexes:  [0] X coordinate of cursor
                         [1] Y coordinate of cursor
*/
static duk_ret_t gfx_getCursor(duk_context *ctx)
{
	int16_t x = cube_gfx_getCursorX();
	int16_t y = cube_gfx_getCursorY();

	duk_idx_t arr_idx;
	arr_idx = duk_push_array(ctx);
	duk_push_int(ctx, x);
	duk_put_prop_index(ctx, arr_idx, 0);
	duk_push_int(ctx, y);
	duk_put_prop_index(ctx, arr_idx, 1);
	//duk_pop(ctx);
	return 1;
}
