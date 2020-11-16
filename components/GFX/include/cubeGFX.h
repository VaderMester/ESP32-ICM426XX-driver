#ifndef __CUBEGFX_H__
#define __CUBEGFX_H__

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <gfxfont.h>
#include <esp_err.h>

void sGFXinitTask(void *pParam);

/// A generic graphics superclass that can handle all sorts of drawing. At a
/// minimum you can subclass and provide drawPixel(). At a maximum you can do a
/// ton of overriding to optimize. Used for any/all Adafruit displays!
/**********************************************************************/
/*!
    @brief  Draw to the screen/framebuffer/etc.
    Must be overridden in subclass.
    @param  x    X coordinate in pixels
    @param  y    Y coordinate in pixels
    @param color  16-bit pixel color.
  */
/**********************************************************************/
// TRANSACTION API / CORE DRAW API
// These MAY be overridden by the subclass to provide device-specific
// optimized code.  Otherwise 'generic' versions are used.
esp_err_t cube_gfx_startWrite(void);
void writePixel(int16_t x, int16_t y, uint16_t color);
void cube_gfx_drawPixel(int16_t x, int16_t y, uint16_t color);
uint16_t cube_gfx_getPixel(int16_t x, int16_t y);
void cube_gfx_writeFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void cube_gfx_writeFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void cube_gfx_writeFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void cube_gfx_writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void cube_gfx_endWrite(void);
void cube_gfx_pauseRender(void);
void cube_gfx_resumeRender(void);

// CONTROL API
// These MAY be overridden by the subclass to provide device-specific
// optimized code.  Otherwise 'generic' versions are used.
void cube_gfx_setRotation(uint8_t r);
void cube_gfx_invertDisplay(bool i);
void cube_gfx_setFrameTime(unsigned int millisec);
unsigned int cube_gfx_getFrameTime(void);
// BASIC DRAW API
// These MAY be overridden by the subclass to provide device-specific
// optimized code.  Otherwise 'generic' versions are used.

// It's good to implement those, even if using transaction API
void cube_gfx_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void cube_gfx_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void cube_gfx_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void cube_gfx_fillScreen(int16_t color);
// Optional and probably not necessary to change
void cube_gfx_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void cube_gfx_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);

// These exist only with Adafruit_GFX (no subclass overrides)
void cube_gfx_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void cube_gfx_drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);

void cube_gfx_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void cube_gfx_fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
void cube_gfx_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void cube_gfx_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void cube_gfx_drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
void cube_gfx_fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
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
void writeChar(int16_t x, int16_t y, unsigned char c, uint16_t color, int16_t bg, uint8_t size_x, uint8_t size_y);
void cube_gfx_ClearAll(void);
void cube_gfx_ClearBuf(void);
void cube_gfx_drawChar_bg(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
void cube_gfx_drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint8_t size);

void cube_gfx_getTextBounds(const char *string, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
/*
uint16_t cube_gfx_getTextHeight(const char *str, int16_t x, int16_t y);
uint16_t cube_gfx_getTextWidth(const char *str, int16_t x, int16_t y);
uint16_t cube_gfx_getTextCornerX(const char *str, int16_t x, int16_t y);
uint16_t cube_gfx_getTextCornerY(const char *str, int16_t x, int16_t y);
*/
void cube_gfx_setTextSize(uint8_t s);
void setTextSize(uint8_t sx, uint8_t sy);
void cube_gfx_setFont(const GFXfont *f);

void cube_gfx_charBounds(unsigned char c, int16_t *x, int16_t *y, int16_t *minx, int16_t *miny, int16_t *maxx, int16_t *maxy);

void cube_gfx_setCursor(int16_t x, int16_t y);
void cube_gfx_setTextColor(uint16_t c);
void cube_gfx_setTextColor_bg(uint16_t c, uint16_t bg);
void cube_gfx_setTextWrap(bool w);
void cube_gfx_Print(int16_t x, int16_t y, int16_t color, const char *string);
int16_t cube_gfx_width(void);
int16_t cube_gfx_height(void);
uint8_t cube_gfx_getRotation(void);
int16_t cube_gfx_getCursorX(void);
int16_t cube_gfx_getCursorY(void);

int16_t WIDTH;        ///< This is the 'raw' display width - never changes
int16_t HEIGHT;       ///< This is the 'raw' display height - never changes
int16_t _width;       ///< Display width as modified by current rotation
int16_t _height;      ///< Display height as modified by current rotation
int16_t cursor_x;     ///< x location to start print()ing text
int16_t cursor_y;     ///< y location to start print()ing text
uint16_t textcolor;   ///< 16-bit background color for print()
uint16_t textbgcolor; ///< 16-bit text color for print()
uint8_t textsize_x;   ///< Desired magnification in X-axis of text to print()
uint8_t textsize_y;   ///< Desired magnification in Y-axis of text to print()
uint8_t rotation;     ///< Display rotation (0 thru 3)
bool wrap;            ///< If set, 'wrap' text at right edge of display
bool _cp437;          ///< If set, use correct CP437 charset (default is off)
GFXfont *gfxFont;     ///< Pointer to special font
GFXfont *font;
GFXglyph *glyph;
uint8_t DBUF;        //global index of the selected buffer index

#endif // __CUBEGFX_H__
