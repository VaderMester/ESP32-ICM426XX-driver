/*
This is the core graphics library for all our displays, providing a common
set of graphics primitives (points, lines, circles, etc.).  It needs to be
paired with a hardware-specific library for each display device we carry
(to handle the lower-level functions).

Adafruit invests time and resources providing this open source code, please
support Adafruit & open-source hardware by purchasing products from Adafruit!

Copyright (c) 2013 Adafruit Industries.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <sdkconfig.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>

#include <cubeGFX.h>

#include <esp_log.h>
#include <esp_attr.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <leddisplay.h>
//#if 0
// Many (but maybe not all) non-AVR board installs define macros
// for compatibility with existing PROGMEM-reading AVR code.
// Do our own checks and defines here for good measure...

#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(uint8_t *)(addr))
#endif
#ifndef pgm_read_word
#define pgm_read_word(addr) (*(uint16_t *)(addr))
#endif
#ifndef pgm_read_dword
#define pgm_read_dword(addr) (*(uint32_t *)(addr))
#endif

#define CONFIG_ARGB4444
//#define CONFIG_RGB565
#define CONFIG_RGB888
/* *********************************************************************************************** */
// local logging and debugging

#define LOGNAME "cubeGFX"
#define ERROR(fmt, ...)     ESP_LOGE(LOGNAME, fmt, ##__VA_ARGS__)
#define WARNING(fmt, ...)   ESP_LOGW(LOGNAME, fmt, ##__VA_ARGS__)
#define INFO(fmt, ...)      ESP_LOGI(LOGNAME, fmt, ##__VA_ARGS__)
#define DEBUG(fmt, ...)     ESP_LOGD(LOGNAME, fmt, ##__VA_ARGS__)
#define TRACE(fmt, ...)     ESP_LOGV(LOGNAME, fmt, ##__VA_ARGS__)

#define MS2TICKS(ms)              ((ms) / portTICK_PERIOD_MS)
#define osSleep(ms)               vTaskDelay(MS2TICKS(ms));

#if !defined(__INT_MAX__) || (__INT_MAX__ > 0xFFFF)
#define pgm_read_pointer(addr) ((void *)pgm_read_dword(addr));
#else
#define pgm_read_pointer(addr) ((void *)pgm_read_word(addr));
#endif

inline GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c)
{
    return gfxFont->glyph + c;
}

inline uint8_t *pgm_read_bitmap_ptr(const GFXfont *gfxFont)
{
    return gfxFont->bitmap;
}

#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) \
    {                       \
        int16_t t = a;      \
        a = b;              \
        b = t;              \
    }
#endif

//Init frame buffers

void sGFXinitTask(void *pParam)
{
    INFO("init display");
    if (leddisplay_init_multipanel() == ESP_OK)
    {
        for(int i = 0; i < CONFIG_LEDDISPLAY_NUM_DISP_BUFFER; i++) {
            if(leddisplay_displaybuffer_init(i) != ESP_OK) {
                ERROR(":-(, init of Displaybuffers failed");
                osSleep(2000);
                vTaskDelete(NULL);
            }
        }
    }
    else
    {
        ERROR(":-(, Failed to init multipanel mode");
        osSleep(2000);
        vTaskDelete(NULL);
    }
    WARNING("INIT OK");
    
    BaseType_t update;
    if(CONFIG_LEDDISPLAY_TASK_CORE_AFFINITY == -1) {
        update = xTaskCreatePinnedToCore(sDisplayUpdateTask, "display_update", 8192 / sizeof(StackType_t), NULL, 10, NULL, tskNO_AFFINITY);
    } else {
        update = xTaskCreatePinnedToCore(sDisplayUpdateTask, "display_update", 8192 / sizeof(StackType_t), NULL, 10, NULL, CONFIG_LEDDISPLAY_TASK_CORE_AFFINITY);
    }
    if (update != pdPASS)
    {
        WARNING("Unable to create 'Update Task'! ERROR Code: %i", update);
    }
    else
    {
        assert(update == pdPASS);
        INFO("Successfully created 'Leddisplay UPDATE task");
    }
    WIDTH = LEDDISPLAY_WIDTH;
    HEIGHT = LEDDISPLAY_HEIGHT;
    _width = WIDTH;
    _height = HEIGHT;
    rotation = 0;
    cursor_y = cursor_x = 0;
    textsize_x = textsize_y = 1;
    textcolor = textbgcolor = 0xFFFF;
    wrap = true;
    _cp437 = false;
    gfxFont = NULL;
    DBUF = 0;
    
    WARNING("heap: EXEC free=%u (min=%u, largest=%u), 32BIT free=%u (min=%u, largest=%u), 8BIT free=%u (min=%u, largest=%u), DMA free=%u (min=%u, largest=%u)",
            heap_caps_get_free_size(MALLOC_CAP_EXEC), heap_caps_get_minimum_free_size(MALLOC_CAP_EXEC), heap_caps_get_largest_free_block(MALLOC_CAP_EXEC),
            heap_caps_get_free_size(MALLOC_CAP_32BIT), heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT), heap_caps_get_largest_free_block(MALLOC_CAP_32BIT),
            heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
            heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_minimum_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
    vTaskDelete(NULL);
}

/**************************************************************************/
/*!
   @brief    Start a display-writing routine, overwrite in subclasses.
*/
/**************************************************************************/
esp_err_t cube_gfx_startWrite(void)
{
    esp_err_t res = ESP_OK;
    int taken = DisplayBufferTake();
    if (taken != -1)
    {
        DBUF = taken;
    }
    else
    {
        res = ESP_FAIL;
    }
    return res;
}
/**************************************************************************/
/*!
   @brief    End a display-writing routine, overwrite in subclasses if
   startWrite is defined!
*/
/**************************************************************************/
void cube_gfx_endWrite()
{
    DisplayBufferFinish(DBUF);
}

void cube_gfx_pauseRender(void)
{	
	Pause_DisplayRender();
}

void cube_gfx_resumeRender(void)
{
	Resume_DisplayRender();
}


/**************************************************************************/
/*!
   @brief    Draw a pixel
    @param   x   x coordinate
    @param   y   y coordinate
   @param    red 8-bit red component
    @param    green 8-bit green component
     @param    blue 8-bit blue component
*/
/**************************************************************************/
/*
void cube_gfx_drawPixel(int16_t x, int16_t y, uint8_t red, uint8_t green, uint8_t blue)
{
    writePixel(x, y, red, green, blue);
}
*/
/* RGB888 - TODO
inline void writePixel(int16_t x, int16_t y, uint8_t red, uint8_t green, uint8_t blue)
{
    if (buffer)
    {
        if ((x < 0) || (y < 0) || (x >= _width) || (y >= _height)) {
            return;
        }
        int16_t t;
        switch (rotation)
        {
        case 1:
            t = x;
            x = WIDTH - 1 - y;
            y = t;
            break;
        case 2:
            x = WIDTH - 1 - x;
            y = HEIGHT - 1 - y;
            break;
        case 3:
            t = x;
            x = y;
            y = HEIGHT - 1 - t;
            break;
        }
        leddisplay_panels_xy_rgb(DBUF, 1, (unsigned)x, (unsigned)y, red, green, blue);
    }
}
*/
inline void writePixel(int16_t x, int16_t y, uint16_t color)
{
    if ((x < 0) || (y < 0) || (x >= _width) || (y >= _height))
    {
        return;
    }
    int16_t t;
    switch (rotation)
    {
    case 1:
        t = x;
        x = WIDTH - 1 - y;
        y = t;
        break;
    case 2:
        x = WIDTH - 1 - x;
        y = HEIGHT - 1 - y;
        break;
    case 3:
        t = x;
        x = y;
        y = HEIGHT - 1 - t;
        break;
    }
    #ifdef CONFIG_RGB565
    uint8_t r = (color & 0xF800) >> 8;
    uint8_t g = (color & 0x07E0) >> 3;
    uint8_t b = (color & 0x001F) << 3;
    leddisplay_panels_xy_rgb(DBUF, 1, (unsigned)x, (unsigned)y, r, g, b);
    #endif
    #ifdef CONFIG_ARGB4444
    /*
    uint8_t alpha = ((color & 0xF000) >> 8);
    uint8_t r = ((color & 0x0F00) >> 4) * 0x11;
    uint8_t g = (color & 0x00F0) * 0x11;
    uint8_t b = ((color & 0x000F) << 4) * 0x11;
    */
    uint8_t alpha = ((color & 0xF000) >> 8);
    uint8_t r = ((color & 0x0F00) >> 4) + 15;
    uint8_t g = (color & 0x00F0) + 15;
    uint8_t b = ((color & 0x000F) << 4) + 15;
    leddisplay_panels_xy_argb(DBUF, 1, (unsigned)x, (unsigned)y, alpha, r, g, b);
    #endif
}

void cube_gfx_ClearBuf(void) {
    leddisplay_panels_clear(DBUF, 0);
}

void cube_gfx_ClearAll(void) {
    for (int i = 0; i < CONFIG_LEDDISPLAY_NUM_DISP_BUFFER; i++)
    {
        cube_gfx_startWrite();
        leddisplay_panels_clear(DBUF, 0);
        cube_gfx_endWrite();
    }
}
/**************************************************************************/
/*!
   @brief    Write a pixel, overwrite in subclasses if startWrite is defined!
    @param   x   x coordinate
    @param   y   y coordinate
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void cube_gfx_drawPixel(int16_t x, int16_t y, uint16_t color)
{
    //drawPixel(x, y, color);
    writePixel(x, y, color);
}

/**************************************************************************/
/*!
   @brief    Reads pixel data at given coordinates
    @param   x   x coordinate
    @param   y   y coordinate

    @return    color 16-bit 4-4-4-4 ARGB Color of the pixel. A (alpha) will be 0 always with this read.
*/
/**************************************************************************/
uint16_t cube_gfx_getPixel(int16_t x, int16_t y)
{
    if ((x < 0) || (y < 0) || (x >= _width) || (y >= _height))
    {
        return 0;
    }
    uint8_t r, g, b;
    leddisplay_panels_get_rgb_xy(DBUF, (unsigned)x, (unsigned)y, &r, &b, &g);
    return (((r & 0xF) << 8) + ((g & 0xF) << 4) + (b & 0xF));
}

/**************************************************************************/
/*!
   @brief    Write a line.  Bresenham's algorithm - thx wikpedia
    @param    x0  Start point x coordinate
    @param    y0  Start point y coordinate
    @param    x1  End point x coordinate
    @param    y1  End point y coordinate
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void writeLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)
    {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
    }
    if (x0 > x1)
    {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
    }
    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);
    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1)
    {
        ystep = 1;
    }
    else
    {
        ystep = -1;
    }

    for (; x0 <= x1; x0++)
    {
        if (steep)
        {
            writePixel(y0, x0, color);
        }
        else
        {
            writePixel(y0, x0, color);
        }
        err -= dy;
        if (err < 0)
        {
            y0 += ystep;
            err += dx;
        }
    }
}


/**************************************************************************/
/*!
   @brief    Write a perfectly horizontal line, overwrite in subclasses if
   startWrite is defined!
    @param    x   Left-most x coordinate
    @param    y   Left-most y coordinate
    @param    w   Width in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void cube_gfx_drawFastHline(int16_t x, int16_t y, int16_t w,
                             uint16_t color)
{
    // Overwrite in subclasses if startWrite is defined!
    // Example: writeLine(x, y, x+w-1, y, color);
    // or fillRect(x, y, w, 1, color);
    cube_gfx_drawFastHLine(x, y, w, color);
}

/**************************************************************************/
/*!
   @brief    Draw a perfectly vertical line (this is often optimized in a
   subclass!)
    @param    x   Top-most x coordinate
    @param    y   Top-most y coordinate
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void cube_gfx_drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
    writeLine(x, y, x, y + h - 1, color);
}

/**************************************************************************/
/*!
   @brief    Draw a perfectly horizontal line (this is often optimized in a
   subclass!)
    @param    x   Left-most x coordinate
    @param    y   Left-most y coordinate
    @param    w   Width in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void cube_gfx_drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
    writeLine(x, y, x + w - 1, y, color);
}

/**************************************************************************/
/*!
   @brief    Fill a rectangle completely with one color. Update in subclasses if
   desired!
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    w   Width in pixels
    @param    h   Height in pixels
   @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void cube_gfx_fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    for (int16_t i = x; i < x + w; i++)
    {
        cube_gfx_drawFastVLine(i, y, h, color);
    }
}

/**************************************************************************/
/*!
   @brief    Fill the screen completely with one color. Update in subclasses if
   desired!
    @param    color 16-bit 5-6-5 Color to fill with, if -1, random fill
*/
/**************************************************************************/
void cube_gfx_fillScreen(int16_t color)
{
    if(color > -1) {
#ifdef CONFIG_RGB565
        uint8_t r = (color & 0xF800) >> 8;
        uint8_t g = (color & 0x07E0) >> 3;
        uint8_t b = (color & 0x001F) << 3;
        leddisplay_panels_fill_rgb(DBUF, 0, r, g, b);
#endif
#ifdef CONFIG_ARGB4444
        uint8_t r = ((color & 0x0F00) >> 4) * 0x11;
        uint8_t g = (color & 0x00F0) * 0x11;
        uint8_t b = ((color & 0x000F) << 4) * 0x11;
        leddisplay_panels_fill_rgb(DBUF, 0, r, g, b);
#endif
    } else {
    leddisplay_panels_fill_random(DBUF, 0);
    }
}

/**************************************************************************/
/*!
   @brief    Draw a line
    @param    x0  Start point x coordinate
    @param    y0  Start point y coordinate
    @param    x1  End point x coordinate
    @param    y1  End point y coordinate
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void cube_gfx_drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                       uint16_t color)
{
    writeLine(x0, y0, x1, y1, color);
}

/**************************************************************************/
/*!
   @brief    Draw a circle outline
    @param    x0   Center-point x coordinate
    @param    y0   Center-point y coordinate
    @param    r   Radius of circle
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void cube_gfx_drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    writePixel(x0, y0 + r, color);
    writePixel(x0, y0 - r, color);
    writePixel(x0 + r, y0, color);
    writePixel(x0 - r, y0, color);

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        writePixel(x0 + x, y0 + y, color);
        writePixel(x0 - x, y0 + y, color);
        writePixel(x0 + x, y0 - y, color);
        writePixel(x0 - x, y0 - y, color);
        writePixel(x0 + y, y0 + x, color);
        writePixel(x0 - y, y0 + x, color);
        writePixel(x0 + y, y0 - x, color);
        writePixel(x0 - y, y0 - x, color);
    }
}

/**************************************************************************/
/*!
    @brief    Quarter-circle drawer, used to do circles and roundrects
    @param    x0   Center-point x coordinate
    @param    y0   Center-point y coordinate
    @param    r   Radius of circle
    @param    cornername  Mask bit #1 or bit #2 to indicate which quarters of
   the circle we're doing
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void cube_gfx_drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        if (cornername & 0x4)
        {
            writePixel(x0 + x, y0 + y, color);
            writePixel(x0 + y, y0 + x, color);
        }
        if (cornername & 0x2)
        {
            writePixel(x0 + x, y0 - y, color);
            writePixel(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8)
        {
            writePixel(x0 - y, y0 + x, color);
            writePixel(x0 - x, y0 + y, color);
        }
        if (cornername & 0x1)
        {
            writePixel(x0 - y, y0 - x, color);
            writePixel(x0 - x, y0 - y, color);
        }
    }
}

/**************************************************************************/
/*!
   @brief    Draw a circle with filled color
    @param    x0   Center-point x coordinate
    @param    y0   Center-point y coordinate
    @param    r   Radius of circle
    @param    color 16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void cube_gfx_fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{

    cube_gfx_drawFastVLine(x0, y0 - r, 2 * r + 1, color);
    cube_gfx_fillCircleHelper(x0, y0, r, 3, 0, color);
}

/**************************************************************************/
/*!
    @brief  Quarter-circle drawer with fill, used for circles and roundrects
    @param  x0       Center-point x coordinate
    @param  y0       Center-point y coordinate
    @param  r        Radius of circle
    @param  corners  Mask bits indicating which quarters we're doing
    @param  delta    Offset from center-point, used for round-rects
    @param  color    16-bit 5-6-5 Color to fill with
*/
/**************************************************************************/
void cube_gfx_fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color)
{
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;
    int16_t px = x;
    int16_t py = y;

    delta++; // Avoid some +1's in the loop

    while (x < y)
    {
        if (f >= 0)
        {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        // These checks avoid double-drawing certain lines, important
        // for the SSD1306 library which has an INVERT drawing mode.
        if (x < (y + 1))
        {
            if (corners & 1) {
                cube_gfx_drawFastVLine(x0 + x, y0 - y, 2 * y + delta, color);
            }
            if (corners & 2) {
                cube_gfx_drawFastVLine(x0 - x, y0 - y, 2 * y + delta, color);
            }
        }
        if (y != py)
        {
            if (corners & 1) {
                cube_gfx_drawFastVLine(x0 + py, y0 - px, 2 * px + delta, color);
            }
            if (corners & 2) {
                cube_gfx_drawFastVLine(x0 - py, y0 - px, 2 * px + delta, color);
            }
            py = y;
        }
        px = x;
    }
}

/**************************************************************************/
/*!
   @brief   Draw a rectangle with no fill color
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    w   Width in pixels
    @param    h   Height in pixels
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void cube_gfx_drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    cube_gfx_drawFastHline(x, y, w, color);
    cube_gfx_drawFastHline(x, y + h - 1, w, color);
    cube_gfx_drawFastVLine(x, y, h, color);
    cube_gfx_drawFastVLine(x + w - 1, y, h, color);
}

/**************************************************************************/
/*!
   @brief   Draw a rounded rectangle with no fill color
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    w   Width in pixels
    @param    h   Height in pixels
    @param    r   Radius of corner rounding
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void cube_gfx_drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if (r > max_radius) {
        r = max_radius;
    }
    // smarter version

    cube_gfx_drawFastHline(x + r, y, w - 2 * r, color);         // Top
    cube_gfx_drawFastHline(x + r, y + h - 1, w - 2 * r, color); // Bottom
    cube_gfx_drawFastVLine(x, y + r, h - 2 * r, color);         // Left
    cube_gfx_drawFastVLine(x + w - 1, y + r, h - 2 * r, color); // Right
    // draw four corners
    cube_gfx_drawCircleHelper(x + r, y + r, r, 1, color);
    cube_gfx_drawCircleHelper(x + w - r - 1, y + r, r, 2, color);
    cube_gfx_drawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color);
    cube_gfx_drawCircleHelper(x + r, y + h - r - 1, r, 8, color);
}

/**************************************************************************/
/*!
   @brief   Draw a rounded rectangle with fill color
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    w   Width in pixels
    @param    h   Height in pixels
    @param    r   Radius of corner rounding
    @param    color 16-bit 5-6-5 Color to draw/fill with
*/
/**************************************************************************/
void cube_gfx_fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color)
{
    int16_t max_radius = ((w < h) ? w : h) / 2; // 1/2 minor axis
    if (r > max_radius) {
        r = max_radius;
    }
    // smarter version
    cube_gfx_fillRect(x + r, y, w - 2 * r, h, color);
    // draw four corners
    cube_gfx_fillCircleHelper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1, color);
    cube_gfx_fillCircleHelper(x + r, y + r, r, 2, h - 2 * r - 1, color);
}

/**************************************************************************/
/*!
   @brief   Draw a triangle with no fill color
    @param    x0  Vertex #0 x coordinate
    @param    y0  Vertex #0 y coordinate
    @param    x1  Vertex #1 x coordinate
    @param    y1  Vertex #1 y coordinate
    @param    x2  Vertex #2 x coordinate
    @param    y2  Vertex #2 y coordinate
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void cube_gfx_drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    cube_gfx_drawLine(x0, y0, x1, y1, color);
    cube_gfx_drawLine(x1, y1, x2, y2, color);
    cube_gfx_drawLine(x2, y2, x0, y0, color);
}

/**************************************************************************/
/*!
   @brief     Draw a triangle with color-fill
    @param    x0  Vertex #0 x coordinate
    @param    y0  Vertex #0 y coordinate
    @param    x1  Vertex #1 x coordinate
    @param    y1  Vertex #1 y coordinate
    @param    x2  Vertex #2 x coordinate
    @param    y2  Vertex #2 y coordinate
    @param    color 16-bit 5-6-5 Color to fill/draw with
*/
/**************************************************************************/
void cube_gfx_fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color)
{
    int16_t a, b, y, last;
    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1)
    {
        _swap_int16_t(y0, y1);
        _swap_int16_t(x0, x1);
    }
    if (y1 > y2)
    {
        _swap_int16_t(y2, y1);
        _swap_int16_t(x2, x1);
    }
    if (y0 > y1)
    {
        _swap_int16_t(y0, y1);
        _swap_int16_t(x0, x1);
    }
    if (y0 == y2)
    { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if (x1 < a) {
            a = x1;
        } else if (x1 > b) {
            b = x1;
        }
        if (x2 < a) {
            a = x2;
        } else if (x2 > b) {
            b = x2;
        }
        cube_gfx_drawFastHline(a, y0, b - a + 1, color);
        return;
    }

    int16_t dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0,
            dx12 = x2 - x1, dy12 = y2 - y1;
    int32_t sa = 0, sb = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if (y1 == y2) {
        last = y1; // Include y1 scanline
    } else {
        last = y1 - 1; // Skip it
    }
    for (y = y0; y <= last; y++)
    {
        a = x0 + sa / dy01;
        b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
        if (a > b) {
            _swap_int16_t(a, b);
        }
        cube_gfx_drawFastHline(a, y, b - a + 1, color);
    }
    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = (int32_t)dx12 * (y - y1);
    sb = (int32_t)dx02 * (y - y0);
    for (; y <= y2; y++)
    {
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
        if (a > b) {
            _swap_int16_t(a, b);
        }
        cube_gfx_drawFastHline(a, y, b - a + 1, color);
    }
}

// BITMAP / XBITMAP / GRAYSCALE / RGB BITMAP FUNCTIONS ---------------------

#if 0
/**************************************************************************/
/*!
   @brief      Draw a PROGMEM-resident 1-bit image at the specified (x,y)
   position, using the specified foreground color (unset bits are transparent).
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with monochrome bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
/*
void cube_gfx_drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
                         int16_t w, int16_t h, uint16_t color)
{

    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for (int16_t j = 0; j < h; j++, y++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            if (i & 7) {
                byte <<= 1;
            } else {
                byte = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
            }
            if (byte & 0x80) {
                writePixel(x + i, y, color);
            }
        }
    }
}
*/
/**************************************************************************/
/*!
   @brief      Draw a PROGMEM-resident 1-bit image at the specified (x,y)
   position, using the specified foreground (for set bits) and background (unset
   bits) colors.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with monochrome bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
    @param    color 16-bit 5-6-5 Color to draw pixels with
    @param    bg 16-bit 5-6-5 Color to draw background with
*/
/**************************************************************************/
/*
void cube_gfx_drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
                         int16_t w, int16_t h, uint16_t color,
                         uint16_t bg)
{

    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for (int16_t j = 0; j < h; j++, y++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            if (i & 7) {
                byte <<= 1;
            } else {
                byte = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
            }
            writePixel(x + i, y, (byte & 0x80) ? color : bg);
        }
    }
}
*/
/**************************************************************************/
/*!
   @brief      Draw a RAM-resident 1-bit image at the specified (x,y) position,
   using the specified foreground color (unset bits are transparent).
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with monochrome bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
    @param    color 16-bit 5-6-5 Color to draw with
*/
/**************************************************************************/
void cube_gfx_drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color)
{

    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;
    for (int16_t j = 0; j < h; j++, y++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            if (i & 7) {
                byte <<= 1;
            } else {
                byte = bitmap[j * byteWidth + i / 8];
            }
            if (byte & 0x80) {
                writePixel(x + i, y, color);
            }
        }
    }
}

/**************************************************************************/
/*!
   @brief      Draw a RAM-resident 1-bit image at the specified (x,y) position,
   using the specified foreground (for set bits) and background (unset bits)
   colors.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with monochrome bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
    @param    color 16-bit 5-6-5 Color to draw pixels with
    @param    bg 16-bit 5-6-5 Color to draw background with
*/
/**************************************************************************/
/*
void cube_gfx_drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg)
{
    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for (int16_t j = 0; j < h; j++, y++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            if (i & 7) {
                byte <<= 1;
            } else {
                byte = bitmap[j * byteWidth + i / 8];
            }
            writePixel(x + i, y, (byte & 0x80) ? color : bg);
        }
    }
}
*/
/**************************************************************************/
/*!
   @brief      Draw PROGMEM-resident XBitMap Files (*.xbm), exported from GIMP.
   Usage: Export from GIMP to *.xbm, rename *.xbm to *.c and open in editor.
   C Array can be directly used with this function.
   There is no RAM-resident version of this function; if generating bitmaps
   in RAM, use the format defined by drawBitmap() and call that instead.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with monochrome bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
    @param    color 16-bit 5-6-5 Color to draw pixels with
*/
/**************************************************************************/
/*
void cube_gfx_drawXBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w, int16_t h, uint16_t color)
{
    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    for (int16_t j = 0; j < h; j++, y++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            if (i & 7) {
                byte >>= 1;
            } else {
                byte = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
            }
            // Nearly identical to drawBitmap(), only the bit order
            // is reversed here (left-to-right = LSB to MSB):
            if (byte & 0x01) {
                writePixel(x + i, y, color);
            }
        }
    }
}
*/
/**************************************************************************/
/*!
   @brief   Draw a PROGMEM-resident 8-bit image (grayscale) at the specified
   (x,y) pos. Specifically for 8-bit display devices such as IS31FL3731; no
   color reduction/expansion is performed.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with grayscale bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
/*
void cube_gfx_drawGrayscaleBitmap(int16_t x, int16_t y,
                                  const uint8_t bitmap[], int16_t w,
                                  int16_t h)
{

    for (int16_t j = 0; j < h; j++, y++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            writePixel(x + i, y, (uint8_t)pgm_read_byte(&bitmap[j * w + i]));
        }
    }
}
*/
/**************************************************************************/
/*!
   @brief   Draw a RAM-resident 8-bit image (grayscale) at the specified (x,y)
   pos. Specifically for 8-bit display devices such as IS31FL3731; no color
   reduction/expansion is performed.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with grayscale bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
void cube_gfx_drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h)
{
    for (int16_t j = 0; j < h; j++, y++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            writePixel(x + i, y, bitmap[j * w + i]);
        }
    }
}

/**************************************************************************/
/*!
   @brief   Draw a PROGMEM-resident 8-bit image (grayscale) with a 1-bit mask
   (set bits = opaque, unset bits = clear) at the specified (x,y) position.
   BOTH buffers (grayscale and mask) must be PROGMEM-resident.
   Specifically for 8-bit display devices such as IS31FL3731; no color
   reduction/expansion is performed.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with grayscale bitmap
    @param    mask  byte array with mask bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
/*
void cube_gfx_drawGrayscaleBitmap(int16_t x, int16_t y,
                                  const uint8_t bitmap[],
                                  const uint8_t mask[], int16_t w,
                                  int16_t h)
{
    int16_t bw = (w + 7) / 8; // Bitmask scanline pad = whole byte
    uint8_t byte = 0;

    for (int16_t j = 0; j < h; j++, y++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            if (i & 7) {
                byte <<= 1;
            }
            else
            {
                byte = pgm_read_byte(&mask[j * bw + i / 8]);
            }
            if (byte & 0x80)
            {
                writePixel(x + i, y, (uint8_t)pgm_read_byte(&bitmap[j * w + i]));
            }
        }
    }
}
*/
/**************************************************************************/
/*!
   @brief   Draw a RAM-resident 8-bit image (grayscale) with a 1-bit mask
   (set bits = opaque, unset bits = clear) at the specified (x,y) position.
   BOTH buffers (grayscale and mask) must be RAM-residentt, no mix-and-match
   Specifically for 8-bit display devices such as IS31FL3731; no color
   reduction/expansion is performed.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with grayscale bitmap
    @param    mask  byte array with mask bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
void cube_gfx_drawGrayscaleBitmap_opaque(int16_t x, int16_t y, uint8_t *bitmap,
                                  uint8_t *mask, int16_t w, int16_t h)
{
    int16_t bw = (w + 7) / 8; // Bitmask scanline pad = whole byte
    uint8_t byte = 0;

    for (int16_t j = 0; j < h; j++, y++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            if (i & 7) {
                byte <<= 1;
            } else {
                byte = mask[j * bw + i / 8];
            }
            if (byte & 0x80)
            {
                writePixel(x + i, y, bitmap[j * w + i]);
            }
        }
    }
}

/**************************************************************************/
/*!
   @brief   Draw a PROGMEM-resident 16-bit image (RGB 5/6/5) at the specified
   (x,y) position. For 16-bit display devices; no color reduction performed.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with 16-bit color bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
/*
void cube_gfx_drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[],
                            int16_t w, int16_t h)
{
    for (int16_t j = 0; j < h; j++, y++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            writePixel(x + i, y, pgm_read_word(&bitmap[j * w + i]));
        }
    }
}
*/
/**************************************************************************/
/*!
   @brief   Draw a RAM-resident 16-bit image (RGB 5/6/5) at the specified (x,y)
   position. For 16-bit display devices; no color reduction performed.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with 16-bit color bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
void cube_gfx_drawRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap,
                            int16_t w, int16_t h)
{
    for (int16_t j = 0; j < h; j++, y++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            writePixel(x + i, y, bitmap[j * w + i]);
        }
    }
}

/**************************************************************************/
/*!
   @brief   Draw a PROGMEM-resident 16-bit image (RGB 5/6/5) with a 1-bit mask
   (set bits = opaque, unset bits = clear) at the specified (x,y) position. BOTH
   buffers (color and mask) must be PROGMEM-resident. For 16-bit display
   devices; no color reduction performed.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with 16-bit color bitmap
    @param    mask  byte array with monochrome mask bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
/*
void cube_gfx_drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[],
                            const uint8_t mask[], int16_t w, int16_t h)
{
    int16_t bw = (w + 7) / 8; // Bitmask scanline pad = whole byte
    uint8_t byte = 0;

    for (int16_t j = 0; j < h; j++, y++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            if (i & 7) {
                byte <<= 1;
            } else {
                byte = pgm_read_byte(&mask[j * bw + i / 8]);
            }
            if (byte & 0x80)
            {
                writePixel(x + i, y, pgm_read_word(&bitmap[j * w + i]));
            }
        }
    }
}
*/
/**************************************************************************/
/*!
   @brief   Draw a RAM-resident 16-bit image (RGB 5/6/5) with a 1-bit mask (set
   bits = opaque, unset bits = clear) at the specified (x,y) position. BOTH
   buffers (color and mask) must be RAM-resident. For 16-bit display devices; no
   color reduction performed.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with 16-bit color bitmap
    @param    mask  byte array with monochrome mask bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
void cube_gfx_drawRGBBitmap_opaque(int16_t x, int16_t y, uint16_t *bitmap, uint8_t *mask, int16_t w, int16_t h)
{
    int16_t bw = (w + 7) / 8; // Bitmask scanline pad = whole byte
    uint8_t byte = 0;
    for (int16_t j = 0; j < h; j++, y++)
    {
        for (int16_t i = 0; i < w; i++)
        {
            if (i & 7) {
                byte <<= 1;
            } else {
                byte = mask[j * bw + i / 8];
            }
            if (byte & 0x80)
            {
                writePixel(x + i, y, bitmap[j * w + i]);
            }
        }
    }
}
#endif
// TEXT- AND CHARACTER-HANDLING FUNCTIONS ----------------------------------

// Draw a character without background
/**************************************************************************/
/*!
   @brief   Draw a single character
    @param    x   Bottom left corner x coordinate
    @param    y   Bottom left corner y coordinate
    @param    c   The 8-bit font-indexed character (likely ascii)
    @param    color 16-bit 5-6-5 Color to draw chraracter with
    @param    size  Font magnification level, 1 is 'original' size
*/
/**************************************************************************/

void cube_gfx_drawChar(int16_t x, int16_t y, unsigned char c,
                            uint16_t color, uint8_t size) {
  writeChar(x, y, c, color, color, size, size);
}
// Draw a character without background
/**************************************************************************/
/*!
   @brief   Draw a single character
    @param    x   Bottom left corner x coordinate
    @param    y   Bottom left corner y coordinate
    @param    c   The 8-bit font-indexed character (likely ascii)
    @param    color 16-bit 5-6-5 Color to draw chraracter with
    @param    size  Font magnification level, 1 is 'original' size
*/
/**************************************************************************/

void cube_gfx_drawChar_bg(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size) {
  writeChar(x, y, c, color, bg, size, size);
}

// write a character
/**************************************************************************/
/*!
   @brief   write a single character
    @param    x   Bottom left corner x coordinate
    @param    y   Bottom left corner y coordinate
    @param    c   The 8-bit font-indexed character (likely ascii)
    @param    color 16-bit 5-6-5 Color to draw chraracter with
    @param    bg 16-bit 5-6-5 Color to fill background with (if same as color,
   no background)
    @param    size_x  Font magnification level in X-axis, 1 is 'original' size
    @param    size_y  Font magnification level in Y-axis, 1 is 'original' size
*/
/**************************************************************************/
void writeChar(int16_t x, int16_t y, unsigned char c, uint16_t color, int16_t bg, uint8_t size_x, uint8_t size_y)
{
    if (!gfxFont)
    { // 'Classic' built-in font

        if ((x >= _width) ||              // Clip right
            (y >= _height) ||             // Clip bottom
            ((x + 6 * size_x - 1) < 0) || // Clip left
            ((y + 8 * size_y - 1) < 0))   // Clip top
        {
            return;
        }
        if (!_cp437 && (c >= 176)) {
            c++; // Handle 'classic' charset behavior
        }
        for (int8_t i = 0; i < 5; i++)
        { // Char bitmap = 5 columns
            uint8_t line = pgm_read_byte(&font[c * 5 + i]);
            for (int8_t j = 0; j < 8; j++, line >>= 1)
            {
                if (line & 1)
                {
                    if (size_x == 1 && size_y == 1)
                        writePixel(x + i, y + j, color);
                    else
                        cube_gfx_fillRect(x + i * size_x, y + j * size_y, size_x, size_y,
                                      color);
                }
                else if (bg != color)
                {
                    if (size_x == 1 && size_y == 1) {
                        writePixel(x + i, y + j, bg);
                    } else {
                        cube_gfx_fillRect(x + i * size_x, y + j * size_y, size_x, size_y, bg);
                    }
                }
            }
        }
        if (bg != color)
        { // If opaque, draw vertical line for last column
            if (size_x == 1 && size_y == 1) {
                cube_gfx_drawFastVLine(x + 5, y, 8, bg);
            } else {
                cube_gfx_fillRect(x + 5 * size_x, y, size_x, 8 * size_y, bg);
            }
        }
    }
    else
    { // Custom font

        // Character is assumed previously filtered by write() to eliminate
        // newlines, returns, non-printable characters, etc.  Calling
        // drawChar() directly with 'bad' characters of font may cause mayhem!

        c -= (uint8_t)pgm_read_byte(&gfxFont->first);
        GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c);
        uint8_t *bitmap = pgm_read_bitmap_ptr(gfxFont);

        uint16_t bo = pgm_read_word(&glyph->bitmapOffset);
        uint8_t w = pgm_read_byte(&glyph->width), h = pgm_read_byte(&glyph->height);
        int8_t xo = pgm_read_byte(&glyph->xOffset),
               yo = pgm_read_byte(&glyph->yOffset);
        uint8_t xx, yy, bits = 0, bit = 0;
        int16_t xo16 = 0, yo16 = 0;

        if (size_x > 1 || size_y > 1)
        {
            xo16 = xo;
            yo16 = yo;
        }

        // Todo: Add character clipping here

        // NOTE: THERE IS NO 'BACKGROUND' COLOR OPTION ON CUSTOM FONTS.
        // THIS IS ON PURPOSE AND BY DESIGN.  The background color feature
        // has typically been used with the 'classic' font to overwrite old
        // screen contents with new data.  This ONLY works because the
        // characters are a uniform size; it's not a sensible thing to do with
        // proportionally-spaced fonts with glyphs of varying sizes (and that
        // may overlap).  To replace previously-drawn text when using a custom
        // font, use the getTextBounds() function to determine the smallest
        // rectangle encompassing a string, erase the area with fillRect(),
        // then draw new text.  This WILL infortunately 'blink' the text, but
        // is unavoidable.  Drawing 'background' pixels will NOT fix this,
        // only creates a new set of problems.  Have an idea to work around
        // this (a canvas object type for MCUs that can afford the RAM and
        // displays supporting setAddrWindow() and pushColors()), but haven't
        // implemented this yet.

        for (yy = 0; yy < h; yy++)
        {
            for (xx = 0; xx < w; xx++)
            {
                if (!(bit++ & 7))
                {
                    bits = pgm_read_byte(&bitmap[bo++]);
                }
                if (bits & 0x80)
                {
                    if (size_x == 1 && size_y == 1)
                    {
                        writePixel(x + xo + xx, y + yo + yy, color);
                    }
                    else
                    {
                        cube_gfx_fillRect(x + (xo16 + xx) * size_x, y + (yo16 + yy) * size_y,
                                      size_x, size_y, color);
                    }
                }
                bits <<= 1;
            }
        }

    } // End classic vs custom font
}
/**************************************************************************/
/*!
    @brief  Print one byte/character of data, used to support print()
    @param  c  The 8-bit ascii character to write
*/
/**************************************************************************/
size_t cube_gfx_write(uint8_t c)
{
    if (!gfxFont)
    { // 'Classic' built-in font

        if (c == '\n')
        {                               // Newline?
            cursor_x = 0;               // Reset x to zero,
            cursor_y += textsize_y * 8; // advance y one line
        }
        else if (c != '\r')
        { // Ignore carriage returns
            if (wrap && ((cursor_x + textsize_x * 6) > _width))
            {                               // Off right?
                cursor_x = 0;               // Reset x to zero,
                cursor_y += textsize_y * 8; // advance y one line
            }
            writeChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize_x,
                     textsize_y);
            cursor_x += textsize_x * 6; // Advance x one char
        }
    }
    else
    { // Custom font

        if (c == '\n')
        {
            cursor_x = 0;
            cursor_y +=
                (int16_t)textsize_y * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
        }
        else if (c != '\r')
        {
            uint8_t first = pgm_read_byte(&gfxFont->first);
            if ((c >= first) && (c <= (uint8_t)pgm_read_byte(&gfxFont->last)))
            {
                GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c - first);
                uint8_t w = pgm_read_byte(&glyph->width),
                        h = pgm_read_byte(&glyph->height);
                if ((w > 0) && (h > 0))
                {                                                        // Is there an associated bitmap?
                    int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset); // sic
                    if (wrap && ((cursor_x + textsize_x * (xo + w)) > _width))
                    {
                        cursor_x = 0;
                        cursor_y += (int16_t)textsize_y *
                                    (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
                    }
                    writeChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize_x,
                             textsize_y);
                }
                cursor_x +=
                    (uint8_t)pgm_read_byte(&glyph->xAdvance) * (int16_t)textsize_x;
            }
        }
    }
    return 1;
}
/**************************************************************************/
/*!
    @brief  Print string starting at x,y coordinates
    @param  c  The 8-bit ascii character to write
*/
/**************************************************************************/
void cube_gfx_Print(int16_t x, int16_t y, int16_t color, const char *string) {
    cube_gfx_setCursor(x, y);
    if (color > -1)
    {
        cube_gfx_setTextColor((uint16_t) color);
    }
    //INFO("String len: %d", strlen(string));
    /*
    char c = string;
    while(*c != '\0') {
        for(int i = 0; i < strlen(string); i++) {
            cube_gfx_write(*c);
            c++;
        }
    }
    */
    const char* c = string;
    while (*c) cube_gfx_write(*c++);
}
/**************************************************************************/
/*!
    @brief   Set text 'magnification' size. Each increase in s makes 1 pixel
   that much bigger.
    @param  s  Desired text size. 1 is default 6x8, 2 is 12x16, 3 is 18x24, etc
*/
/**************************************************************************/
void cube_gfx_setTextSize(uint8_t s)
{
    setTextSize(s, s);
}

/**************************************************************************/
/*!
    @brief   Sets frame time for rendering. It affect how quickyl a new frame
    appears on the display.
    @param  millisec  Period in milliseconds for the frame to repeat.
*/
/**************************************************************************/
void cube_gfx_setFrameTime(unsigned int millisec)
{
    leddisplay_panels_set_frametime(millisec);
}

/**************************************************************************/
/*!
    @brief   Returns current frame time setting.for rendering.
    @return  frame time in milliseconds
*/
/**************************************************************************/
unsigned int cube_gfx_getFrameTime(void)
{
    return leddisplay_panels_get_frametime();
}
/**************************************************************************/
/*!
    @brief   Set text 'magnification' size. Each increase in s makes 1 pixel
   that much bigger.
    @param  s_x  Desired text width magnification level in X-axis. 1 is default
    @param  s_y  Desired text width magnification level in Y-axis. 1 is default
*/
/**************************************************************************/
void setTextSize(uint8_t s_x, uint8_t s_y)
{
    textsize_x = (s_x > 0) ? s_x : 1;
    textsize_y = (s_y > 0) ? s_y : 1;
}

/**************************************************************************/
/*!
    @brief      Set rotation setting for display
    @param  x   0 thru 3 corresponding to 4 cardinal rotations
*/
/**************************************************************************/
void cube_gfx_setRotation(uint8_t x)
{
    rotation = (x & 3);
    switch (rotation)
    {
    case 0:
    case 2:
        _width = WIDTH;
        _height = HEIGHT;
        break;
    case 1:
    case 3:
        _width = HEIGHT;
        _height = WIDTH;
        break;
    }
}

/**************************************************************************/
/*!
    @brief Set the font to display when print()ing, either custom or default
    @param  f  The GFXfont object, if NULL use built in 6x8 font
*/
/**************************************************************************/
void cube_gfx_setFont(const GFXfont *f)
{
    if (f)
    { // Font struct pointer passed in?
        if (!gfxFont)
        { // And no current font struct?
            // Switching from classic to new font behavior.
            // Move cursor pos down 6 pixels so it's on baseline.
            cursor_y += 6;
        }
    }
    else if (gfxFont)
    { // NULL passed.  Current font struct defined?
        // Switching from new to classic font behavior.
        // Move cursor pos up 6 pixels so it's at top-left of char.
        cursor_y -= 6;
    }
    gfxFont = (GFXfont *)f;
}

/**************************************************************************/
/*!
    @brief    Helper to determine size of a character with current font/size.
       Broke this out as it's used by both the PROGMEM- and RAM-resident
   getTextBounds() functions.
    @param    c     The ascii character in question
    @param    x     Pointer to x location of character
    @param    y     Pointer to y location of character
    @param    minx  Minimum clipping value for X
    @param    miny  Minimum clipping value for Y
    @param    maxx  Maximum clipping value for X
    @param    maxy  Maximum clipping value for Y
*/
/**************************************************************************/
void cube_gfx_charBounds(unsigned char c, int16_t *x, int16_t *y, int16_t *minx, int16_t *miny, int16_t *maxx, int16_t *maxy)
{
    if (gfxFont)
    {

        if (c == '\n')
        {           // Newline?
            *x = 0; // Reset x to zero, advance y by one line
            *y += textsize_y * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
        }
        else if (c != '\r')
        { // Not a carriage return; is normal char
            uint8_t first = pgm_read_byte(&gfxFont->first),
                    last = pgm_read_byte(&gfxFont->last);
            if ((c >= first) && (c <= last))
            { // Char present in this font?
                GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c - first);
                uint8_t gw = pgm_read_byte(&glyph->width),
                        gh = pgm_read_byte(&glyph->height),
                        xa = pgm_read_byte(&glyph->xAdvance);
                int8_t xo = pgm_read_byte(&glyph->xOffset),
                       yo = pgm_read_byte(&glyph->yOffset);
                if (wrap && ((*x + (((int16_t)xo + gw) * textsize_x)) > _width))
                {
                    *x = 0; // Reset x to zero, advance y by one line
                    *y += textsize_y * (uint8_t)pgm_read_byte(&gfxFont->yAdvance);
                }
                int16_t tsx = (int16_t)textsize_x, tsy = (int16_t)textsize_y,
                        x1 = *x + xo * tsx, y1 = *y + yo * tsy, x2 = x1 + gw * tsx - 1,
                        y2 = y1 + gh * tsy - 1;
                if (x1 < *minx)
                    *minx = x1;
                if (y1 < *miny)
                    *miny = y1;
                if (x2 > *maxx)
                    *maxx = x2;
                if (y2 > *maxy)
                    *maxy = y2;
                *x += xa * tsx;
            }
        }
    }
    else
    { // Default font

        if (c == '\n')
        {                         // Newline?
            *x = 0;               // Reset x to zero,
            *y += textsize_y * 8; // advance y one line
                                  // min/max x/y unchaged -- that waits for next 'normal' character
        }
        else if (c != '\r')
        { // Normal char; ignore carriage returns
            if (wrap && ((*x + textsize_x * 6) > _width))
            {                         // Off right?
                *x = 0;               // Reset x to zero,
                *y += textsize_y * 8; // advance y one line
            }
            int x2 = *x + textsize_x * 6 - 1, // Lower-right pixel of char
                y2 = *y + textsize_y * 8 - 1;
            if (x2 > *maxx)
                *maxx = x2; // Track max x, y
            if (y2 > *maxy)
                *maxy = y2;
            if (*x < *minx)
                *minx = *x; // Track min x, y
            if (*y < *miny)
                *miny = *y;
            *x += textsize_x * 6; // Advance x one char
        }
    }
}
/*
uint16_t cube_gfx_getCharHeight(unsigned char c, int16_t x, int16_t y)
{
    uint8_t gh, gw = 0;
    int16_t ya, xa, xo, yo = 0;
    if (gfxFont)
    {
        ya = gfxfont.yAdvance;
        if (c == '\n')
        {                         // Newline?
            x = 0;               // Reset x to zero, used only in internal calculations
            y += textsize_y * ya; // advance y one line, used only in internal calculations
                                  // min/max x/y unchaged -- that waits for next 'normal' character
            gh, gw = 0;
        }
        else if (c != '\r')
        { // Not a carriage return; is normal char
            uint8_t first = gfxFont.first;
            uint8_t last = gfxFont.last;
            if ((c >= first) && (c <= last))
            { // Char present in this font?
                GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c - first);
                //gw = glyph.width;
                gh = glyph.height;
                //xa = glyph.xAdvance;
                //xo = glyph.xOffset;
                //yo = glyph.yOffset;
                
                if (wrap && ((x + ((xo + gw) * textsize_x)) > _width))
                {
                    x = 0; // Reset x to zero, advance y by one line
                    y += textsize_y * gfxFont.yAdvance;
                }
            }
            return gh;
        }
    }
    return NULL;
}

uint16_t cube_gfx_getCharWidth(unsigned char c, int16_t x, int16_t y)
{
    uint8_t gh, gw = 0;
    int16_t ya, xa, xo, yo = 0;
    if (gfxFont)
    {
        ya = gfxfont.yAdvance;
        if (c == '\n')
        {                         // Newline?
            x = 0;               // Reset x to zero, used only in internal calculations
            y += textsize_y * ya; // advance y one line, used only in internal calculations
                                  // min/max x/y unchaged -- that waits for next 'normal' character
            gh, gw = 0;
        }
        else if (c != '\r')
        { // Not a carriage return; is normal char
            uint8_t first = gfxFont.first;
            uint8_t last = gfxFont.last;
            if ((c >= first) && (c <= last))
            { // Char present in this font?
                GFXglyph *glyph = pgm_read_glyph_ptr(gfxFont, c - first);
                gw = glyph.width;
                //gh = glyph.height;
                //xa = glyph.xAdvance;
                //xo = glyph.xOffset;
                //yo = glyph.yOffset;
                
                if (wrap && ((x + ((xo + gw) * textsize_x)) > _width))
                {
                    x = 0; // Reset x to zero, advance y by one line
                    y += textsize_y * gfxFont.yAdvance;
                }
            }
            return gh;
        }
    }
    return NULL;
}
*/
/**************************************************************************/
/*!
    @brief    Helper to determine size of a string with current font/size. Pass
   string and a cursor position, returns UL corner and W,H.
    @param    str     The ascii string to measure
    @param    x       The current cursor X
    @param    y       The current cursor Y
    @param    x1      The boundary X coordinate, set by function
    @param    y1      The boundary Y coordinate, set by function
    @param    w      The boundary width, set by function
    @param    h      The boundary height, set by function
*/
/**************************************************************************/

void cube_gfx_getTextBounds(const char *str, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h)
{
    uint8_t c; // Current character

    *x1 = x;
    *y1 = y;
    *w = *h = 0;

    int16_t minx = _width, miny = _height, maxx = -1, maxy = -1;

    while ((c = *str++)) {
        cube_gfx_charBounds(c, &x, &y, &minx, &miny, &maxx, &maxy);
    }
    if (maxx >= minx)
    {
        *x1 = minx;
        *w = maxx - minx + 1;
    }
    if (maxy >= miny)
    {
        *y1 = miny;
        *h = maxy - miny + 1;
    }
}

/**************************************************************************/
/*!
    @brief    Helper to determine size of a string with current font/size. Pass
   string and a cursor position, returns UL corner and W,H.
    @param    str     The ascii string to measure
    @param    x       X cursor position from where the calculation is referenced
    @param    y       Y cursor position from where the calculation is referenced

    @return   Height of string text.
*/
/**************************************************************************/
/*
uint16_t cube_gfx_getTextHeight(const char *str, int16_t x, int16_t y)
{
    uint8_t c; // Current character
    uint8_t gh, gw = 0;
    int16_t ya, xa, xo, yo = 0;
    int16_t x1 = x;
    int16_t y1 = y;
    uint16_t tw, th = 0;

    int16_t minx = _width, miny = _height, maxx = -1, maxy = -1;

    while ((c = *str++)) {
        cube_gfx_charBounds(c, &x, &y, &minx, &miny, &maxx, &maxy);
    }
    if (maxx >= minx)
    {
        x1 = minx;
        tw = maxx - minx + 1;
    }
    if (maxy >= miny)
    {
        y1 = miny;
        th = maxy - miny + 1;
    }
    return th;
}
*/
/**************************************************************************/
/*!
    @brief    Helper to determine size of a string with current font/size. Pass
   string and a cursor position, returns UL corner and W,H.
    @param    str     The ascii string to measure
    @param    x       X cursor position from where the calculation is referenced
    @param    y       Y cursor position from where the calculation is referenced

    @return   Width of string text.
*/
/**************************************************************************/
/*
uint16_t cube_gfx_getTextWidth(const char *str, int16_t x, int16_t y)
{
    uint8_t c; // Current character
    uint8_t gh, gw = 0;
    int16_t ya, xa, xo, yo = 0;
    int16_t x1 = x;
    int16_t y1 = y;
    uint16_t tw, th = 0;

    int16_t minx = _width, miny = _height, maxx = -1, maxy = -1;

    while ((c = *str++)) {
        cube_gfx_charBounds(c, &x, &y, &minx, &miny, &maxx, &maxy);
    }
    if (maxx >= minx)
    {
        x1 = minx;
        tw = maxx - minx + 1;
    }
    if (maxy >= miny)
    {
        y1 = miny;
        th = maxy - miny + 1;
    }
    return tw;
}
*/
/**************************************************************************/
/*!
    @brief    Helper to determine size of a string with current font/size. Pass
   string and a cursor position, returns UL corner and W,H.
    @param    str     The ascii string to measure
    @param    x       X cursor position from where the calculation is referenced
    @param    y       Y cursor position from where the calculation is referenced

    @return   X coordinate of the text boundary from which height and width is referenced.
*/
/**************************************************************************/
/*
uint16_t cube_gfx_getTextCornerX(const char *str, int16_t x, int16_t y)
{
    uint8_t c; // Current character
    uint8_t gh, gw = 0;
    int16_t ya, xa, xo, yo = 0;
    int16_t x1 = x;
    int16_t y1 = y;
    uint16_t tw, th = 0;

    int16_t minx = _width, miny = _height, maxx = -1, maxy = -1;

    while ((c = *str++)) {
        cube_gfx_charBounds(c, &x, &y, &minx, &miny, &maxx, &maxy);
    }
    if (maxx >= minx)
    {
        x1 = minx;
        tw = maxx - minx + 1;
    }
    if (maxy >= miny)
    {
        y1 = miny;
        th = maxy - miny + 1;
    }
    return x1;
}
*/
/**************************************************************************/
/*!
    @brief    Helper to determine size of a string with current font/size. Pass
   string and a cursor position, returns UL corner and W,H.
    @param    str     The ascii string to measure
    @param    x       X cursor position from where the calculation is referenced
    @param    y       Y cursor position from where the calculation is referenced

    @return   Y coordinate of the text boundary from which height and width is referenced.
*/
/**************************************************************************/
/*
uint16_t cube_gfx_getTextCornerY(const char *str, int16_t x, int16_t y)
{
    uint8_t c; // Current character
    uint8_t gh, gw = 0;
    int16_t ya, xa, xo, yo = 0;
    int16_t x1 = x;
    int16_t y1 = y;
    uint16_t tw, th = 0;

    int16_t minx = _width, miny = _height, maxx = -1, maxy = -1;

    while ((c = *str++)) {
        cube_gfx_charBounds(c, &x, &y, &minx, &miny, &maxx, &maxy);
    }
    if (maxx >= minx)
    {
        x1 = minx;
        tw = maxx - minx + 1;
    }
    if (maxy >= miny)
    {
        y1 = miny;
        th = maxy - miny + 1;
    }
    return y1;
}
*/
/**********************************************************************/
/*!
    @brief  Set text cursor location
    @param  x    X coordinate in pixels
    @param  y    Y coordinate in pixels
  */
/**********************************************************************/
void cube_gfx_setCursor(int16_t x, int16_t y)
{
    cursor_x = x;
    cursor_y = y;
}
/**********************************************************************/
/*!
    @brief   Set text font color with transparant background
    @param   c   16-bit 5-6-5 Color to draw text with
    @note    For 'transparent' background, background and foreground
             are set to same color rather than using a separate flag.
  */
/**********************************************************************/
void cube_gfx_setTextColor(uint16_t c)
{
    textcolor = textbgcolor = c;
}

/**********************************************************************/
/*!
    @brief   Set text font color with custom background color
    @param   c   16-bit 5-6-5 Color to draw text with
    @param   bg  16-bit 5-6-5 Color to draw background/fill with
  */
/**********************************************************************/
void cube_gfx_setTextColor_bg(uint16_t c, uint16_t bg)
{
    textcolor = c;
    textbgcolor = bg;
}

/**********************************************************************/
/*!
  @brief  Set whether text that is too long for the screen width should
          automatically wrap around to the next line (else clip right).
  @param  w  true for wrapping, false for clipping
  */
/**********************************************************************/
void cube_gfx_setTextWrap(bool w)
{
    wrap = w;
}

/************************************************************************/
/*!
    @brief      Get width of the display, accounting for current rotation
    @returns    Width in pixels
  */
/************************************************************************/
int16_t cube_gfx_width(void)
{
    return _width;
}

/************************************************************************/
/*!
    @brief      Get height of the display, accounting for current rotation
    @returns    Height in pixels
  */
/************************************************************************/
int16_t cube_gfx_height(void)
{
    return _height;
}

/************************************************************************/
/*!
    @brief      Get rotation setting for display
    @returns    0 thru 3 corresponding to 4 cardinal rotations
  */
/************************************************************************/
uint8_t cube_gfx_getRotation(void)
{
    return rotation;
}

// get current cursor position (get rotation safe maximum values,
// using: width() for x, height() for y)
/************************************************************************/
/*!
    @brief  Get text cursor X location
    @returns    X coordinate in pixels
  */
/************************************************************************/
int16_t cube_gfx_getCursorX(void)
{
    return cursor_x;
}

/************************************************************************/
/*!
    @brief      Get text cursor Y location
    @returns    Y coordinate in pixels
  */
/************************************************************************/
int16_t cube_gfx_getCursorY(void)
{
    return cursor_y;
}
/**********************************************************************/
/*! TODO
        @brief    Get the pixel color value at a given coordinate
        @param    x   x coordinate
        @param    y   y coordinate
        @returns  The desired pixel's 16-bit 5-6-5 color value
*/
/**********************************************************************/
/*
uint16_t cube_gfx_dispbuf_getPixel(int16_t x, int16_t y) const {
  int16_t t;
  switch (rotation) {
  case 1:
    t = x;
    x = WIDTH - 1 - y;
    y = t;
    break;
  case 2:
    x = WIDTH - 1 - x;
    y = HEIGHT - 1 - y;
    break;
  case 3:
    t = x;
    x = y;
    y = HEIGHT - 1 - t;
    break;
  }
  return getRawPixel(x, y);
}
*/