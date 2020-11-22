/*!
    \file
    \brief HUB75 LED display driver using I2S parallel mode (see \ref LEDDISPLAY)

    \defgroup LEDDISPLAY LEDDISPLAY

    Based on:
    - https://esp32.com/viewtopic.php?t=3188
    - https://github.com/ESP32DE/I2S_parallel_example_drive_a_64x32_display
    - https://github.com/pixelmatix/esp32_I2sParallelDmaLedMatrix
    - https://github.com/pixelmatix/SmartMatrix/tree/teensylc
    - https://github.com/mrfaptastic/ESP32-RGB64x32MatrixPanel-I2S-DMA

    Copyright:

    - Copyright 2017 Espressif Systems (Shanghai) PTE LTD
    - Copyright 2018 Louis Beaudoin (Pixelmatix)
    - Copyright 2019 Philippe Kehl (flipflip at oinkzwurgl dot org)

    Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except
    in compliance with the License.  You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software distributed under the
    License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
    express or implied.  See the License for the specific language governing permissions and
    limitations under the License.

    See src/leddisplay.c for details.

    @{
*/

#ifndef __LEDDISPLAY_H__
#define __LEDDISPLAY_H__

#include <stdint.h>
#include <sdkconfig.h>
#include <esp_err.h>
#include <freertos/semphr.h>

// configuration (see also leddisplay.c)
#if CONFIG_LEDDISPLAY_TYPE_32X16_4SCAN || CONFIG_LEDDISPLAY_TYPE_32X16_8SCAN
#  define LEDDISPLAY_WIDTH                32
#  define LEDDISPLAY_HEIGHT               16

#elif CONFIG_LEDDISPLAY_TYPE_32X32_8SCAN || CONFIG_LEDDISPLAY_TYPE_32X32_16SCAN
#  define LEDDISPLAY_WIDTH                32
#  define LEDDISPLAY_HEIGHT               32

#elif CONFIG_LEDDISPLAY_TYPE_64X32_8SCAN || CONFIG_LEDDISPLAY_TYPE_64X32_16SCAN
#  define LEDDISPLAY_WIDTH                64
#  define LEDDISPLAY_HEIGHT               32

#elif CONFIG_LEDDISPLAY_TYPE_64X64_32SCAN
#  define LEDDISPLAY_WIDTH                64
#  define LEDDISPLAY_HEIGHT               64

#elif CONFIG_LEDDISPLAY_TYPE_32X32x6_16SCAN
#  define LEDDISPLAY_WIDTH                32
#  define LEDDISPLAY_HEIGHT               32
#  define NUMBER_OF_PANELS				  6

#elif CONFIG_LEDDISPLAY_TYPE_64X32x3_16SCAN
#  define LEDDISPLAY_WIDTH                64
#  define LEDDISPLAY_HEIGHT               32
#  define NUMBER_OF_PANELS				  3

#elif CONFIG_LEDDISPLAY_TYPE_192x32_16SCAN
#  define LEDDISPLAY_WIDTH                192
#  define LEDDISPLAY_HEIGHT               32
#  define NUMBER_OF_PANELS				  1

#else
#  error This CONFIG_LEDDISPLAY_TYPE is not implemented!
#endif

#define ROWS_PER_FRAME            (LEDDISPLAY_HEIGHT / LEDDISPLAY_ROWS_IN_PARALLEL)

/* *********************************************************************************************** */
/*!
    \name display functions

    Example:

\code{.c}
    #include <leddisplay.h>
    leddisplay_init();
\endcode

    @{
*/

//! initialise the LED display
/*!
    Call this before calling any other leddisplay_*() function.

    \returns #ESP_OK on success, or on error: #ESP_ERR_NO_MEM, #ESP_FAIL
*/
esp_err_t leddisplay_init(void);

//! shutdown the LED display
void leddisplay_shutdown(void);

//! set global brightness level
/*
    \param[in] brightness  global brightness level, range 0..100 [%]
    \returns the previously set global brightness level
*/
int leddisplay_set_brightness(int brightness);

//! get global brightness levell
/*!
    \returns the currently set global brightness level (0..100 [%])
*/
int leddisplay_get_brightness(void);

//@}

/* *********************************************************************************************** */
/*!
    \name pixel based functions

    These functions operate directly on the internal buffers, which is relatively expensive on CPU
    usage. At 160MHz CPU speed it takes about 20ms to set all pixels on a 64x32 display.

    Example:

\code{.c}
    #include <leddisplay.h>
    leddisplay_init();
    leddisplay_pixel_fill_rgb(0, 0, 0);        // fill black, i.e. clear frame
    leddisplay_pixel_fill_rgb(255, 0, 0);      // fill red at full brightness
    leddisplay_pixel_xy_rgb(10, 5, 0, 255, 0); // set green pixel at x=10 / y=5
    leddisplay_pixel_update();                 // display
\endcode

    @{
*/

//! set pixel to colour
/*!
    Top-left of display is x=0 / y=0.

    \param[in] x_coord  x coordinate
    \param[in] y_coord  y coordinate
    \param[in] red      red value
    \param[in] green    green value
    \param[in] blue     blue value
*/
void leddisplay_pixel_xy_rgb(uint16_t x_coord, uint16_t y_coord, uint8_t red, uint8_t green, uint8_t blue);


//! fill all pixels with colour
/*!
    \param[in] red    red value
    \param[in] green  green value
    \param[in] blue   blue value
*/
void leddisplay_pixel_fill_rgb(uint8_t red, uint8_t green, uint8_t blue);

//! update display with current frame
/*!
    Flushes the frame to the display.

    \param[in] block   waits for framebuffer to become available again if non-zero

    \note After this function returns the frame memory will be invalid and you will have to draw the
          next frame fully from scratch.
*/
void leddisplay_pixel_update(int block);

void leddisplay_pixel_update_2(void);

//@}

/* *********************************************************************************************** */
/*!
    \name frame based functions

    These functions operate on a user supplied buffer and are more CPU efficient at the cost of
    additional memory. At 160MHz CPU speed it takes about 8ms to process a 64x32 frame into the DMA
    buffers.

    Example:

\code{.c}
    #include <leddisplay.h>
    leddisplay_init();
    static leddisplay_frame_t frame;
    leddisplay_frame_clear(&frame);            // clear frame (i.e. fill black)
    leddisplay_frame_fill_rgb(255, 0, 0);      // fill red at full brightness
    leddisplay_frame_xy_rgb(10, 5, 0, 255, 0); // set green pixel at x=10 / y=5
    leddisplay_frame_update(&frame);           // display
\endcode

    @{
*/	

//! frame type
typedef union leddisplay_frame_u
{
    //! access RGB pixel by coordinates
    uint8_t yx[LEDDISPLAY_HEIGHT][LEDDISPLAY_WIDTH][3];
    //! access RGB pixel by index
    uint8_t ix[LEDDISPLAY_HEIGHT * LEDDISPLAY_WIDTH][3];
    //! raw frame data
    uint8_t raw[LEDDISPLAY_HEIGHT * LEDDISPLAY_WIDTH * 3];
} leddisplay_frame_t;


#ifdef NUMBER_OF_PANELS

//! initialise the LED display with MultiPanel setup
/*!
    Call this before calling any other leddisplay_*() function.

    \returns #ESP_OK on success, or on error: #ESP_ERR_NO_MEM, #ESP_FAIL
*/
esp_err_t leddisplay_init_multipanel(void);



#ifdef CONFIG_LEDDISPLAY_PWR_EN_GPIO
/* Power enable API
*   I recommend using a P-type Mosfet, like the Si2319CD, which is a SOT-23 package, with 4,4A max current
*/
void leddisplay_init_power_en_gpio(void);
void leddisplay_set_power_state(uint32_t state);
uint32_t leddisplay_get_power_state(void);
#endif //#ifdef CONFIG_LEDDISPLAY_PWR_EN_GPIO



/*! initialise the LED display with MultiPanel Display Buffer given by "bufferindex"
These Diplay buffer will be used to write pixel data to, from which
the matrix driver will convert into DMA data to be output to the matrix panels

    Call this before calling any other leddisplay_panels_*() function.
    \#bufferindex   0 or 1

    \returns #ESP_OK on success, or on error: #ESP_ERR_NO_MEM, or ESP_FAIL
*/
esp_err_t leddisplay_displaybuffer_init(uint8_t bufferindex);

/*! Sets diplay buffer select dispbufindex ready value to 1"

    Call this before calling any other leddisplay_panels_*() function.
    \#bufferindex   0 or 1
*/
//void leddisplay_panels_set_dispbuf_ready(unsigned int dispbufindex);

/*! Clears diplay buffer select dispbufindex ready value to 1"

    Call this before calling any other leddisplay_panels_*() function.
    \#bufferindex   0 or 1

    \returns 0 (OK) or -1(ERROR);
*/
//void leddisplay_panels_clear_dispbuf_ready(unsigned int dispbufindex);

/*! Get diplay buffer ready value selected by dispbufindex"

    Call this before calling any other leddisplay_panels_*() function.
    \#bufferindex   0 or 1

    \returns 0 (not ready) or 1(ready);
*/
//int leddisplay_panels_get_dispbuf_ready(unsigned int dispbufindex);


/*! Get last rendered displaybuffer index

    Call this before calling any other leddisplay_panels_*() function.
    \#bufferindex   0 or 1

    \returns 0 (not ready) or 1(ready);
*/
int leddisplay_panels_get_next_to_render(void);

void leddisplay_panels_set_lastrendered(unsigned int dispbufindex);

int leddisplay_panels_get_lastrendered(void);

int leddisplay_panels_get_buffer_status(unsigned int dispbufindex);

void leddisplay_panels_set_buffer_status(unsigned int dispbufindex, uint8_t status);

  
/*! Gets the index of the Diplay buffer, which can be used.
  Inside the function it just checks semaphore availability for the diplay buffer,
  by taking it, and BLOCKING the indexed display buffer.
  
  @warning CALLING leddisplay_panels_block_dispbuf() without calling
  leddisplay_panels_unblock() on the returned index will return an ERROR.
  Because this function also blocks the displaybuffer with returned index,
  displaybuffer can be manupulated right away.
  
  We are using a frontbuffer(a) and backbuffer(b) method.
  By default, display buffer(a) is used. If it is blocked, (b) will be used.

    Call this before calling any other leddisplay_panels_*() function.

    \returns Index of unblocked Display buffer: 0(a) or 1(b), -1 on ERROR

*/
int leddisplay_panels_get_unblocked_dispbuf_index(void);

/*! Unblocks display buffer, with the "index" argument given
    by default, index "0" or "1" should return OK
    Internally function releases(gives) the Semaphores assigned to
    the indexed display buffer 

    \returns #ESP_OK on success, or on error: #ESP_FAIL,
*/
esp_err_t leddisplay_panels_unblock_dispbuf(unsigned int index);

/*! Blocks display buffer, with the "index" argument given
    by default, index "0" or "1" should return OK
    Internally function takes the Semaphores assigned to
    the indexed diplay buffer

    \returns #ESP_OK on success, or on error: #ESP_FAIL
    \ THis function will wait forever to for the buffer to be unblocked. Wll never return ESP_FAIL if index is correct.
*/
esp_err_t leddisplay_panels_block_dispbuf(unsigned int index);

/*! Blocks display buffer, with the "index" argument given.
    It waits for "ms" time for the buffer to be available
    by default, index "0" or "1" should return OK
    Internally function takes the Semaphores assigned to
    the indexed diplay buffer, waits fo

    \param[in] index -  display buffer index, 0 (a) or 1(b)
    \param[in] ms -     time in milliseconds, to wait for the buffer to be blockable

    \returns #ESP_OK on success, or on error: #ESP_FAIL
    \ If ms time passes and buffer can not be blocked - in use by something else - will return ESP_FAIL
*/
esp_err_t leddisplay_panels_block_MS_dispbuf(unsigned int index, unsigned int ms);

/*! Sets global orientation of the panels.
    Implemented:
        0     Horizontal
        1     Vertical
    
    Since we have more panels, we need to know how these panels are put next to each other.
    Value 0 = Horizontal (default)
        Max X coord: (NUMBER_OF_PANELS x LEDDISPLAY_WIDTH)-1
    y  +-----+-----+---
    |  |  1  |  2  | . . .
    v  +-----+-----+---
    Max Y coord: LEDDISPLAY_HEIGHT-1

    Value 1 = Vertical
            Max X coord: LEDDISPLAY_WIDTH-1
    y +-----+
    | |  1  |
    v +-----+
      |  2  |
      +-----+
      |  :  |
    Max Y coord: (NUMBER_OF_PANELS x LEDDISPLAY_HEIGHT)-1

    Value 2 = T-Shape - NOT YET INPLEMENTED
        Max X coord: (LEDDISPLAY_WIDTH x 3)-1
    y       
    |      
    v +-----+-----+-----+
      |  1  |  2  |  3  |
      +-----+-----+-----+
            |  4  |
            +-----+
            |  5  |
            +-----+
            |  6  |
            +-----+
    Max Y coord: (LEDDISPLAY_HEIGHT x 4)-1

    Value 3 = 2rows - NOT YET INPLEMENTED
    0 x->                 Max X coord: (LEDDISPLAY_WIDTH x 3)-1
    y +-----+-----+-----+
    | |  1  |  2  |  3  |
    v +-----+-----+-----+
      |  4  |  5  |  6  |
      +-----+-----+-----+
    Max Y coord: (LEDDISPLAY_HEIGHT x 2)-1
    
*/
void leddisplay_panels_set_orientation(uint8_t orientation);

/*! Gets global orientation value
    see leddisplay_panels_set_orientation)

    \returns #value of orientation value
*/
int leddisplay_panels_get_orientation(void);

/*! Update display. Converts diplaybuffer data to LED MAtrix DMA data, and pushed it out the display
    
    \param[in]  panelnum is the number of panels to display, or the actual number that should be updated.
                panelnum is used directly for indexing the individual frame-buffers of the panels
                    panelnum = 0 -> Update all panels
                    panelnum = 1- NUMBER_OF_PANELS -> Update only the panel designated by the number
                    panelnum = NUMBER_OF_PANELS -255 -> At the moment this updates ALL panels, can be used for future implementations
*/
int leddisplay_panels_update(int dispbufindex, uint8_t panelnum);


/*! Clears displaybuffer. Sets every pixel to 0,0,0.
    THIS FUNCTION DOES NOT UPDATE THE DISPLAYED IMAGE!

    \param[in,out] dispbufindex -  display buffer index, 0 (a) or 1(b)
                                @note: This function checks if indexed display buffer is unlocked.

    \param[in]  panelnum is the number of panels to display, or the actual number that should be cleared
                panelnum is used directly for indexing the individual frame-buffers of the panels
                    panelnum = 0 -> Update all panels
                    panelnum = 1- NUMBER_OF_PANELS -> Update only the panel designated by the number
                    panelnum = NUMBER_OF_PANELS -255 -> At the moment this updates ALL panels, can be used for future implementations
*/
int leddisplay_panels_clear(int dispbufindex, uint8_t panelnum);

int leddisplay_panels_fill_random(int dispbufindex, uint8_t panelnum);

/*! Set pixel to colour on a panel diplay buffer

    !!! BEFORE USING THIS FUNCTION, DIPSLAY BUFFER AT dispbufindex, MUST BE BLOCKED BY leddisplay_panels_block_dispbuf()

    These functions automatically handles panel transitions, and panel orientation when mass amount of pixels are being written
    Top-left of display panel is x=0 / y=0.
    Function checks global orientation value (static uint8_t s_panel_orientation)
    \param[in,out] dispbufindex -  display buffer index, 0 (a) or 1(b)
    \param[in]     x_coord  x coordinate
    \param[in]     y_coord  y coordinate
    \param[in]     startpanel
                    0: Begins from Panel 1
                    1->NUMBER_OF_PANELS: shifts coordinate field by number of panels given here
                    If coordinate given is bigger than the coordinate field of a single panel, then it will write to the pixel on the other panels,
                    according to the global orientation.
    \param[in]     red      red value
    \param[in]     green    green value
    \param[in]     blue     blue value

    \returns
            0   = OK
            1   = startindex incorrect
            2   = Panelindex internally exceeded the NUMBER_OF_PANELS number
            3   = X coordinate is too large
            4   = Y coordinate is too large
*/
int leddisplay_panels_xy_rgb(int dispbufindex, uint8_t startpanel, uint16_t x_coord, uint16_t y_coord, uint8_t red, uint8_t green, uint8_t blue);

/*! Fills selected display buffer, with given color
    Function checks global orientation value (static uint8_t s_panel_orientation)
    
    !!! BEFORE USING THIS FUNCTION, DIPSLAY BUFFER AT dispbufindex, MUST BE BLOCKED BY leddisplay_panels_block_dispbuf()

    \param[in,out] dispbufindex -  display buffer index, 0 (a) or 1(b)
    \param[in]     x_coord  x coordinate
    \param[in]     y_coord  y coordinate
    \param[in]     startpanel
                    0: Fills each of the panel buffers for the referenced dips. buffer
                    1->NUMBER_OF_PANELS: Fills only selected panel. Panel numberings start at 1, not 0.
    \param[in]     red      red value
    \param[in]     green    green value
    \param[in]     blue     blue value

    \returns
            0   = OK
            1   = startindex incorrect
            2   = Panelindex internally exceeded the NUMBER_OF_PANELS number
*/
int leddisplay_panels_fill_rgb(int dispbufindex, uint8_t startpanel, uint8_t red, uint8_t green, uint8_t blue);

/*! Set pixel to colour on a panel diplay buffer, with alpha value

    !!! BEFORE USING THIS FUNCTION, DIPSLAY BUFFER AT dispbufindex, MUST BE BLOCKED BY leddisplay_panels_block_dispbuf()

    These functions automatically handles panel transitions, and panel orientation when mass amount of pixels are being written
    Top-left of display panel is x=0 / y=0.
    Function checks global orientation value (static uint8_t s_panel_orientation)

    Internally this function handles ARGB color mixing in a simplified - yet fast - method.
    Function reads pixel value at given x,y coordinates, and mixes the red, green, blue values passed, according to the alpha.

    \param[in,out] dispbufindex -  display buffer index, 0 (a) or 1(b)
    \param[in]     x_coord  x coordinate
    \param[in]     y_coord  y coordinate
    \param[in]     startpanel
                    0: Begins from Panel 1
                    1->NUMBER_OF_PANELS: starts from indexed panel (panel indexes don't start from 0)
                    xy coordinates are then referenced from the 0 point of the indexed panel.
                    If coordinate given is bigger than the coordinate field of the panel, then it will write to the pixel on the other panels,
                    according to the global orientation.
    \param[in]     red      red value
    \param[in]     green    green value
    \param[in]     blue     blue value

    \returns
            0   = OK
            1   = startpanel - index incorrect
            2   = Panelindex (value internal to the function) internally exceeded the NUMBER_OF_PANELS number
            3   = X coordinate is too large
            4   = Y coordinate is too large
*/
int leddisplay_panels_xy_argb(int dispbufindex, uint8_t startpanel, uint16_t x_coord, uint16_t y_coord, uint8_t alpha, uint8_t red, uint8_t green, uint8_t blue);

//! gets pixel colour from xy coordinates given
/*!
    Top-left of display is x=0 / y=0.

    \param[in,out] p_frame  pointer to frame memory
    \param[in]     x_coord  x coordinate
    \param[in]     y_coord  y coordinate
    \param[in]     red      pointer to the red value
    \param[in]     green    pointer to the green value
    \param[in]     blue     pointer to the blue value
*/
void leddisplay_panels_get_rgb_xy(int dispbufindex, uint16_t x_coord, uint16_t y_coord, uint8_t *red, uint8_t *green, uint8_t *blue);

/*! Auto renders displaybuffers, at a rate given in period parameter. ONLY TO BE Used for xTaskCreatePinnedToCore() for core 1
    @note:  function may not run as quickly as given in period, if the Leddisplay components settings allows a slower execution.
            The matrix driver will wait for I2S interface to complete the shifting, which can cause slighly slower running.

    \param[in]  interval - Render interval in ms

*/
void DisplayRender_FPS(unsigned int FPS);

/*! Auto renders displaybuffers. ONLY TO BE Used for xTaskCreatePinnedToCore() for core 1
    Runs as fast as possible.

*/
void DisplayRender_Cont(void);

void leddisplay_panels_set_frametime(unsigned int millisec);
unsigned int leddisplay_panels_get_frametime(void);

void DisplayRender_period(void);
/*! Gets next available - or soon to be available buffer, blocks the buffer, or returns error.
    This function also sets the status of the take Displaybuffer to "Work in progress", to signal
    the rendering driver, that this is being worked on.
    You must call a DisplayBufferFinish(int dispbufindex) after done writing to the buffer
    to release it for rendering.

    \returns    0(a) or 1(b) as buffer index
                -1 : ERROR
*/
int DisplayBufferTake(void);

void Pause_DisplayRender(void);

void Resume_DisplayRender(void);

int DisplayBufferSetNextTo(int dispbufindex);
void DisplayBufferQclear(void);
int getNextToRender(void);

/*! Releases the Displaybuffer taken by DisplayBufferTake() and marks it's status "Ready to render".
    If you finished using the buffer, but some other function will continue working on it, or a different task,
    simply call leddisplay_panels_ublock_dispbuf(<indextaken>) so the status is not set.

    \returns    ESP_OK  - OK
                ESP_FAIL = ERROR. Internally the output of a leddisplay_panels_unblock_dispbuf();
*/
esp_err_t DisplayBufferFinish(int dispbufindex);

#endif

//! set pixel to colour
/*!
    Top-left of display is x=0 / y=0.

    \param[in,out] p_frame  pointer to frame memory
    \param[in]     x_coord  x coordinate
    \param[in]     y_coord  y coordinate
    \param[in]     red      red value
    \param[in]     green    green value
    \param[in]     blue     blue value
*/
void leddisplay_frame_xy_rgb(leddisplay_frame_t *p_frame, uint16_t x_coord, uint16_t y_coord, uint8_t red, uint8_t green, uint8_t blue);


//! gets pixel colour from xy coordinates given
/*!
    Top-left of display is x=0 / y=0.

    \param[in,out] p_frame  pointer to frame memory
    \param[in]     x_coord  x coordinate
    \param[in]     y_coord  y coordinate
    \param[in]     red      pointer to the red value
    \param[in]     green    pointer to the green value
    \param[in]     blue     pointer to the blue value
*/
void leddisplay_frame_get_rgb_xy(leddisplay_frame_t *p_frame, uint16_t x_coord, uint16_t y_coord, uint8_t *red, uint8_t *green, uint8_t *blue);

//! set pixel to colour
/*!
    Top-left of display is x=0 / y=0.

    \param[in,out] p_frame  pointer to frame memory
    \param[in]     x_coord  x coordinate
    \param[in]     y_coord  y coordinate
    \param[in]     alpha    transparency value
    \param[in]     red      red value
    \param[in]     green    green value
    \param[in]     blue     blue value
*/
void leddisplay_frame_xy_argb(leddisplay_frame_t *p_frame, uint16_t x_coord, uint16_t y_coord, uint8_t alpha, uint8_t red, uint8_t green, uint8_t blue);
//! fill frame with colour
/*!
    \param[in,out] p_frame  pointer to frame memory
    \param[in]     red      red value
    \param[in]     green    green value
    \param[in]     blue     blue value
*/
void leddisplay_frame_fill_rgb(leddisplay_frame_t *p_frame, uint8_t red, uint8_t green, uint8_t blue);

//! clear frame
/*!
    \param[in,out] p_frame  pointer to frame memory

    This is equivalient ot leddisplay_frame_xy_rgb(p_frame, 0, 0, 0) but faster.
*/
void leddisplay_frame_clear(leddisplay_frame_t *p_frame);

//! update display with frame
/*!
    Renders a frame to the display. The RGB data must be of the size #LEDDISPLAY_WIDTH *
    #LEDDISPLAY_HEIGHT * 3. This will block as necessary until the frame buffer memory
    becomes available.

    \param[in,out] frame  RGB data for one frame, or NULL to clear the display

    \note The function will modify the frame memory and you will have to draw the next frame fully
          from scratch.
*/
void leddisplay_frame_update(leddisplay_frame_t *p_frame);

void leddisplay_push_single_frame(leddisplay_frame_t *p_frame);

void sDisplayUpdateTask(void *pParam);

//! update display with frame
/*!
    Renders a frame to the display. The RGB data must be of the size #LEDDISPLAY_WIDTH *
    #LEDDISPLAY_HEIGHT * 3. This will block as necessary until the frame buffer memory
    becomes available.
    Unlike the normal leddisplay_frame_update(), this function takes an "uint8_t index" argument.

    \param[in,out] frame  RGB data for one frame, or NULL to clear the display

    \note The function will modify the frame memory and you will have to draw the next frame fully
          from scratch.
*/
void leddisplay_frame_update_index(leddisplay_frame_t *p_frame, uint8_t index);

//@}

/* *********************************************************************************************** */
//@}
#endif // __LEDDISPLAY_H__
