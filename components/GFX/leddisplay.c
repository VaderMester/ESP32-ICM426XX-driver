/*!
    \file
    \brief HUB75 LED display driver using I2S parallel mode

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

    The following text is from the original driver example from the esp forum (see above):

      This is example code to driver a p3(2121)64*32 -style RGB LED display. These types of displays
      do not have memory and need to be refreshed continuously. The display has 2 RGB inputs, 4
      inputs to select the active line, a pixel clock input, a latch enable input and an
      output-enable input. The display can be seen as 2 64x16 displays consisting of the upper half
      and the lower half of the display. Each half has a separate RGB pixel input, the rest of the
      inputs are shared.

      Each display half can only show one line of RGB pixels at a time: to do this, the RGB data for
      the line is input by setting the RGB input pins to the desired value for the first pixel,
      giving the display a clock pulse, setting the RGB input pins to the desired value for the
      second pixel, giving a clock pulse, etc. Do this 64 times to clock in an entire row. The
      pixels will not be displayed yet: until the latch input is made high, the display will still
      send out the previously clocked in line. Pulsing the latch input high will replace the
      displayed data with the data just clocked in.

      The 4 line select inputs select where the currently active line is displayed: when provided
      with a binary number (0-15), the latched pixel data will immediately appear on this
      line. Note: While clocking in data for a line, the *previous* line is still displayed, and
      these lines should be set to the value to reflect the position the *previous* line is supposed
      to be on.

      Finally, the screen has an OE input, which is used to disable the LEDs when latching new data
      and changing the state of the line select inputs: doing so hides any artifacts that appear at
      this time. The OE line is also used to dim the display by only turning it on for a limited
      time every line.

      All in all, an image can be displayed by 'scanning' the display, say, 100 times per
      second. The slowness of the human eye hides the fact that only one line is showed at a time,
      and the display looks like every pixel is driven at the same time.

      Now, the RGB inputs for these types of displays are digital, meaning each red, green and blue
      subpixel can only be on or off. This leads to a color palette of 8 pixels, not enough to
      display nice pictures. To get around this, we use binary code modulation.

      Binary code modulation is somewhat like PWM, but easier to implement in our case. First, we
      define the time we would refresh the display without binary code modulation as the 'frame
      time'. For, say, a four-bit binary code modulation, the frame time is divided into 15 ticks of
      equal length.

      We also define 4 subframes (0 to 3), defining which LEDs are on and which LEDs are off during
      that subframe. (Subframes are the same as a normal frame in non-binary-coded-modulation mode,
      but are showed faster.)  From our (non-monochrome) input image, we take the (8-bit: bit 7 to
      bit 0) RGB pixel values. If the pixel values have bit 7 set, we turn the corresponding LED on
      in subframe 3. If they have bit 6 set, we turn on the corresponding LED in subframe 2, if bit
      5 is set subframe 1, if bit 4 is set in subframe 0.

      Now, in order to (on average within a frame) turn a LED on for the time specified in the pixel
      value in the input data, we need to weigh the subframes. We have 15 pixels: if we show
      subframe 3 for 8 of them, subframe 2 for 4 of them, subframe 1 for 2 of them and subframe 1
      for 1 of them, this 'automatically' happens. (We also distribute the subframes evenly over the
      ticks, which reduces flicker.)

      In this code, we use the I2S peripheral in parallel mode to achieve this. Essentially, first
      we allocate memory for all subframes. This memory contains a sequence of all the signals
      (2xRGB, line select, latch enable, output enable) that need to be sent to the display for that
      subframe.  Then we ask the I2S-parallel driver to set up a DMA chain so the subframes are sent
      out in a sequence that satisfies the requirement that subframe x has to be sent out for (2^x)
      ticks. Finally, we fill the subframes with image data.

      We use a frontbuffer/backbuffer technique here to make sure the display is refreshed in one go
      and drawing artifacts do not reach the display.  In practice, for small displays this is not
      really necessarily.

      Finally, the binary code modulated intensity of a LED does not correspond to the intensity as
      seen by human eyes. To correct for that, a luminance correction is used. See val2pwm.c for
      more info.

      Note: Because every subframe contains one bit of grayscale information, they are also referred
      to as 'bitplanes' by the code below.

    Note that the text above may not be fully true anymore for this implementation.

    \todo study SmartMatrix changes (https://github.com/pixelmatix/SmartMatrix/tree/teensylc),
      specifically https://github.com/pixelmatix/SmartMatrix/commit/e3f6784304da95bfe1fc63a68b4acf05546c0e78
    \todo https://github.com/mrfaptastic/ESP32-RGB64x32MatrixPanel-I2S-DMA

*/

/* *********************************************************************************************** */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>

#include <sdkconfig.h>
#include <esp_log.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>

#include "val2pwm.h"
#include "i2s_parallel.h"

#include <leddisplay.h>
#include "esp_attr.h"

/* *********************************************************************************************** */
// local logging and debugging

#define LOGNAME "leddisplay"
#define ERROR(fmt, ...)   ESP_LOGE(LOGNAME, fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)    ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...)   ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...)   ESP_LOGV(LOGNAME, fmt, ## __VA_ARGS__)

#define MS2TICKS(ms)              ((ms) / portTICK_PERIOD_MS)
#define osSleep(ms)               vTaskDelay(MS2TICKS(ms));
/* *********************************************************************************************** */
// some useful macros

/* ************* LOCAL DEBUG *******************************/
#define DEBUG_EN 0

#ifdef BIT
#  undef BIT
#endif
#define BIT(bit) (1<<(bit))
#define STRINGIFY(x) _STRINGIFY(x)
#define _STRINGIFY(x) #x
#define NUMOF(x) (sizeof(x)/sizeof(*(x)))

/* *********************************************************************************************** */
// I2S bus bits (corresponds to the GPIOs)

// display panel upper half
#define BIT_R1   BIT(0)   // CONFIG_LEDDISPLAY_R1_GPIO
#define BIT_G1   BIT(1)   // CONFIG_LEDDISPLAY_G1_GPIO
#define BIT_B1   BIT(2)   // CONFIG_LEDDISPLAY_B1_GPIO

// display panel lower half
#define BIT_R2   BIT(3)   // CONFIG_LEDDISPLAY_R2_GPIO
#define BIT_G2   BIT(4)   // CONFIG_LEDDISPLAY_G2_GPIO
#define BIT_B2   BIT(5)   // CONFIG_LEDDISPLAY_B2_GPIO

// display panel control signals (latch, output enable)
#define BIT_LAT  BIT(6)   // CONFIG_LEDDISPLAY_LAT_GPIO

//----64x32x3
#define BIT_OE  BIT(7)
#define BIT_SEL0  BIT(8)
#define BIT_SEL1  BIT(9)
#define BIT_SEL2  BIT(10)

//----32x32x6
#define BIT_OE0   BIT(7)   // CONFIG_LEDDISPLAY_OE0_GPIO
#define BIT_OE1   BIT(8)   // CONFIG_LEDDISPLAY_OE1_GPIO
#define BIT_OE2   BIT(9)   // CONFIG_LEDDISPLAY_OE2_GPIO

// row address
#define BIT_A    BIT(11)   // CONFIG_LEDDISPLAY_A_GPIO
#define BIT_B    BIT(12)   // CONFIG_LEDDISPLAY_B_GPIO
#define BIT_C    BIT(13)  // CONFIG_LEDDISPLAY_C_GPIO
#define BIT_D    BIT(14)  // CONFIG_LEDDISPLAY_D_GPIO
#define BIT_E    BIT(15)  // CONFIG_LEDDISPLAY_E_GPIO

// CONFIG_LEDDISPLAY_E_CLK

/* *********************************************************************************************** */
// configuration (see also leddisplay.h)

#if CONFIG_LEDDISPLAY_TYPE_32X16_4SCAN     // doesn't work
#  warning CONFIG_LEDDISPLAY_TYPE_32X16_4SCAN does not work
#  define LEDDISPLAY_ROWS_IN_PARALLEL      4

#elif CONFIG_LEDDISPLAY_TYPE_32X16_8SCAN   // tested, works
#  define LEDDISPLAY_ROWS_IN_PARALLEL      2

#elif CONFIG_LEDDISPLAY_TYPE_32X32_8SCAN   // doesn't work
#  warning CONFIG_LEDDISPLAY_TYPE_32X32_8SCAN does not work
#  define LEDDISPLAY_ROWS_IN_PARALLEL      4

#elif CONFIG_LEDDISPLAY_TYPE_32X32_16SCAN  // tested, works
#  define LEDDISPLAY_ROWS_IN_PARALLEL      2

#elif CONFIG_LEDDISPLAY_TYPE_64X32_8SCAN   // doesn't work
#  warning CONFIG_LEDDISPLAY_TYPE_64X32_8SCAN does not work
#  define LEDDISPLAY_ROWS_IN_PARALLEL      4

#elif CONFIG_LEDDISPLAY_TYPE_64X32_16SCAN  // tested, works
#  define LEDDISPLAY_ROWS_IN_PARALLEL      2

#elif CONFIG_LEDDISPLAY_TYPE_32X32x6_16SCAN  // LED Cube
#  define LEDDISPLAY_ROWS_IN_PARALLEL      2

#elif CONFIG_LEDDISPLAY_TYPE_64X32x3_16SCAN  // LED Cube
#  define LEDDISPLAY_ROWS_IN_PARALLEL      2

#elif CONFIG_LEDDISPLAY_TYPE_192x32_16SCAN  // LED Cube
#  define LEDDISPLAY_ROWS_IN_PARALLEL      2

#elif CONFIG_LEDDISPLAY_TYPE_64X64_32SCAN  // not tested
#  define LEDDISPLAY_ROWS_IN_PARALLEL      2
#  define LEDDISPLAY_NEED_E_GPIO 1
#  if CONFIG_LEDDISPLAY_E_GPIO < 0
#    error Need CONFIG_LEDDISPLAY_E_GPIO > 0!
#  endif
#else
#  error This CONFIG_LEDDISPLAY_TYPE is not implemented!
#endif

#if CONFIG_LEDDISPLAY_I2S_FREQ_13MHZ
#  define I2S_CLOCK_SPEED  13333334
#elif CONFIG_LEDDISPLAY_I2S_FREQ_16MHZ
#  define I2S_CLOCK_SPEED  16000000
#elif CONFIG_LEDDISPLAY_I2S_FREQ_20MHZ
#  define I2S_CLOCK_SPEED  20000000
#elif CONFIG_LEDDISPLAY_I2S_FREQ_26MHZ
#  define I2S_CLOCK_SPEED  26666667
#else
#  error This CONFIG_LEDDISPLAY_I2S_FREQ is not implemented!
#endif

#define NUM_FRAME_BUFFERS         2
#define CONFIG_BLOCK_EN           1
//#define OE_OFF_CLKS_AFTER_LATCH   1
#define COLOR_DEPTH_BITS CONFIG_COLOR_DEPTH_BITS
#define PIXELS_PER_LATCH          ((LEDDISPLAY_WIDTH * LEDDISPLAY_HEIGHT) / LEDDISPLAY_HEIGHT)
#define ROWS_PER_FRAME            (LEDDISPLAY_HEIGHT / LEDDISPLAY_ROWS_IN_PARALLEL)

//Uncomment here if you are sure the memory requirements can be OK for available system resources
#define CONFIG_S_LSB_MSB_TRANSITION_BIT 0
/* *********************************************************************************************** */

// RGB data for two rows of pixels, and address and control signals
typedef struct row_bit_s
{
    uint16_t pixel[LEDDISPLAY_WIDTH];
} row_bit_t;
// Note: sizeof(data) must be multiple of 32 bits, as DMA linked list buffer address pointer must be word-aligned

// row data for each bitplane
typedef struct row_data_s
{
    row_bit_t rowbits[COLOR_DEPTH_BITS];
} row_data_t;

// full frame (display)
typedef struct frame_s
{
    row_data_t rowdata[ROWS_PER_FRAME];
} frame_t;

/* *********************************************************************************************** */
// pixel data (bitplanes) is organized from LSB to MSB sequentially by row, from row 0 to row
// matrixHeight/matrixRowsInParallel (two rows of pixels are refreshed in parallel)
static frame_t *s_frames;

static uint32_t s_current_frame;
static int s_lsb_msb_transition_bit;

// DMA memory linked list descriptors
lldesc_t *s_dmadesc_a;
lldesc_t *s_dmadesc_b;

// brightness level (value for data calculation, and percent used in API)
static int s_brightness_val;
static int s_brightness_percent;

static uint FRAME_TIME = 40;
uint32_t globalTicks;

#ifdef NUMBER_OF_PANELS
//global panel index used by leddisplay_frame_update() function
//static uint8_t currentpanelindex;
/*
*/
/*Panel selection type for using OE0, OE1, OE2 for OE logic.
*
* +----------------------------------------+
* |			       panelselect_t                |
* +---------+------------+-----+-----+-----+	 
* |   BIT#  |     7-3    |  2  |  1  |  0  |
* +---------+------------+-----+-----+-----+ 
* |  VALUE  | Empty bits | OE2 | OE1 | OE0 |
* +---------+------------+-----+-----+-----+
*
*
*  OE OUTPUT DECODING (Use 3->8 Demux decoder
*  USE INVERTED OUTPUTS!!!
* | Byte Value | OE2 | OE1 | OE0 | Panel No. |
* +------------+-----+-----+-----+-----------+
* |      0     |  L  |  L  |  L  |     X     |
* +------------+-----+-----+-----+-----------+
* |      7     |  H  |  H  |  H  |     X     |
* +------------+-----+-----+-----+-----------+
* |      1     |  L  |  L  |  H  |     1     |
* +------------+-----+-----+-----+-----------+
* |      2     |  L  |  H  |  L  |     2     |
* +------------+-----+-----+-----+-----------+
* |      3     |  L  |  H  |  H  |     3     |
* +------------+-----+-----+-----+-----------+
* |      4     |  H  |  L  |  L  |     4     |
* +------------+-----+-----+-----+-----------+
* |      5     |  H  |  L  |  H  |     5     |
* +------------+-----+-----+-----+-----------+
* |      6     |  H  |  H  |  H  |     6     |
* +------------+-----+-----+-----+-----------+
*/
/* Frame Buffer Layout
*
*
*  panels_t[0]   |   panels_t[1]
*                |
* +-----------+  |  +-----------+
* |  Panel_1  |  |  |  Panel_1  |
* +-----------+  |  +-----------+
* |  Panel_2  |  |  |  Panel_2  |
* +-----------+  |  +-----------+
* |  Panel_3  |  |  |  Panel_3  |
* +-----------+  |  +-----------+
* |     *     |  |  |     *     |
* |     *     |  |  |     *     |
* |     *     |  |  |     *     |
* |     *     |  |  |     *     |
*
*/
/* leddisplay_panels_t
* Array of leddisplay_frame_t pointers, which can be mallocced to be leddisplay_frame_t framebuffers.
*/
typedef struct leddisplay_panels_s
{  
    leddisplay_frame_t *pn[NUMBER_OF_PANELS];
} leddisplay_panels_t;

volatile leddisplay_panels_t pan[2];

//Panel framebuffers coordinate orientation
/*
    See leddisplay.h, leddisplay_panels_set_orientation()
*/
uint8_t s_panels_orientation = 0;


uint8_t lastrendered;
//Displaybuffer status
//0 - Buffer is being written
//1 - ready to render
//2 - Render in progress
//3 - Render complete
//4->255 NOT IMPLEMENTED
static uint8_t dbstat[2] = {0, 0};   //render complete status

SemaphoreHandle_t xframebufsem[CONFIG_LEDDISPLAY_NUM_DISP_BUFFER];
StaticSemaphore_t xframesembuf[CONFIG_LEDDISPLAY_NUM_DISP_BUFFER];

SemaphoreHandle_t xlastrenderedSem = NULL;
StaticSemaphore_t xlastrenderedSembuf;

SemaphoreHandle_t xStatusSem = NULL;
StaticSemaphore_t xStatusSemBuf;

StaticQueue_t xQueueBuffer;
QueueHandle_t xDispQ;
uint8_t xQstore[2 * sizeof(int)];


static const int a = 0;
static const int b = 1;
#endif 

// flush complete semaphore
SemaphoreHandle_t s_shift_complete_sem;
static IRAM_ATTR int s_shift_complete_sem_cb(void)
{
    static BaseType_t xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(s_shift_complete_sem, &xHigherPriorityTaskWoken );
    return xHigherPriorityTaskWoken;
}

void leddisplay_pixel_update(int block)
{
    i2s_parallel_flip_to_buffer(&I2S1, s_current_frame);
    s_current_frame = (s_current_frame + 1) % NUM_FRAME_BUFFERS;

    // wait until buffer is no longer used (I2S will continue using buffer until it's done and only
    // then switch to the new one)
    if (block != 0)
    {
        xSemaphoreTake(s_shift_complete_sem, portMAX_DELAY);
    }
    // 100 / portTICK_PERIOD_MS
}

void leddisplay_pixel_update_2(void)
{
    i2s_parallel_resume(&I2S1);
    i2s_parallel_flip_to_buffer(&I2S1, s_current_frame);
    s_current_frame = (s_current_frame + 1) % NUM_FRAME_BUFFERS;
    // wait until buffer is no longer used (I2S will continue using buffer until it's done and only
    // then switch to the new one)
    xSemaphoreTake(s_shift_complete_sem, portMAX_DELAY);
    i2s_parallel_pause(&I2S1);
    // 100 / portTICK_PERIOD_MS
}

esp_err_t leddisplay_init(void)
{
    esp_err_t res = ESP_OK;

    INFO(STRINGIFY(LEDDISPLAY_WIDTH) "x" STRINGIFY(LEDDISPLAY_HEIGHT) " (" STRINGIFY(COLOR_DEPTH_BITS) "bits)");

    DEBUG("GPIOs:"
        " R1="  STRINGIFY(CONFIG_LEDDISPLAY_R1_GPIO)
        " G1="  STRINGIFY(CONFIG_LEDDISPLAY_G1_GPIO)
        " B1="  STRINGIFY(CONFIG_LEDDISPLAY_B1_GPIO)
        " R2="  STRINGIFY(CONFIG_LEDDISPLAY_R2_GPIO)
        " G2="  STRINGIFY(CONFIG_LEDDISPLAY_G2_GPIO)
        " B2="  STRINGIFY(CONFIG_LEDDISPLAY_B2_GPIO)
        " A="   STRINGIFY(CONFIG_LEDDISPLAY_A_GPIO)
        " B="   STRINGIFY(CONFIG_LEDDISPLAY_B_GPIO)
        " C="   STRINGIFY(CONFIG_LEDDISPLAY_C_GPIO)
        " D="   STRINGIFY(CONFIG_LEDDISPLAY_D_GPIO)
#if LEDDISPLAY_NEED_E_GPIO
        " E="   STRINGIFY(CONFIG_LEDDISPLAY_E_GPIO)
#endif
        " LAT=" STRINGIFY(CONFIG_LEDDISPLAY_LAT_GPIO)
        " OE0="  STRINGIFY(CONFIG_LEDDISPLAY_OE0_GPIO)
        " OE1="  STRINGIFY(CONFIG_LEDDISPLAY_OE1_GPIO)
        " OE2="  STRINGIFY(CONFIG_LEDDISPLAY_OE2_GPIO)
        " CLK=" STRINGIFY(CONFIG_LEDDISPLAY_CLK_GPIO));

    // set default brightness 75%
    leddisplay_set_brightness(30);

    // allocate memory for the frame buffers, initialise frame buffers
        if (res == ESP_OK)
    {
        DEBUG("frame buffers: size=%u (available total=%u, largest=%u)", NUM_FRAME_BUFFERS * sizeof(frame_t),
            heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
        s_frames = (frame_t *)heap_caps_malloc(NUM_FRAME_BUFFERS * sizeof(frame_t), MALLOC_CAP_DMA);
        if (s_frames == NULL)
        {
            WARNING("framebuf alloc");
            res = ESP_ERR_NO_MEM;
        }
        // clear frame buffers
        else
        {
            const int old_brightness = leddisplay_set_brightness(0);

            s_current_frame = 1;
            leddisplay_pixel_fill_rgb(0, 0, 0);
            s_current_frame = 0;
            leddisplay_pixel_fill_rgb(0, 0, 0);

            leddisplay_set_brightness(old_brightness);
        }
    }

    // calculate the lowest LSBMSB_TRANSITION_BIT value that will fit in memory and achieves the minimal refresh rate
    int numDescriptorsPerRow = 0;
    int refreshRate = 0;
    bool ramOkay = false;
    bool refreshOkay = false;
    if (res == ESP_OK)
    {
        int largestBlockFree = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
        int totalFree        = heap_caps_get_free_size(MALLOC_CAP_DMA);

        s_lsb_msb_transition_bit = 0;

        while (1)
        {
            ramOkay = false;
            refreshOkay = false;

            // calculate memory requirements for this value of s_lsb_msb_transition_bit
            numDescriptorsPerRow = 1;
            for (int i = s_lsb_msb_transition_bit + 1; i < COLOR_DEPTH_BITS; i++)
            {
                numDescriptorsPerRow += (1 << (i - s_lsb_msb_transition_bit - 1));
            }
            int ramRequired = numDescriptorsPerRow * ROWS_PER_FRAME * NUM_FRAME_BUFFERS * sizeof(lldesc_t);

            // calculate achievable refresh rate for this value of s_lsb_msb_transition_bit
            int psPerClock = 1000000000000UL / I2S_CLOCK_SPEED;
            int nsPerLatch = (PIXELS_PER_LATCH * psPerClock) / 1000;
            // add time to shift out LSBs + LSB-MSB transition bit - this ignores fractions...
            int nsPerRow = COLOR_DEPTH_BITS * nsPerLatch;
            // add time to shift out MSBs
            for (int i = s_lsb_msb_transition_bit + 1; i < COLOR_DEPTH_BITS; i++)
            {
                nsPerRow += (1 << (i - s_lsb_msb_transition_bit - 1)) * (COLOR_DEPTH_BITS - i) * nsPerLatch;
            }
            int nsPerFrame = nsPerRow * ROWS_PER_FRAME;
            refreshRate = 1000000000UL / nsPerFrame;

            // check if that satisfies our requirements
            if ( (ramRequired < largestBlockFree) && (ramRequired < (totalFree - CONFIG_LEDDISPLAY_PRESERVE_RAM_SIZE)) )
            {
                ramOkay = true;
            }
            if (refreshRate >= CONFIG_LEDDISPLAY_MIN_FRAME_RATE)
            {
                refreshOkay = true;
            }

            // log summary
            DEBUG("lsb_msb_transition_bit=%d: ramRequired=%u available=%u largest=%u %s, refreshRate=%d %s",
                s_lsb_msb_transition_bit, ramRequired, totalFree, largestBlockFree, ramOkay ? ":-)" : ":-(",
                refreshRate, refreshOkay ? ":-)" : ":-(");

            // stop if we're satisfied
            if (ramOkay && refreshOkay)
            {
                break;
            }
            // try again if we can do more
            if ( s_lsb_msb_transition_bit < (COLOR_DEPTH_BITS - 1) )
            {
                s_lsb_msb_transition_bit++;
            }
            // give up
            else
            {
                break;
            }
        }

        // are we happy?
        if (ramOkay && refreshOkay)
        {
            DEBUG("finally: lsb_msb_transition_bit=%d/%d, rows=%d, RAM=%d, refresh=%d", s_lsb_msb_transition_bit, COLOR_DEPTH_BITS - 1,
                ROWS_PER_FRAME, NUM_FRAME_BUFFERS * numDescriptorsPerRow * ROWS_PER_FRAME * sizeof(lldesc_t), refreshRate);
        }
        // give up if we could not meet the RAM and refresh rate requirements
        else
        {
            if (!ramOkay)
            {
                WARNING("desc alloc");
                res = ESP_ERR_NO_MEM;
            }
            if (!refreshOkay)
            {
                WARNING("refresh");
                res = ESP_FAIL;
            }
        }
    }

    // malloc the DMA linked list descriptors that i2s_parallel will need
    int desccount = numDescriptorsPerRow * ROWS_PER_FRAME;
    if (res == ESP_OK)
    {
        s_dmadesc_a = (lldesc_t *)heap_caps_malloc(desccount * sizeof(lldesc_t), MALLOC_CAP_DMA);
        if (s_dmadesc_a == NULL)
        {
            WARNING("desc a alloc");
            res = ESP_ERR_NO_MEM;
        }
        s_dmadesc_b = (lldesc_t *)heap_caps_malloc(desccount * sizeof(lldesc_t), MALLOC_CAP_DMA);
        if (s_dmadesc_b == NULL)
        {
            WARNING("desc b alloc");
            res = ESP_ERR_NO_MEM;
        }
    }

    //heap_caps_print_heap_info(MALLOC_CAP_DMA);

    // fill DMA linked lists for both frames
    if (res == ESP_OK)
    {
        lldesc_t *prevdmadesca = NULL;
        lldesc_t *prevdmadescb = NULL;
        int currentDescOffset = 0;
        for (int j = 0; j < ROWS_PER_FRAME; j++)
        {
            // first set of data is LSB through MSB, single pass - all color bits are displayed once, which takes care of everything below and inlcluding LSBMSB_TRANSITION_BIT
            // TODO: size must be less than DMA_MAX - worst case for SmartMatrix Library: 16-bpp with 256 pixels per row would exceed this, need to break into two
            i2s_parallel_link_dma_desc(&s_dmadesc_a[currentDescOffset], prevdmadesca, &(s_frames[0].rowdata[j].rowbits[0].pixel), sizeof(row_bit_t) * COLOR_DEPTH_BITS);
            prevdmadesca = &s_dmadesc_a[currentDescOffset];
            i2s_parallel_link_dma_desc(&s_dmadesc_b[currentDescOffset], prevdmadescb, &(s_frames[1].rowdata[j].rowbits[0].pixel), sizeof(row_bit_t) * COLOR_DEPTH_BITS);
            prevdmadescb = &s_dmadesc_b[currentDescOffset];
            currentDescOffset++;
            //DEBUG("row %d:", j);

            for (int i = s_lsb_msb_transition_bit + 1; i < COLOR_DEPTH_BITS; i++)
            {
                // binary time division setup: we need 2 of bit (LSBMSB_TRANSITION_BIT + 1) four of (LSBMSB_TRANSITION_BIT + 2), etc
                // because we sweep through to MSB each time, it divides the number of times we have to sweep in half (saving linked list RAM)
                // we need 2^(i - LSBMSB_TRANSITION_BIT - 1) == 1 << (i - LSBMSB_TRANSITION_BIT - 1) passes from i to MSB
                //DEBUG("buffer %d: repeat %d times, size: %d, from %d - %d", nextBufdescIndex, 1<<(i - LSBMSB_TRANSITION_BIT - 1), (COLOR_DEPTH_BITS - i), i, COLOR_DEPTH_BITS-1);
                for (int k = 0; k < (1 << (i - s_lsb_msb_transition_bit - 1)); k++)
                {
                    i2s_parallel_link_dma_desc(&s_dmadesc_a[currentDescOffset], prevdmadesca, &(s_frames[0].rowdata[j].rowbits[i].pixel), sizeof(row_bit_t) * (COLOR_DEPTH_BITS - i));
                    prevdmadesca = &s_dmadesc_a[currentDescOffset];
                    i2s_parallel_link_dma_desc(&s_dmadesc_b[currentDescOffset], prevdmadescb, &(s_frames[1].rowdata[j].rowbits[i].pixel), sizeof(row_bit_t) * (COLOR_DEPTH_BITS - i));
                    prevdmadescb = &s_dmadesc_b[currentDescOffset];
                    currentDescOffset++;
                    //DEBUG("i %d, j %d, k %d", i, j, k);
                }
            }
        }
        // end markers
        s_dmadesc_a[desccount - 1].eof = 1;
        s_dmadesc_b[desccount - 1].eof = 1;
        s_dmadesc_a[desccount - 1].qe.stqe_next = (lldesc_t *)&s_dmadesc_a[0];
        s_dmadesc_b[desccount - 1].qe.stqe_next = (lldesc_t *)&s_dmadesc_b[0];
    }

    // flush complete semaphore
    if (res == ESP_OK)
    {
#if CONFIG_SUPPORT_STATIC_ALLOCATION
        static StaticSemaphore_t sem;
        s_shift_complete_sem = xSemaphoreCreateBinaryStatic(&sem);
#else
        s_shift_complete_sem = xSemaphoreCreateBinary();
#endif
        i2s_parallel_set_shiftcomplete_cb(s_shift_complete_sem_cb);
    }

    // initialise parallel I2S
    if (res == ESP_OK)
    {
        i2s_parallel_config_t cfg =
        {
            .gpio_bus =
            {
                CONFIG_LEDDISPLAY_R1_GPIO,   //  0 BIT_R1
                CONFIG_LEDDISPLAY_G1_GPIO,   //  1 BIT_G1
                CONFIG_LEDDISPLAY_B1_GPIO,   //  2 BIT_B1
                CONFIG_LEDDISPLAY_R2_GPIO,   //  3 BIT_R2
                CONFIG_LEDDISPLAY_G2_GPIO,   //  4 BIT_G2
                CONFIG_LEDDISPLAY_B2_GPIO,   //  5 BIT_B2
                CONFIG_LEDDISPLAY_LAT_GPIO,  //  6 BIT_LAT
                CONFIG_LEDDISPLAY_OE_GPIO,  //  7 BIT_OE0
				CONFIG_LEDDISPLAY_OE1_GPIO,  //  8 BIT_OE1
				CONFIG_LEDDISPLAY_OE2_GPIO,  //  9 BIT_OE2
                CONFIG_LEDDISPLAY_A_GPIO,    //  10 BIT_A
                CONFIG_LEDDISPLAY_B_GPIO,    //  11 BIT_B
                CONFIG_LEDDISPLAY_C_GPIO,    //  12 BIT_C
                CONFIG_LEDDISPLAY_D_GPIO,    //  13 BIT_D
#if LEDDISPLAY_NEED_E_GPIO
                CONFIG_LEDDISPLAY_E_GPIO,    // 14 BIT_E
#else
                -1,
#endif
                -1,                          // 15
            },
            .gpio_clk    = CONFIG_LEDDISPLAY_CLK_GPIO,
            .clkspeed_hz = I2S_CLOCK_SPEED,
            .bits        = I2S_PARALLEL_BITS_16,
            .desccount_a = desccount,
            .desccount_b = desccount,
            .lldesc_a    = s_dmadesc_a,
            .lldesc_b    = s_dmadesc_b,
        };

        esp_err_t res2 = i2s_parallel_setup(&I2S1, &cfg);
        if (res2 != ESP_OK)
        {
            WARNING("i2s fail (%d, %s)", res, esp_err_to_name(res));
            res = ESP_FAIL;
        }
    }

    if (res == ESP_OK)
    {
        INFO("init done");
    }
    // clean up on error
    else
    {
        WARNING("init fail: %s (%d)", esp_err_to_name(res), res);
        leddisplay_shutdown();
    }
    return res;
}//end of display_init()

#ifdef NUMBER_OF_PANELS
esp_err_t leddisplay_init_multipanel(void)
{
    esp_err_t res = ESP_OK;

    INFO(STRINGIFY(LEDDISPLAY_WIDTH) "x" STRINGIFY(LEDDISPLAY_HEIGHT) " (" STRINGIFY(COLOR_DEPTH_BITS) "bits)");

    WARNING("GPIOs:"
        " R1="  STRINGIFY(CONFIG_LEDDISPLAY_R1_GPIO)
        " G1="  STRINGIFY(CONFIG_LEDDISPLAY_G1_GPIO)
        " B1="  STRINGIFY(CONFIG_LEDDISPLAY_B1_GPIO)
        " R2="  STRINGIFY(CONFIG_LEDDISPLAY_R2_GPIO)
        " G2="  STRINGIFY(CONFIG_LEDDISPLAY_G2_GPIO)
        " B2="  STRINGIFY(CONFIG_LEDDISPLAY_B2_GPIO)
        " A="   STRINGIFY(CONFIG_LEDDISPLAY_A_GPIO)
        " B="   STRINGIFY(CONFIG_LEDDISPLAY_B_GPIO)
        " C="   STRINGIFY(CONFIG_LEDDISPLAY_C_GPIO)
        " D="   STRINGIFY(CONFIG_LEDDISPLAY_D_GPIO)
        " CLK=" STRINGIFY(CONFIG_LEDDISPLAY_CLK_GPIO)

#if LEDDISPLAY_NEED_E_GPIO
        " E="   STRINGIFY(CONFIG_LEDDISPLAY_E_GPIO)
#endif
#ifdef CONFIG_LEDDISPLAY_TYPE_64X32x3_16SCAN
        " LAT=" STRINGIFY(CONFIG_LEDDISPLAY_LAT_GPIO)
        " OE="  STRINGIFY(CONFIG_LEDDISPLAY_OE_GPIO)
        " SEL0="  STRINGIFY(CONFIG_LEDDISPLAY_SEL0_GPIO)
        " SEL1="  STRINGIFY(CONFIG_LEDDISPLAY_SEL1_GPIO)
        " SEL2="  STRINGIFY(CONFIG_LEDDISPLAY_SEL1_GPIO)
#elif CONFIG_LEDDISPLAY_TYPE_32X32x6_16SCAN
        " LAT=" STRINGIFY(CONFIG_LEDDISPLAY_LAT_GPIO)
        " OE0="  STRINGIFY(CONFIG_LEDDISPLAY_OE0_GPIO)
        " OE1="  STRINGIFY(CONFIG_LEDDISPLAY_OE1_GPIO)
        " OE2="  STRINGIFY(CONFIG_LEDDISPLAY_OE2_GPIO)
#elif CONFIG_LEDDISPLAY_TYPE_192x32_16SCAN
        " LAT=" STRINGIFY(CONFIG_LEDDISPLAY_LAT_GPIO)
        " OE="  STRINGIFY(CONFIG_LEDDISPLAY_OE_GPIO)
#endif
    );
    // set default brightness 75%
    leddisplay_set_brightness(40);

    // allocate memory for the frame buffers, initialise frame buffers
    if (res == ESP_OK)
    {
        DEBUG("frame buffers: size=%u (available total=%u, largest=%u)", NUM_FRAME_BUFFERS * sizeof(frame_t),
            heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
        s_frames = (frame_t *)heap_caps_malloc(NUM_FRAME_BUFFERS * sizeof(frame_t), MALLOC_CAP_DMA);
        if (s_frames == NULL)
        {
            WARNING("framebuf alloc");
            res = ESP_ERR_NO_MEM;
            return res;
        }
    }


    // calculate the lowest LSBMSB_TRANSITION_BIT value that will fit in memory and achieves the minimal refresh rate
    int numDescriptorsPerRow = 0;
    int refreshRate = 0;
    bool ramOkay = false;
    bool refreshOkay = false;
    if (res == ESP_OK)
    {
        int largestBlockFree = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
        int totalFree        = heap_caps_get_free_size(MALLOC_CAP_DMA);

        s_lsb_msb_transition_bit = 0;

        while (1)
        {
            ramOkay = false;
            refreshOkay = false;

            // calculate memory requirements for this value of s_lsb_msb_transition_bit
            numDescriptorsPerRow = 1;
            for (int i = s_lsb_msb_transition_bit + 1; i < COLOR_DEPTH_BITS; i++)
            {
                numDescriptorsPerRow += (1 << (i - s_lsb_msb_transition_bit - 1));
            }
            int ramRequired = numDescriptorsPerRow * ROWS_PER_FRAME * NUM_FRAME_BUFFERS * sizeof(lldesc_t);

            // calculate achievable refresh rate for this value of s_lsb_msb_transition_bit
            int psPerClock = 1000000000000UL / I2S_CLOCK_SPEED;
            int nsPerLatch = (PIXELS_PER_LATCH * psPerClock) / 1000;
            // add time to shift out LSBs + LSB-MSB transition bit - this ignores fractions...
            int nsPerRow = COLOR_DEPTH_BITS * nsPerLatch;
            // add time to shift out MSBs
            for (int i = s_lsb_msb_transition_bit + 1; i < COLOR_DEPTH_BITS; i++)
            {
                nsPerRow += (1 << (i - s_lsb_msb_transition_bit - 1)) * (COLOR_DEPTH_BITS - i) * nsPerLatch;
            }
            int nsPerFrame = nsPerRow * ROWS_PER_FRAME;
            refreshRate = 1000000000UL / nsPerFrame;

            // check if that satisfies our requirements
            if ( (ramRequired < largestBlockFree) && (ramRequired < (totalFree - CONFIG_LEDDISPLAY_PRESERVE_RAM_SIZE)) )
            {
                ramOkay = true;
            }
            if (refreshRate >= CONFIG_LEDDISPLAY_MIN_FRAME_RATE)
            {
                refreshOkay = true;
            }

            // log summary
            DEBUG("lsb_msb_transition_bit=%d: ramRequired=%u available=%u largest=%u %s, refreshRate=%d %s",
                s_lsb_msb_transition_bit, ramRequired, totalFree, largestBlockFree, ramOkay ? ":-)" : ":-(",
                refreshRate, refreshOkay ? ":-)" : ":-(");

            // stop if we're satisfied
            if (ramOkay && refreshOkay)
            {
                break;
            }
            // try again if we can do more
            if ( s_lsb_msb_transition_bit < (COLOR_DEPTH_BITS - 1) )
            {
                s_lsb_msb_transition_bit++;
            }
            // give up
            else
            {
                break;
            }
        }

        // are we happy?
        if (ramOkay && refreshOkay)
        {
            WARNING("finally: lsb_msb_transition_bit=%d/%d, rows=%d, DESC RAM=%d, BUFFER RAM=%d, refresh=%d", s_lsb_msb_transition_bit, COLOR_DEPTH_BITS - 1,
                ROWS_PER_FRAME, NUM_FRAME_BUFFERS * numDescriptorsPerRow * ROWS_PER_FRAME * sizeof(lldesc_t), NUM_FRAME_BUFFERS * 2 * ROWS_PER_FRAME * LEDDISPLAY_WIDTH * COLOR_DEPTH_BITS, refreshRate);
        }
        // give up if we could not meet the RAM and refresh rate requirements
        else
        {
            if (!ramOkay)
            {
                WARNING("desc alloc");
                res = ESP_ERR_NO_MEM;
            }
            if (!refreshOkay)
            {
                WARNING("refresh");
                res = ESP_FAIL;
            }
        }
    }

    // malloc the DMA linked list descriptors that i2s_parallel will need
    int desccount = numDescriptorsPerRow * ROWS_PER_FRAME;
    if (res == ESP_OK)
    {
        s_dmadesc_a = (lldesc_t *)heap_caps_malloc(desccount * sizeof(lldesc_t), MALLOC_CAP_DMA);
        if (s_dmadesc_a == NULL)
        {
            WARNING("desc a alloc");
            res = ESP_ERR_NO_MEM;
        }
        s_dmadesc_b = (lldesc_t *)heap_caps_malloc(desccount * sizeof(lldesc_t), MALLOC_CAP_DMA);
        if (s_dmadesc_b == NULL)
        {
            WARNING("desc b alloc");
            res = ESP_ERR_NO_MEM;
        }
    }

    //heap_caps_print_heap_info(MALLOC_CAP_DMA);

    // fill DMA linked lists for both frames
    if (res == ESP_OK)
    {
        lldesc_t *prevdmadesca = NULL;
        lldesc_t *prevdmadescb = NULL;
        int currentDescOffset = 0;
        for (int j = 0; j < ROWS_PER_FRAME; j++)
        {
            // first set of data is LSB through MSB, single pass - all color bits are displayed once, which takes care of everything below and inlcluding LSBMSB_TRANSITION_BIT
            // TODO: size must be less than DMA_MAX - worst case for SmartMatrix Library: 16-bpp with 256 pixels per row would exceed this, need to break into two
            i2s_parallel_link_dma_desc(&s_dmadesc_a[currentDescOffset], prevdmadesca, &(s_frames[0].rowdata[j].rowbits[0].pixel), sizeof(row_bit_t) * COLOR_DEPTH_BITS);
            prevdmadesca = &s_dmadesc_a[currentDescOffset];
            i2s_parallel_link_dma_desc(&s_dmadesc_b[currentDescOffset], prevdmadescb, &(s_frames[1].rowdata[j].rowbits[0].pixel), sizeof(row_bit_t) * COLOR_DEPTH_BITS);
            prevdmadescb = &s_dmadesc_b[currentDescOffset];
            currentDescOffset++;
            //DEBUG("row %d:", j);

            for (int i = s_lsb_msb_transition_bit + 1; i < COLOR_DEPTH_BITS; i++)
            {
                // binary time division setup: we need 2 of bit (LSBMSB_TRANSITION_BIT + 1) four of (LSBMSB_TRANSITION_BIT + 2), etc
                // because we sweep through to MSB each time, it divides the number of times we have to sweep in half (saving linked list RAM)
                // we need 2^(i - LSBMSB_TRANSITION_BIT - 1) == 1 << (i - LSBMSB_TRANSITION_BIT - 1) passes from i to MSB
                //DEBUG("buffer %d: repeat %d times, size: %d, from %d - %d", nextBufdescIndex, 1<<(i - LSBMSB_TRANSITION_BIT - 1), (COLOR_DEPTH_BITS - i), i, COLOR_DEPTH_BITS-1);
                for (int k = 0; k < (1 << (i - s_lsb_msb_transition_bit - 1)); k++)
                {
                    i2s_parallel_link_dma_desc(&s_dmadesc_a[currentDescOffset], prevdmadesca, &(s_frames[0].rowdata[j].rowbits[i].pixel), sizeof(row_bit_t) * (COLOR_DEPTH_BITS - i));
                    prevdmadesca = &s_dmadesc_a[currentDescOffset];
                    i2s_parallel_link_dma_desc(&s_dmadesc_b[currentDescOffset], prevdmadescb, &(s_frames[1].rowdata[j].rowbits[i].pixel), sizeof(row_bit_t) * (COLOR_DEPTH_BITS - i));
                    prevdmadescb = &s_dmadesc_b[currentDescOffset];
                    currentDescOffset++;
                    //DEBUG("i %d, j %d, k %d", i, j, k);
                }
            }
        }
        // end markers
        s_dmadesc_a[desccount - 1].eof = 1;
        s_dmadesc_b[desccount - 1].eof = 1;
        s_dmadesc_a[desccount - 1].qe.stqe_next = (lldesc_t *)&s_dmadesc_a[0];
        s_dmadesc_b[desccount - 1].qe.stqe_next = (lldesc_t *)&s_dmadesc_b[0];
    }

    // flush complete semaphore
    if (res == ESP_OK)
    {
#if CONFIG_SUPPORT_STATIC_ALLOCATION
        static StaticSemaphore_t sem;
        s_shift_complete_sem = xSemaphoreCreateBinaryStatic(&sem);
#else
        s_shift_complete_sem = xSemaphoreCreateBinary();
#endif
        i2s_parallel_set_shiftcomplete_cb(s_shift_complete_sem_cb);
    }

    // initialise parallel I2S
    if (res == ESP_OK)
    {
        i2s_parallel_config_t cfg =
        {
            .gpio_bus =
            {

                CONFIG_LEDDISPLAY_R1_GPIO,   //  0 BIT_R1
                CONFIG_LEDDISPLAY_G1_GPIO,   //  1 BIT_G1
                CONFIG_LEDDISPLAY_B1_GPIO,   //  2 BIT_B1
                CONFIG_LEDDISPLAY_R2_GPIO,   //  3 BIT_R2
                CONFIG_LEDDISPLAY_G2_GPIO,   //  4 BIT_G2
                CONFIG_LEDDISPLAY_B2_GPIO,   //  5 BIT_B2
#ifdef CONFIG_LEDDISPLAY_TYPE_32x32x6_16SCAN
                CONFIG_LEDDISPLAY_LAT_GPIO,  //  6 BIT_LAT
                CONFIG_LEDDISPLAY_OE0_GPIO,  //  7 BIT_OE0
				CONFIG_LEDDISPLAY_OE1_GPIO,  //  8 BIT_OE1
				CONFIG_LEDDISPLAY_OE2_GPIO,  //  9 BIT_OE2
                -1,
                CONFIG_LEDDISPLAY_A_GPIO,    //  10 BIT_A
                CONFIG_LEDDISPLAY_B_GPIO,    //  11 BIT_B
                CONFIG_LEDDISPLAY_C_GPIO,    //  12 BIT_C
                CONFIG_LEDDISPLAY_D_GPIO,    //  13 BIT_D
#if LEDDISPLAY_NEED_E_GPIO
                CONFIG_LEDDISPLAY_E_GPIO,    // 14 BIT_E
#else
                -1,
#endif
#endif
#ifdef CONFIG_LEDDISPLAY_TYPE_64X32x3_16SCAN
                CONFIG_LEDDISPLAY_LAT_GPIO,     //  6 BIT_LAT
                CONFIG_LEDDISPLAY_OE_GPIO,      //  7 BIT_OE
                CONFIG_LEDDISPLAY_SEL0_GPIO,    //  8 BIT_SEL0
                CONFIG_LEDDISPLAY_SEL1_GPIO,    //  9 BIT_SEL1
                CONFIG_LEDDISPLAY_SEL2_GPIO,    //  10 BIT_SEL2
                CONFIG_LEDDISPLAY_A_GPIO,       //  11 BIT_A
                CONFIG_LEDDISPLAY_B_GPIO,       //  12 BIT_B
                CONFIG_LEDDISPLAY_C_GPIO,       //  13 BIT_C
                CONFIG_LEDDISPLAY_D_GPIO,       //  14 BIT_D
#if LEDDISPLAY_NEED_E_GPIO
                CONFIG_LEDDISPLAY_E_GPIO,       // 15 BIT_E
#else
                -1,
#endif
#endif
#ifdef CONFIG_LEDDISPLAY_TYPE_192x32_16SCAN
                CONFIG_LEDDISPLAY_LAT_GPIO,     //  6 BIT_LAT
                CONFIG_LEDDISPLAY_OE_GPIO,      //  7 BIT_OE
                -1,                             //  8
                -1,                             //  9
                -1,                              //  10
                CONFIG_LEDDISPLAY_A_GPIO,       //  11 BIT_A
                CONFIG_LEDDISPLAY_B_GPIO,       //  12 BIT_B
                CONFIG_LEDDISPLAY_C_GPIO,       //  13 BIT_C
                CONFIG_LEDDISPLAY_D_GPIO,       //  14 BIT_D
#if LEDDISPLAY_NEED_E_GPIO
                CONFIG_LEDDISPLAY_E_GPIO,       // 15 BIT_E
#else
                -1,
#endif
#endif
            },
            .gpio_clk    = CONFIG_LEDDISPLAY_CLK_GPIO,
            .clkspeed_hz = I2S_CLOCK_SPEED,
            .bits        = I2S_PARALLEL_BITS_16,
            .desccount_a = desccount,
            .desccount_b = desccount,
            .lldesc_a    = s_dmadesc_a,
            .lldesc_b    = s_dmadesc_b,
        };

        esp_err_t res2 = i2s_parallel_setup(&I2S1, &cfg);

        gpio_set_pull_mode(CONFIG_LEDDISPLAY_LAT_GPIO, GPIO_PULLUP_ONLY);
        gpio_set_pull_mode(CONFIG_LEDDISPLAY_OE_GPIO, GPIO_PULLUP_ONLY);

        if (res2 != ESP_OK)
        {
            WARNING("i2s fail (%d, %s)", res, esp_err_to_name(res));
            res = ESP_FAIL;
        }
    }

    if (res == ESP_OK)
    {
        INFO("init done");
    }
    // clean up on error
    else
    {
        WARNING("init fail: %s (%d)", esp_err_to_name(res), res);
        leddisplay_shutdown();
    }
    return res;
}//end of display_init_multipanel()

esp_err_t leddisplay_displaybuffer_init(uint8_t bufferindex)
{
    esp_err_t res = ESP_OK;
    if (bufferindex > CONFIG_LEDDISPLAY_NUM_DISP_BUFFER - 1)
    {
        res = ESP_FAIL;
        INFO("Configured maximum number of display buffers is smaller than what you want to configure");
        return res;
    }
    else
    {
        // allocate memory for the frame buffers, initialise frame buffers
        if (res == ESP_OK)
        {
            INFO("Display buffer: size=%u (available total=%u, largest=%u)", NUMBER_OF_PANELS * sizeof(leddisplay_frame_t),
                 heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
            for (int i = 0; i < NUMBER_OF_PANELS; i++)
            {
                pan[bufferindex].pn[i] = (leddisplay_frame_t *)heap_caps_malloc(sizeof(leddisplay_frame_t), MALLOC_CAP_DMA);
                if (pan[bufferindex].pn[i] == NULL)
                {
                    WARNING("displaybuf alloc ERROR, buff index: %d", bufferindex);
                    res = ESP_ERR_NO_MEM;
                    return res;
                }
            }
            if (xframebufsem[bufferindex] == NULL)
            {
                xframebufsem[bufferindex] = xSemaphoreCreateBinaryStatic(&xframesembuf[bufferindex]);
                leddisplay_panels_clear(bufferindex, 0);
                if (xSemaphoreGive(xframebufsem[bufferindex]) != pdTRUE)
                {
                    WARNING("Failed to allocate static semaphore for '%d'", bufferindex);
                    res = ESP_FAIL;
                }
            }
            if (xDispQ == 0)
            {
                xDispQ = xQueueCreateStatic(CONFIG_LEDDISPLAY_NUM_DISP_BUFFER, sizeof(int), &xQstore[0], &xQueueBuffer);
            }
            if (xStatusSem == NULL)
            {
                xStatusSem = xSemaphoreCreateBinaryStatic(&xStatusSemBuf);
                xSemaphoreGive(xStatusSem);
            }
            if (xlastrenderedSem == NULL)
            {
                xlastrenderedSem = xSemaphoreCreateBinaryStatic(&xlastrenderedSembuf);
                if (xSemaphoreGive(xlastrenderedSem) == pdTRUE)
                {
                    INFO("Created lastrenderedSem");
                }
            }
        }
        leddisplay_panels_set_buffer_status(bufferindex, 1); //ready
    }
    return res;
}

#ifdef CONFIG_LEDDISPLAY_PWR_EN_GPIO

void leddisplay_init_power_en_gpio(void) {
    gpio_pad_select_gpio(CONFIG_LEDDISPLAY_PWR_EN_GPIO);
    gpio_set_pull_mode(CONFIG_LEDDISPLAY_PWR_EN_GPIO, GPIO_PULLUP_ONLY);
    gpio_set_direction(CONFIG_LEDDISPLAY_PWR_EN_GPIO, GPIO_MODE_OUTPUT_OD);
    gpio_set_level(CONFIG_LEDDISPLAY_PWR_EN_GPIO, 0);
}

void leddisplay_set_power_state(uint32_t state) {
    gpio_set_level(CONFIG_LEDDISPLAY_PWR_EN_GPIO, state ? 0 : 1);
}

uint32_t leddisplay_get_power_state(void) {
    return gpio_get_level(CONFIG_LEDDISPLAY_PWR_EN_GPIO) ? 0 : 1;
}
#endif //#ifdef CONFIG_LEDDISPLAY_PWR_EN_GPIO


/*
IRAM_ATTR void leddisplay_panels_set_dispbuf_ready(unsigned int dispbufindex) {
    pan[dispbufindex].ready = 1;
}

IRAM_ATTR void leddisplay_panels_clear_dispbuf_ready(unsigned int dispbufindex) {
    pan[dispbufindex].ready = 0;
}

IRAM_ATTR int leddisplay_panels_get_dispbuf_ready(unsigned int dispbufindex) {
    return pan[dispbufindex].ready;
}
*/

IRAM_ATTR int leddisplay_panels_get_next_to_render(void) {
    xSemaphoreTake(xlastrenderedSem, portMAX_DELAY);
    int ret = lastrendered;
    xSemaphoreGive(xlastrenderedSem);
    return ret ? a : b;
}

IRAM_ATTR void leddisplay_panels_set_lastrendered(unsigned int dispbufindex) {
    xSemaphoreTake(xlastrenderedSem, portMAX_DELAY);
    lastrendered = dispbufindex;
    xSemaphoreGive(xlastrenderedSem);
}

IRAM_ATTR int leddisplay_panels_get_lastrendered(void) {
    xSemaphoreTake(xlastrenderedSem, portMAX_DELAY);
    int ret = lastrendered;
    xSemaphoreGive(xlastrenderedSem);
    return ret;
}

IRAM_ATTR int leddisplay_panels_get_buffer_status(unsigned int dispbufindex) {
    xSemaphoreTake(xStatusSem, portMAX_DELAY);
    uint8_t status = dbstat[dispbufindex];
    xSemaphoreGive(xStatusSem);
    return status;
}

IRAM_ATTR void leddisplay_panels_set_buffer_status(unsigned int dispbufindex, uint8_t status) {
    xSemaphoreTake(xStatusSem, portMAX_DELAY);
    dbstat[dispbufindex] = status;
    xSemaphoreGive(xStatusSem);
}

void leddisplay_panels_set_frametime(unsigned int millisec) {
    xSemaphoreTake(xStatusSem, portMAX_DELAY);
    FRAME_TIME = millisec;
    xSemaphoreGive(xStatusSem);
}
unsigned int leddisplay_panels_get_frametime(void) {
    xSemaphoreTake(xStatusSem, portMAX_DELAY);
    uint retval = FRAME_TIME;
    xSemaphoreGive(xStatusSem);
    return retval;
}
/*
IRAM_ATTR int leddisplay_panels_get_unblocked_dispbuf_index(void) {
    //int availablebuf;
    int bufunblocked[2] = {0, 0};
    if (xSemaphoreTake(xframebufsem[a], MS2TICKS(5)) == pdTRUE) {
        //availablebuf = a;
        //INFO("Semaphore taken for disp. buf 'a'");
        bufunblocked[a] = 1;
        xSemaphoreGive(xframebufsem[a]);
    }
    if (xSemaphoreTake(xframebufsem[b], MS2TICKS(5)) == pdTRUE) {
        //availablebuf = b;
        //INFO("Semaphore taken for disp. buf 'b'");
        bufunblocked[b] = 1;
        xSemaphoreGive(xframebufsem[b]);
    }

    if(bufunblocked[a] == 1) {
        if(leddisplay_panels_get_dispbuf_ready(a) == 1) {
            INFO("(a)");
            return a;
        } else {
            if (bufunblocked[b] == 1) {
                if(leddisplay_panels_get_dispbuf_ready(b) == 1) {
                    INFO("(b)");
                    return b;
                } else {
                    INFO("(a)");
                    return a;
                }
            }
        }
    }
    if(bufunblocked[b] == 1) {
        INFO("(b)");
        return b;
    }
    WARNING("All buffers are blocked");
    return -1;
}
*/

IRAM_ATTR esp_err_t leddisplay_panels_unblock_dispbuf(unsigned int index) {
    esp_err_t res = ESP_OK;
        if(xSemaphoreGive(xframebufsem[index]) != pdTRUE) {
            res = ESP_FAIL;
            INFO("Failed to unblock buf: %d (%s) ", index, index ? "b" : "a");
        }
    return res;
}

IRAM_ATTR esp_err_t leddisplay_panels_block_dispbuf(unsigned int index) {
    esp_err_t res = ESP_OK;
        if(xSemaphoreTake(xframebufsem[index], portMAX_DELAY) != pdTRUE) {
            res = ESP_FAIL;
            INFO("Failed to block buf: %d (%s) ", index, index ? "b" : "a");
        }
    return res;
}

IRAM_ATTR esp_err_t leddisplay_panels_block_MS_dispbuf(unsigned int index, unsigned int ms) {
    esp_err_t res = ESP_OK;
        if(xSemaphoreTake(xframebufsem[index], MS2TICKS(ms)) != pdTRUE) {
            res = ESP_FAIL;
            DEBUG("Failed to block buf: %d (%s) ", index, index ? "b" : "a");
        }
    return res;
}

IRAM_ATTR void leddisplay_panels_set_orientation(uint8_t orientation) {
    //Values higher than 1 to be implemented
    if(orientation > 1 ) {
        INFO("Tried to set panels orientation but argument is incorrect");
        return;
    }
    s_panels_orientation = orientation;
}

IRAM_ATTR int leddisplay_panels_get_orientation(void) {
    return s_panels_orientation;
}

IRAM_ATTR int leddisplay_panels_update(int dispbufindex, uint8_t panelnum) {
    if (dispbufindex != -1)
    {
        uint8_t disp = (unsigned) dispbufindex;
        //if (xSemaphoreTake(xframebufsem[disp], 0) == pdTRUE) {
            //leddisplay_panels_set_buffer_status(disp, 2); //render in progress
        #if LEDDISPLAY_WIDTH < 192
            if (panelnum == 0 || panelnum >= NUMBER_OF_PANELS)
            {
                //uint8_t maxpanelindex = NUMBER_OF_PANELS;
                for (int i = 0; i < NUMBER_OF_PANELS; i++)
                {
                    //uint8_t currentpanelnum = i+1;
                    //leddisplay_frame_t *pnframe = &pan.pn[i];
                    //leddisplay_frame_update_index(pnframe, panelnum);
                    leddisplay_frame_update_index(pan[disp].pn[i], i+1);
                }
            }
            else
            {
                //uint8_t panelindex = panelnum - 1;
                //leddisplay_frame_t *pnframe = &pan.pn[panelnum-1];
                //leddisplay_frame_update_index(pnframe, panelnum);
                leddisplay_frame_update_index(pan[disp].pn[panelnum - 1], panelnum);
            }
        #else
            leddisplay_frame_update(pan[disp].pn[0]);
            //leddisplay_panels_set_buffer_status(disp, 3); //render completed
        #endif
            //leddisplay_panels_unblock_dispbuf(disp);
            return disp;
        //}
    }
    return -1;
}

int leddisplay_panels_clear(int dispbufindex, uint8_t panelnum) {
    if(dispbufindex >= 0) {
            if( panelnum < 1 || panelnum >= NUMBER_OF_PANELS ){
                //uint8_t maxpanelindex = NUMBER_OF_PANELS;
                for(int i = 0; i < NUMBER_OF_PANELS; i++) {
                        //uint8_t currentpanelnum = i+1;
                        //leddisplay_frame_t *pnframe = &pan.pn[i];
                    //leddisplay_frame_update_index(pnframe, panelnum);
                    leddisplay_frame_clear(pan[dispbufindex].pn[i]);
                }
            } else {
                //uint8_t panelindex = panelnum - 1;
                //leddisplay_frame_t *pnframe = &pan.pn[panelnum-1];
                //leddisplay_frame_update_index(pnframe, panelnum);
                leddisplay_frame_clear(pan[dispbufindex].pn[panelnum-1]);
            }
            return 0;
    }
    ERROR("Incorrect value for 'int dispbufindex'");
    return -1;

}

inline int leddisplay_panels_xy_rgb(int dispbufindex, uint8_t startpanel, uint16_t x_coord, uint16_t y_coord, uint8_t red, uint8_t green, uint8_t blue) {
    int ret = 0;
    if((startpanel > NUMBER_OF_PANELS) || (startpanel == 0) || (dispbufindex < 0)){
        INFO("Incorrect index/panel given");
        ret = 1;
        return ret;   //Index/panel issue
    }

    #if LEDDISPLAY_WIDTH < 192
        //INFO("db: %d, startpanel: %d", dispbufindex, startpanel);
        uint8_t panelindex = startpanel - 1;
        //INFO("panelindex: %d", panelindex);
        //Error handling is in the end if anything goes wrong, wouldn't wanna break the function with too many IFs.
        //This function will be iterated quick like crazy, and need to be fast, if things are correct
        if ((s_panels_orientation == 0) && (y_coord < LEDDISPLAY_HEIGHT)) { //Horizontal
        #if LEDDISPLAY_WIDTH == 32
            uint16_t x = x_coord;
            uint8_t panoffset = (x >> 5);
            panelindex += panoffset;  //fast devision by 32
            uint8_t xoffs = (panelindex + 1 - startpanel) << 5;
            //INFO("x_coord: %d, y_coord: %d, xoffs: %d, panelindex: %d, panoffset: %d", x_coord, y_coord, xoffs, panelindex, panoffset);
            x_coord -= xoffs;
            leddisplay_frame_xy_rgb(pan[dispbufindex].pn[panelindex], x_coord, y_coord, red, green, blue);
        #elif LEDDISPLAY_WIDTH == 64
            uint16_t x = x_coord;
            uint8_t panoffset = (x % 64);
            panelindex += panoffset;
            x_coord -= (panelindex * 64);
            leddisplay_frame_xy_rgb(pan[dispbufindex].pn[panelindex], x_coord, y_coord, red, green, blue);
        #endif
            return 0; //everything is OK
        } else if ((s_panels_orientation == 1) && (x_coord < LEDDISPLAY_WIDTH)) { //Vertical
        #if LEDDISPLAY_HEIGHT == 32
            uint16_t y = y_coord;
            uint8_t panoffset = (y >> 5);
            panelindex += panoffset;  //fast devision by 32
            uint8_t yoffs = (panelindex + 1 - startpanel) << 5;
            y_coord -= yoffs;
            leddisplay_frame_xy_rgb(pan[dispbufindex].pn[panelindex], x_coord, y_coord, red, green, blue);
        #endif
        } else {
            if ((s_panels_orientation == 1) && (x_coord >= LEDDISPLAY_WIDTH))
            {
                #if DEBUG_EN
                WARNING("Fuck! X coordinate outside coord. field");
                #endif
                ret = 3; //X Coord. outside coord. field
            }
            if ((s_panels_orientation == 0) && (y_coord >= LEDDISPLAY_HEIGHT))
            {
                #if DEBUG_EN
                WARNING("Fuck! Y coordinate outside coord. field");
                #endif
                ret = 4; //Y Coord. outside coord. field
            }
        }
    #else
        //INFO("x_coord: %d, y_coord: %d, xoffs: %d, panelindex: %d, panoffset: %d", x_coord, y_coord, xoffs, panelindex, panoffset);
        leddisplay_frame_xy_rgb(pan[dispbufindex].pn[0], x_coord, y_coord, red, green, blue);
    #endif //END of LEDDISPLAY_WIDTH < 192
            return 0; //everything is OK
    return ret;
}

inline void leddisplay_panels_get_rgb_xy(int dispbufindex, uint16_t x_coord, uint16_t y_coord, uint8_t *red, uint8_t *green, uint8_t *blue) {
    leddisplay_frame_get_rgb_xy(pan[dispbufindex].pn[0], x_coord, y_coord, red, green, blue);
}


inline int leddisplay_panels_xy_argb(int dispbufindex, uint8_t startpanel, uint16_t x_coord, uint16_t y_coord, uint8_t alpha, uint8_t red, uint8_t green, uint8_t blue) {
    int ret = 0;
    
    if ((startpanel > NUMBER_OF_PANELS) || (startpanel == 0) || (dispbufindex < 0))
    {
        INFO("Incorrect index/panel given");
        ret = 1;
        return ret; //Index/panel issue
    }
    //INFO("db: %d, startpanel: %d", dispbufindex, startpanel);
    uint8_t panelindex = startpanel - 1;
    //INFO("panelindex: %d", panelindex);
    //Error handling is in the end if anything goes wrong, wouldn't wanna break the function with too many IFs.
    //This function will be iterated quick like crazy, and need to be fast, if things are correct
#if LEDDISPLAY_WIDTH < 192
    if ((s_panels_orientation == 0) && (y_coord < LEDDISPLAY_HEIGHT))
    { //Horizontal
#if LEDDISPLAY_WIDTH == 32
        uint16_t x = x_coord;
        uint8_t panoffset = (x >> 5);
        panelindex += panoffset; //fast devision by 32
        uint8_t xoffs = (panelindex + 1 - startpanel) << 5;
        //INFO("x_coord: %d, y_coord: %d, xoffs: %d, panelindex: %d, panoffset: %d", x_coord, y_coord, xoffs, panelindex, panoffset);
        x_coord -= xoffs;
        if (alpha != 0)
        {
            leddisplay_frame_xy_argb(pan[dispbufindex].pn[panelindex], x_coord, y_coord, alpha, red, green, blue);
        }
        else
        {
            leddisplay_frame_xy_rgb(pan[dispbufindex].pn[panelindex], x_coord, y_coord, red, green, blue);
        }
#elif LEDDISPLAY_WIDTH == 64
        uint16_t x = x_coord;
        uint8_t panoffset = (x % 64);
        panelindex += panoffset;
        x_coord -= (panelindex * 64);
        if (alpha != 0)
        {
            leddisplay_frame_xy_argb(pan[dispbufindex].pn[panelindex], x_coord, y_coord, alpha, red, green, blue);
        }
        else
        {
            leddisplay_frame_xy_rgb(pan[dispbufindex].pn[panelindex], x_coord, y_coord, red, green, blue);
        }
#endif
        return 0; //everything is OK
    }
    else if ((s_panels_orientation == 1) && (x_coord < LEDDISPLAY_WIDTH))
    { //Vertical
#if LEDDISPLAY_HEIGHT == 32
        uint16_t y = y_coord;
        uint8_t panoffset = (y >> 5);
        panelindex += panoffset; //fast devision by 32
        uint8_t yoffs = (panelindex + 1 - startpanel) << 5;
        y_coord -= yoffs;

        return 0; //everything is OK
    }
    else
    {
        if ((s_panels_orientation == 1) && (x_coord >= LEDDISPLAY_WIDTH))
        {   
            #if DEBUG_EN
            WARNING("Fuck! X coordinate outside coord. field");
            #endif
            ret = 3; //X Coord. outside coord. field
        }
        if ((s_panels_orientation == 0) && (y_coord >= LEDDISPLAY_HEIGHT))
        {
            #if DEBUG_EN
            WARNING("Fuck! Y coordinate outside coord. field");
            #endif
            ret = 4; //Y Coord. outside coord. field
        }
    }
#endif
#else
    //INFO("x_coord: %d, y_coord: %d, xoffs: %d, panelindex: %d, panoffset: %d", x_coord, y_coord, xoffs, panelindex, panoffset);
    if (alpha != 0)
        {
            leddisplay_frame_xy_argb(pan[dispbufindex].pn[panelindex], x_coord, y_coord, alpha, red, green, blue);
        }
        else
        {
            leddisplay_frame_xy_rgb(pan[dispbufindex].pn[panelindex], x_coord, y_coord, red, green, blue);
        }
#endif //END of LEDDISPLAY_WIDTH < 192
    return ret;
}

int leddisplay_panels_fill_random(int dispbufindex, uint8_t panelnum) {
    if((panelnum > NUMBER_OF_PANELS) || (dispbufindex < 0)){
        DEBUG("Incorrect index/panel given");
        return 1;   //Index/panel issue
    }
    if(panelnum == 0) {
        for(int i = 0; i < NUMBER_OF_PANELS; i++) {
            esp_fill_random(pan[dispbufindex].pn[i], sizeof(leddisplay_frame_t));
        }
    } else {
        esp_fill_random(pan[dispbufindex].pn[panelnum], sizeof(leddisplay_frame_t));
    }
    return 0;
}

int leddisplay_panels_fill_rgb(int dispbufindex, uint8_t panelnum, uint8_t red, uint8_t green, uint8_t blue) {
    if((panelnum > NUMBER_OF_PANELS) || (dispbufindex < 0)){
        DEBUG("Incorrect index/panel given");
        return 1;   //Index/panel issue
    }
    if(panelnum == 0) {
        for(int i = 0; i < NUMBER_OF_PANELS; i++) {
            leddisplay_frame_fill_rgb(pan[dispbufindex].pn[i], red, green, blue);
        }
    } else {
        uint8_t panelindex = panelnum - 1;
        leddisplay_frame_fill_rgb(pan[dispbufindex].pn[panelindex], red, green, blue);
    }
    return 0;
}

inline void leddisplay_frame_update_index(leddisplay_frame_t *p_frame, uint8_t index) {
    xSemaphoreTake(s_shift_complete_sem, portMAX_DELAY);
#if 0
    pan_sel_t psel;
	psel.by = 0;
	
    for (uint16_t x = 0; x < LEDDISPLAY_WIDTH; x++)
    {
        for (uint16_t y = 0; y < LEDDISPLAY_HEIGHT; y++)
        {
            const uint8_t *p_rgb = p_frame->yx[y][x];
            leddisplay_pixel_xy_rgb(x, y, p_rgb[0], p_rgb[1], p_rgb[2]);
        }
    }
#else
#if CONFIG_LEDDISPLAY_CORR_BRIGHT_STRICT || CONFIG_LEDDISPLAY_CORR_BRIGHT_MODIFIED
    for (uint16_t ix = 0; ix < NUMOF(p_frame->raw); ix++)
    {
        p_frame->raw[ix] = val2pwm(p_frame->raw[ix]);
    }
#endif

#ifdef CONFIG_LEDDISPLAY_TYPE_32X32x6_16SCAN
    //Setting OE bits according to panel index. This is done here, so later inside the loop we don't have to
    //Weh wannah makeh dis code fastah
    uint16_t currentpanelindex = 0;
    if(index & 0x1){ currentpanelindex |= BIT_OE0; }
    index = index >> 1;
    if(index & 0x1){ currentpanelindex |= BIT_OE1; }
    index = index >> 1;
    if(index & 0x1){ currentpanelindex |= BIT_OE2; }

    for (unsigned int y_coord = 0; y_coord < ROWS_PER_FRAME; y_coord++) // half height - 16 iterations
    {
        row_data_t *row_data = &s_frames[s_current_frame].rowdata[y_coord];

        for (int bitplane_ix = 0; bitplane_ix < COLOR_DEPTH_BITS; bitplane_ix++)  // color depth - 8 iterations
        {
            uint16_t mask = (1 << bitplane_ix); // 24 bit color

            // the destination for the pixel bitstream
            row_bit_t *rowbits = &row_data->rowbits[bitplane_ix]; //matrixUpdateFrames location to write to uint16_t's

            for (int x_coord = 0; x_coord < LEDDISPLAY_WIDTH; x_coord++) // row pixel width 64 iterations
            {
                int v = 0; // the output bitstream

                //----- START OF ~OE LOGIC -------
                //v |= currentpanelindex;

                /*  Need to disable OE after latch to hide row transition
                    Here OE bits are hight for active signal
                    After this, other than X_coord = 0, we will output a constant high for OE bits
                    Unless logic below tells otherwise
                */
                if (x_coord != 0) { v |= currentpanelindex; }

                // drive latch while shifting out last bit of RGB data
                // need to turn off OE one clock before latch, otherwise can get ghosting
                // OE here is inverted, so XOR is used
                if (x_coord == (PIXELS_PER_LATCH - 2)) { v ^= (currentpanelindex); }
                if (x_coord == (PIXELS_PER_LATCH - 1))
                {
                    v |= BIT_LAT;
                    v ^= currentpanelindex;
                }
                // turn off OE after brightness value is reached when displaying MSBs
                //OE here is inverted
                // MSBs always output normal brightness
                // LSB (!bitplane_ix) outputs normal brightness as MSB from previous row is being displayed
                if ( (((bitplane_ix > s_lsb_msb_transition_bit) || !bitplane_ix) && (x_coord >= s_brightness_val)) )
                {
                    v ^= currentpanelindex;
                }

                // special case for the bits *after* LSB through (s_lsb_msb_transition_bit) - OE is output after data is shifted, so need to set OE to fractional brightness
                if (bitplane_ix && (bitplane_ix <= s_lsb_msb_transition_bit))
                {
                    // divide brightness in half for each bit below s_lsb_msb_transition_bit
                    int lsbBrightness = s_brightness_val >> (s_lsb_msb_transition_bit - bitplane_ix + 1);
                    if ((x_coord <= lsbBrightness)) { v ^= currentpanelindex; } // For Brightness
                }
                //----- END OF ~OE LOGIC -------
                                //COLORS
                // top half
                const uint8_t *p_rgb_top = p_frame->yx[y_coord][x_coord];
                if (p_rgb_top[0] & mask) { v |= BIT_R1; }
                if (p_rgb_top[1] & mask) { v |= BIT_G1; }
                if (p_rgb_top[2] & mask) { v |= BIT_B1; }

                // bottom half
                const uint8_t *p_rgb_bot = p_frame->yx[y_coord + ROWS_PER_FRAME][x_coord];
                if (p_rgb_bot[0] & mask) { v |= BIT_R2; }
                if (p_rgb_bot[1] & mask) { v |= BIT_G2; }
                if (p_rgb_bot[2] & mask) { v |= BIT_B2; }

                // if there is no latch to hold address, output ADDX lines directly to GPIO and latch data at end of cycle
                // normally output current rows ADDX, special case for LSB, output previous row's ADDX (as previous row is being displayed for one latch cycle)
                int gpioRowAddress = (bitplane_ix == 0) ? y_coord - 1 : y_coord;

                if (gpioRowAddress & BIT(0)) { v |= BIT_A; } // 1
                if (gpioRowAddress & BIT(1)) { v |= BIT_B; } // 2
                if (gpioRowAddress & BIT(2)) { v |= BIT_C; } // 4
                if (gpioRowAddress & BIT(3)) { v |= BIT_D; } // 8
#if LEDDISPLAY_NEED_E_GPIO
                if (gpioRowAddress & BIT(4)) { v |= BIT_E; } // 16
#endif
                // 16 bit parallel mode
                // Save the calculated value to the bitplane memory in reverse order to account for I2S Tx FIFO mode1 ordering
                if ((x_coord % 2) != 0)
                {
                    rowbits->pixel[x_coord - 1] = v;
                }
                else
                {
                    rowbits->pixel[x_coord + 1] = v;
                } // end reordering

            } // end x_coord iteration
        } // colour depth loop (8)
    } // end row iteration
    leddisplay_pixel_update(CONFIG_BLOCK_EN);
#endif

#ifdef CONFIG_LEDDISPLAY_TYPE_64X32x3_16SCAN
    //In this case, LAT, ~OE, and CLK signals are gated with IC-s (Like 74HCT243) to the panels.
    //BIT_SEL0-2 are used to activate the IC to gate the signals through.
    uint16_t panelmask = 0;
    switch (index)
    {
    case 1:
        panelmask |= BIT_SEL0;
        break;
    case 2:
        panelmask |= BIT_SEL1;
        break;
    case 3:
        panelmask |= BIT_SEL2;
        break;
    }
    for (unsigned int y_coord = 0; y_coord < ROWS_PER_FRAME; y_coord++) // half height - 16 iterations
    {
        row_data_t *row_data = &s_frames[s_current_frame].rowdata[y_coord];

        for (int bitplane_ix = 0; bitplane_ix < COLOR_DEPTH_BITS; bitplane_ix++)  // color depth - 8 iterations
        {
            uint16_t mask = (1 << bitplane_ix); // 24 bit color

            // the destination for the pixel bitstream
            row_bit_t *rowbits = &row_data->rowbits[bitplane_ix]; //matrixUpdateFrames location to write to uint16_t's

            for (int x_coord = 0; x_coord < LEDDISPLAY_WIDTH; x_coord++) // row pixel width 64 iterations
            {
                int v = panelmask; // the output bitstream

                //----- START OF ~OE LOGIC -------
                //v |= panelmask;

                /*  Need to disable OE after latch to hide row transition
                    Here OE bits are hight for active signal
                    After this, other than X_coord = 0, we will output a constant high for OE bits
                    Unless logic below tells otherwise
                */
                if (x_coord == 0) { v |= BIT_OE; }

                // drive latch while shifting out last bit of RGB data
                // need to turn off OE one clock before latch, otherwise can get ghosting
                //if (x_coord == (PIXELS_PER_LATCH - 1)) { v |= (BIT_LAT | BIT_OE0); }
                if (x_coord == (PIXELS_PER_LATCH - 2)) { v |= BIT_OE; }
                if (x_coord == (PIXELS_PER_LATCH - 1))
                {
                    v |= BIT_LAT;
                    v |= BIT_OE;
                }

                // turn off OE after brightness value is reached when displaying MSBs
                // MSBs always output normal brightness
                // LSB (!bitplane_ix) outputs normal brightness as MSB from previous row is being displayed
                if ( ((bitplane_ix > s_lsb_msb_transition_bit) || !bitplane_ix) && (x_coord >= s_brightness_val) )
                {
                    v |= BIT_OE; // For Brightness
                }

                // special case for the bits *after* LSB through (s_lsb_msb_transition_bit) - OE is output after data is shifted, so need to set OE to fractional brightness
                if (bitplane_ix && (bitplane_ix <= s_lsb_msb_transition_bit))
                {
                    // divide brightness in half for each bit below s_lsb_msb_transition_bit
                    int lsbBrightness = s_brightness_val >> (s_lsb_msb_transition_bit - bitplane_ix + 1);
                    if (x_coord >= lsbBrightness) { v |= BIT_OE0; } // For Brightness
                }
                //----- END OF ~OE LOGIC -------
                                //COLORS
                // top half
                const uint8_t *p_rgb_top = p_frame->yx[y_coord][x_coord];
                if (p_rgb_top[0] & mask) { v |= BIT_R1; }
                if (p_rgb_top[1] & mask) { v |= BIT_G1; }
                if (p_rgb_top[2] & mask) { v |= BIT_B1; }

                // bottom half
                const uint8_t *p_rgb_bot = p_frame->yx[y_coord + ROWS_PER_FRAME][x_coord];
                if (p_rgb_bot[0] & mask) { v |= BIT_R2; }
                if (p_rgb_bot[1] & mask) { v |= BIT_G2; }
                if (p_rgb_bot[2] & mask) { v |= BIT_B2; }

                // if there is no latch to hold address, output ADDX lines directly to GPIO and latch data at end of cycle
                // normally output current rows ADDX, special case for LSB, output previous row's ADDX (as previous row is being displayed for one latch cycle)
                int gpioRowAddress = (bitplane_ix == 0) ? y_coord - 1 : y_coord;

                if (gpioRowAddress & BIT(0)) { v |= BIT_A; } // 1
                if (gpioRowAddress & BIT(1)) { v |= BIT_B; } // 2
                if (gpioRowAddress & BIT(2)) { v |= BIT_C; } // 4
                if (gpioRowAddress & BIT(3)) { v |= BIT_D; } // 8
#if LEDDISPLAY_NEED_E_GPIO
                if (gpioRowAddress & BIT(4)) { v |= BIT_E; } // 16
#endif
                // 16 bit parallel mode
                // Save the calculated value to the bitplane memory in reverse order to account for I2S Tx FIFO mode1 ordering
                if ((x_coord % 2) != 0)
                {
                    rowbits->pixel[x_coord - 1] = v;
                }
                else
                {
                    rowbits->pixel[x_coord + 1] = v;
                } // end reordering

            } // end x_coord iteration
        } // colour depth loop (8)
    } // end row iteration
    leddisplay_pixel_update(CONFIG_BLOCK_EN);
#endif
#endif // for #if(0)
}

IRAM_ATTR void DisplayRender_FPS(unsigned int FPS)
{
    /*
    Logic: xDispQ, the queue handling the back and frontbuffers ([a] and [b])
            Queue size is 2.
            We check how many items are on the queue.
            Whene there is only 1 item on the queue, this means that a next frame is not ready.
            We keep rendering that frame on the queue, until there is a "next" item on queue, which can be rendered
            If we detect this, after we finish rendering the frame, we remove it from the queue making place for the next frame.
            
            While we are rendering a frame we keep it blocked with a semaphore for extra safety.
        
*/
    uint8_t period = (uint16_t)(1000/FPS);
    while (1)
    {
        uint32_t prevTick = xTaskGetTickCount();
        //uint8_t astat = leddisplay_panels_get_buffer_status(a);
        //uint8_t bstat = leddisplay_panels_get_buffer_status(b);
        int torender = -1;
        int onQ = uxQueueMessagesWaiting(xDispQ);

        if (onQ == 0) //should not happen, only on startup
        {
            osSleep(10);
            continue;
        }
        if (onQ > 0)
        {
            xQueuePeek(xDispQ, &(torender), 0); //we Use peek, so the item is not removed, if there is error.
            if (torender > -1)
            {
                torender = (uint8_t)torender;
                if (leddisplay_panels_block_MS_dispbuf(torender, 0) == ESP_OK)
                {
                    //INFO("<>");
                    leddisplay_push_single_frame(pan[torender].pn[0]); //actual rendering
                    #if DEBUG_EN
                    INFO("-> (%s)", torender ? "b" : "a");
                    #endif
                    leddisplay_panels_set_lastrendered(torender);
                    if (onQ == 2) //we are only removing an item from Queue if the next one is already on it.
                    {
                        xQueueReceive(xDispQ, &(torender), 0); //We use this only to remove the item from the queue
                        leddisplay_panels_unblock_dispbuf(torender);
                    }
                    //leddisplay_panels_unblock_dispbuf(torender);
                    vTaskDelayUntil(&prevTick, MS2TICKS(period));
                    continue;
                }
                else
                {
                    int lastrender = leddisplay_panels_get_lastrendered();
                    if (lastrender == torender)
                    {
                        //INFO("<>");
                        leddisplay_push_single_frame(pan[torender].pn[0]); //actual rendering
                        #if DEBUG_EN
                        INFO("-> (%s)", torender ? "b" : "a");
                        #endif
                        //leddisplay_panels_set_lastrendered(torender);
                        if (onQ == 2) //we are only removing an item from Queue if the next one is already on it.
                        {
                            xQueueReceive(xDispQ, &(torender), 0); //We use this only to remove the item from the queue
                            leddisplay_panels_unblock_dispbuf(torender);
                        }
                        //vTaskDelayUntil(&prevTick, MS2TICKS(period));
                        continue;
                    }
                    #if DEBUG_EN
                    INFO("DAFUQ (%s)", torender ? "b" : "a");
                    #endif
                    //if(leddisplay_panels_unblock_dispbuf(lastrender) != ESP_OK) {
                    if (leddisplay_panels_block_MS_dispbuf(torender, 0) == ESP_OK)
                    {
                        //INFO("<>");
                        leddisplay_push_single_frame(pan[torender].pn[0]); //actual rendering
                        #if DEBUG_EN
                        INFO("-> (%s)", torender ? "b" : "a");
                        #endif
                        leddisplay_panels_set_lastrendered(torender);
                        if (onQ == 2) //we are only removing an item from Queue if the next one is already on it.
                        {
                            xQueueReceive(xDispQ, &(torender), 0); //We use this only to remove the item from the queue
                            leddisplay_panels_unblock_dispbuf(torender);
                        }
                        vTaskDelayUntil(&prevTick, MS2TICKS(period));
                        continue;
                    }
                    continue;
                }
            }
            else
            {
                xQueueReceive(xDispQ, &(torender), 0); //Index incorrect (-1), we remove it from the
                #if DEBUG_EN
                WARNING("Render, ERR (%d)", torender);
                #endif
                continue;
            }
        }
    }
    vTaskDelete(NULL);
}

void Pause_DisplayRender(void) {
    xSemaphoreTake(xlastrenderedSem, portMAX_DELAY);  //used for blocking render, until it is taken
}

void Resume_DisplayRender(void) {
    xSemaphoreGive(xlastrenderedSem);  //used for blocking render, until it is taken
}

/* I hate when I can not comment a larger amount of code section if it has a comment in it...:(
void DisplayRender_period()
{
*/
    /*
    Logic: xDispQ, the queue handling the back and frontbuffers ([a] and [b])
            Queue size is 2.
            We check how many items are on the queue.
            Whene there is only 1 item on the queue, this means that a next frame is not ready.
            We keep rendering that frame on the queue, until there is a "next" item on queue, which can be rendered
            If we detect this, after we finish rendering the frame, we remove it from the queue making place for the next frame.
            
            While we are rendering a frame we keep it blocked with a semaphore for extra safety.
        
*/
/*
    while (1)
    {
        xSemaphoreTake(xStatusSem, portMAX_DELAY);  //used for blocking render, until it is taken
        xSemaphoreGive(xStatusSem);  //Give it back immediatelly, so it can be set again if needed.
        uint32_t prevTick = xTaskGetTickCount();
        //uint8_t astat = leddisplay_panels_get_buffer_status(a);
        //uint8_t bstat = leddisplay_panels_get_buffer_status(b);
        int torender = -1;
        int onQ = uxQueueMessagesWaiting(xDispQ);

        if (onQ == 0) //should not happen, only on startup
        {
            osSleep(10);
            continue;
        }
        if (onQ > 0)
        {
            xQueuePeek(xDispQ, &(torender), 0); //we Use peek, so the item is not removed, if there is error.
            if (torender > -1)
            {
                torender = (uint8_t)torender;
                if (leddisplay_panels_block_MS_dispbuf(torender, 0) == ESP_OK)
                {
                    //INFO("<>");
                    leddisplay_panels_update(torender, 0); //actual rendering
                    #if DEBUG_EN
                    INFO("-> (%s)", torender ? "b" : "a");
                    #endif
                    leddisplay_panels_set_lastrendered(torender);
                    if (onQ == 2) //we are only removing an item from Queue if the next one is already on it.
                    {
                        xQueueReceive(xDispQ, &(torender), 0); //We use this only to remove the item from the queue
                        leddisplay_panels_unblock_dispbuf(torender);
                    }
                    //leddisplay_panels_unblock_dispbuf(torender);
                    vTaskDelayUntil(&prevTick, MS2TICKS(FRAME_TIME));
                    continue;
                }
                else
                {
                    int lastrender = leddisplay_panels_get_lastrendered();
                    if (lastrender == torender)
                    {
                        //INFO("<>");
                        leddisplay_panels_update(torender, 0); //actual rendering
                        #if DEBUG_EN
                        INFO("-> (%s)", torender ? "b" : "a");
                        #endif
                        //leddisplay_panels_set_lastrendered(torender);
                        if (onQ == 2) //we are only removing an item from Queue if the next one is already on it.
                        {
                            xQueueReceive(xDispQ, &(torender), 0); //We use this only to remove the item from the queue
                            leddisplay_panels_unblock_dispbuf(torender);
                        }
                        vTaskDelayUntil(&prevTick, MS2TICKS(FRAME_TIME));
                        continue;
                    }
                    #if DEBUG_EN
                    INFO("DAFUQ (%s)", torender ? "b" : "a");
                    #endif
                    //if(leddisplay_panels_unblock_dispbuf(lastrender) != ESP_OK) {
                    if (leddisplay_panels_block_MS_dispbuf(torender, 0) == ESP_OK)
                    {
                        //INFO("<>");
                        leddisplay_panels_update(torender, 0); //actual rendering
                        #if DEBUG_EN
                        INFO("-> (%s)", torender ? "b" : "a");
                        #endif
                        leddisplay_panels_set_lastrendered(torender);
                        if (onQ == 2) //we are only removing an item from Queue if the next one is already on it.
                        {
                            xQueueReceive(xDispQ, &(torender), 0); //We use this only to remove the item from the queue
                            leddisplay_panels_unblock_dispbuf(torender);
                        }
                        vTaskDelayUntil(&prevTick, MS2TICKS(FRAME_TIME));
                        continue;
                    }
                    osSleep(10);
                    continue;
                }
            }
            else
            {
                xQueueReceive(xDispQ, &(torender), 0); //Index incorrect (-1), we remove it from the
                #if DEBUG_EN
                WARNING("Render, ERR (%d)", torender);
                #endif
                continue;
            }
        }
    }
    vTaskDelete(NULL);
}
*/

void DisplayRender_period()
{
    /*
    Logic: xDispQ, the queue handling the back and frontbuffers ([a] and [b])
            Queue size is 2.
            We check how many items are on the queue.
            Whene there is only 1 item on the queue, this means that a next frame is not ready.
            We keep rendering that frame on the queue, until there is a "next" item on queue, which can be rendered
            If we detect this, after we finish rendering the frame, we remove it from the queue making place for the next frame.
            
            While we are rendering a frame we keep it blocked with a semaphore for extra safety.
        
*/
    while (1)
    {
        int onQ = uxQueueMessagesWaiting(xDispQ);
        if (onQ == 0) //When the Queue is empty, we wait for it to have an item.
        {
            osSleep(10);
            continue;
        }
        xSemaphoreTake(xlastrenderedSem, portMAX_DELAY);  //used for blocking render, until it is taken
        //uint32_t prevTick = xTaskGetTickCount();
        //uint8_t astat = leddisplay_panels_get_buffer_status(a);
        //uint8_t bstat = leddisplay_panels_get_buffer_status(b);
        int torender = -1;
        if (onQ > 0)
        {
            xQueuePeek(xDispQ, &(torender), 0);
            if (torender > -1)
            {
                torender = (uint8_t)torender;
                if (leddisplay_panels_block_dispbuf(torender) == ESP_OK)
                {
                    //INFO("<>");
                    leddisplay_panels_update(torender, 0); //actual rendering
                    /*
                    #if DEBUG_EN
                    INFO("-> (%s)", torender ? "b" : "a");
                    #endif
                    */
                    //leddisplay_panels_set_lastrendered(torender);
                    leddisplay_panels_unblock_dispbuf(torender);
                    //leddisplay_panels_unblock_dispbuf(torender);
                    //vTaskDelayUntil(&prevTick, MS2TICKS(FRAME_TIME));
                    leddisplay_panels_set_buffer_status(torender, 3);
                    xQueueReceive(xDispQ, &(torender), 0);
                    xSemaphoreGive(xlastrenderedSem);
                    #if DEBUG_EN
                    INFO("-> (%s)", torender ? "b" : "a");
                    #endif
                    osSleep(10);
                    continue;
                }
            }
            else
            {   
                xQueueReceive(xDispQ, &(torender), 0); //Index incorrect (-1), we remove it from the Queue, and move on with our lives
                #if DEBUG_EN
                WARNING("Render, ERR (%d)", torender);
                #endif
                continue;
            }
        }
    }
    vTaskDelete(NULL);
}

IRAM_ATTR void DisplayRender_Cont(void)
{
    /*
    Logic: xDispQ, the queue handling the back and frontbuffers ([a] and [b])
            Queue size is 2.
            We check how many items are on the queue.
            Whene there is only 1 item on the queue, this means that a next frame is not ready.
            We keep rendering that frame on the queue, until there is a "next" item on queue, which can be rendered
            If we detect this, after we finish rendering the frame, we remove it from the queue making place for the next frame.
            
            While we are rendering a frame we keep it blocked with a semaphore for extra safety.
        
*/
    while (1)
    {
        //uint32_t prevTick = xTaskGetTickCount();
        //uint8_t astat = leddisplay_panels_get_buffer_status(a);
        //uint8_t bstat = leddisplay_panels_get_buffer_status(b);
        int torender = -1;
        int onQ = uxQueueMessagesWaiting(xDispQ);

        if (onQ == 0) //should not happen, only on startup
        {
            //osSleep(10);
            continue;
        }
        if (onQ > 0)
        {
            xQueuePeek(xDispQ, &(torender), 0); //we Use peek, so the item is not removed, if there is error.
            if (torender > -1)
            {
                torender = (uint8_t)torender;
                if (leddisplay_panels_block_MS_dispbuf(torender, 0) == ESP_OK)
                {
                    //INFO("<>");
                    leddisplay_panels_update(torender, 0); //actual rendering
                    #if DEBUG_EN
                    INFO("-> (%s)", torender ? "b" : "a");
                    #endif
                    //leddisplay_panels_set_lastrendered(torender);
                    if (onQ == 2) //we are only removing an item from Queue if the next one is already on it.
                    {
                        xQueueReceive(xDispQ, &(torender), 0); //We use this only to remove the item from the queue
                        //leddisplay_panels_unblock_dispbuf(torender);
                    }
                    leddisplay_panels_unblock_dispbuf(torender);
                    //vTaskDelayUntil(&prevTick, MS2TICKS(period));
                    continue;
                }
                else
                {
                    /*
                    int lastrender = leddisplay_panels_get_lastrendered();
                    if (lastrender == torender)
                    {
                        //INFO("<>");
                        leddisplay_panels_update(torender, 0); //actual rendering
                        INFO("-> (%s)", torender ? "b" : "a");
                        //leddisplay_panels_set_lastrendered(torender);
                        if (onQ == 2) //we are only removing an item from Queue if the next one is already on it.
                        {
                            xQueueReceive(xDispQ, &(torender), 0); //We use this only to remove the item from the queue
                            leddisplay_panels_unblock_dispbuf(torender);
                        }
                        //vTaskDelayUntil(&prevTick, MS2TICKS(period));
                        continue;
                    }
                    INFO("DAFUQ (%s)", torender ? "b" : "a");
                    //if(leddisplay_panels_unblock_dispbuf(lastrender) != ESP_OK) {
                    if (leddisplay_panels_block_MS_dispbuf(torender, 0) == ESP_OK)
                    {
                        //INFO("<>");
                        leddisplay_panels_update(torender, 0); //actual rendering
                        INFO("-> (%s)", torender ? "b" : "a");
                        leddisplay_panels_set_lastrendered(torender);
                        if (onQ == 2) //we are only removing an item from Queue if the next one is already on it.
                        {
                            xQueueReceive(xDispQ, &(torender), 0); //We use this only to remove the item from the queue
                            leddisplay_panels_unblock_dispbuf(torender);
                        }
                        //vTaskDelayUntil(&prevTick, MS2TICKS(period));
                        continue;
                    }
                    */
                    continue;
                }
            }
            else
            {
                xQueueReceive(xDispQ, &(torender), 0); //Index incorrect (-1), we remove it from the
                #if DEBUG_EN
                WARNING("Render, ERR (%d)", torender);
                #endif
                continue;
            }
        }
    }
    vTaskDelete(NULL);
}

void sDisplayUpdateTask(void *pParam)
{
    #ifdef CONFIG_DISPLAY_UPDATE_CONT
    INFO("Created Display Task: Continuous with frametime");
    DisplayRender_period();
    #endif
    #ifdef CONFIG_DISPLAY_UPDATE_FPS
    DisplayRender_FPS(CONFIG_LEDDISPLAY_MIN_FRAME_RATE);
    #endif
    vTaskDelete(NULL);
}

IRAM_ATTR int DisplayBufferTake(void)
{
    globalTicks = xTaskGetTickCount();
    int buf = getNextToRender();
    if(buf != -1) {
        buf = (unsigned int) buf;
    }
    else
    {   
        INFO("getnextrender error");
        return -1;
    }
    leddisplay_panels_block_dispbuf(buf);
    #if DEBUG_EN
    INFO("Buffer taken: %s", buf ? "b" : "a");
    #endif
    leddisplay_panels_set_buffer_status(buf, 0);
    return buf;
}
/*
IRAM_ATTR esp_err_t DisplayBufferFinish(int dispbufindex)
{   
        if(dispbufindex < 0) {
            WARNING("incorrect bufindex");
            return ESP_FAIL;
        }

        esp_err_t unblock = leddisplay_panels_unblock_dispbuf(dispbufindex);
        if(unblock == ESP_OK) {
            leddisplay_panels_set_buffer_status(dispbufindex, 1); // ready to render
        }
        return unblock;
}
*/
IRAM_ATTR esp_err_t DisplayBufferFinish(int dispbufindex)
{
    if(dispbufindex >= 0) {
        esp_err_t unblock = leddisplay_panels_unblock_dispbuf(dispbufindex);
        if (unblock == ESP_OK)
        {
        #if DEBUG_EN
            INFO("Release buffer: %s", dispbufindex ? "b" : "a");
        #endif
            leddisplay_panels_set_buffer_status(dispbufindex, 1); // ready to render
        }
        vTaskDelayUntil(&globalTicks, MS2TICKS(FRAME_TIME));
        DisplayBufferSetNextTo(dispbufindex);
        return unblock;
    }
    esp_err_t err = ESP_FAIL;
    #if DEBUG_EN
    INFO("FAILED TO FINISH");
    #endif
    return err;
}

IRAM_ATTR int DisplayBufferSetNextTo(int dispbufindex)
{
    if(dispbufindex != -1) {
        int i = dispbufindex;
        xQueueSendToBack(xDispQ, (void *) &i, portMAX_DELAY);
        return 0;
    }
    #if DEBUG_EN
    WARNING("Buffer set next to: incorrect bufindex");
    #endif
    return -1;
}

IRAM_ATTR void DisplayBufferQclear(void) {
    int recv;
    for(int i = 0; i < 2; i++)
    {
        xQueueReceive(xDispQ, &recv, 0);
    }
}

IRAM_ATTR int getNextToRender(void)
{   
    #if NUM_DISP_BUFFER < 2
    return a;
    #else // #if NUM_DISP_BUFFER < 2
    uint32_t onQ = uxQueueMessagesWaiting(xDispQ);
    uint32_t i;
    switch(onQ) {
        case 0: //when you empty the queue, 
            return a;
        case 1:
            xQueuePeek(xDispQ, &( i ), 0);
            return i ? a : b;
        case 2: //queue is full
            xQueuePeek(xDispQ, &( i ), 0);
            return i;
            //return -1;
    }
    #endif  // #if NUM_DISP_BUFFER < 2
    return -1;
}
#endif

void leddisplay_shutdown(void)
{
    INFO("Display shutdown");
    i2s_parallel_stop(&I2S1);
    if (s_frames != NULL)
    {
        heap_caps_free(s_frames);
        s_frames = NULL;
    }
    if (s_dmadesc_a != NULL)
    {
        heap_caps_free(s_dmadesc_a);
        s_dmadesc_a = NULL;
    }
    if (s_dmadesc_b != NULL)
    {
        heap_caps_free(s_dmadesc_b);
        s_dmadesc_b = NULL;
    }
#if CONFIG_SUPPORT_STATIC_ALLOCATION
#else
    vSemaphoreDelete(s_shift_complete_sem);
#endif


}

/* *********************************************************************************************** */

int leddisplay_set_brightness(int brightness)
{
    const int last_brightness_percent = s_brightness_percent;

    if (brightness <= 0)
    {
        s_brightness_val = 0;
        s_brightness_percent = 0;
    }
    else if (brightness >= 100)
    {
        s_brightness_val = LEDDISPLAY_WIDTH;
        s_brightness_percent = 100;
    }
    else
    {
        s_brightness_percent = brightness;

        // scale brightness percent to value for this display: 0..100% --> 0..LEDDISPLAY_WIDTH
        //const int brightness_val = ((((1000 * LEDDISPLAY_WIDTH) * brightness) + 500) / 1000) / 100;
        int brightness_val = (((1000 * LEDDISPLAY_WIDTH) * brightness) / 1000) / 100;

#if CONFIG_LEDDISPLAY_CORR_BRIGHT_STRICT

        const int f = 256 / LEDDISPLAY_WIDTH;
        s_brightness_val = val2pwm(brightness_val * f) / f;

#elif CONFIG_LEDDISPLAY_CORR_BRIGHT_MODIFIED

        const int f = 256 / LEDDISPLAY_WIDTH;
        const int lut = val2pwm(brightness_val * f) / f;
        if (lut <= 0)
        {
            s_brightness_val = 1;
        }
        else
        {
            s_brightness_val = lut;
        }

#else
        s_brightness_val = brightness_val;
#endif
    }

    return last_brightness_percent;
}

int leddisplay_get_brightness(void)
{
    return s_brightness_percent;
}

/* *********************************************************************************************** */

void leddisplay_pixel_xy_rgb(uint16_t x_coord, uint16_t y_coord, uint8_t red, uint8_t green, uint8_t blue)
{
    if ( (x_coord >= LEDDISPLAY_WIDTH) || (y_coord >= LEDDISPLAY_HEIGHT) )
    {
        return;
    }

    // What half of the HUB75 panel are we painting to?
    bool paint_top_half = true;
    if ( y_coord > (ROWS_PER_FRAME - 1) ) // co-ords start at zero, y_coord = 15 = 16 (rows per frame)
    {
        y_coord -= ROWS_PER_FRAME; // if it's 16, subtract 16. Array position 0 again.
        paint_top_half = false;
    }

#if CONFIG_LEDDISPLAY_CORR_BRIGHT_STRICT || CONFIG_LEDDISPLAY_CORR_BRIGHT_MODIFIED
    red   = val2pwm(red);
    green = val2pwm(green);
    blue  = val2pwm(blue);
#endif

    row_data_t *row_data = &s_frames[s_current_frame].rowdata[y_coord];

    for (int bitplane_ix = 0; bitplane_ix < COLOR_DEPTH_BITS; bitplane_ix++)  // color depth - 8 iterations
    {
        // the destination for the pixel bitstream
        row_bit_t *rowbits = &row_data->rowbits[bitplane_ix]; //matrixUpdateFrames location to write to uint16_t's

        int v = 0; // the output bitstream

        // if there is no latch to hold address, output ADDX lines directly to GPIO and latch data at end of cycle
        // normally output current rows ADDX, special case for LSB, output previous row's ADDX (as previous row is being displayed for one latch cycle)
        int gpioRowAddress = (bitplane_ix == 0) ? y_coord - 1 : y_coord;

        if (gpioRowAddress & BIT(0)) { v |= BIT_A; } // 1
        if (gpioRowAddress & BIT(1)) { v |= BIT_B; } // 2
        if (gpioRowAddress & BIT(2)) { v |= BIT_C; } // 4
        if (gpioRowAddress & BIT(3)) { v |= BIT_D; } // 8
#if LEDDISPLAY_NEED_E_GPIO
        if (gpioRowAddress & BIT(4)) { v |= BIT_E; } // 16
#endif

        // need to disable OE after latch to hide row transition
        if (x_coord == 0)
        {
            v |= BIT_OE0;
        }


        // drive latch while shifting out last bit of RGB data
        // need to turn off OE one clock before latch, otherwise can get ghosting
        if (x_coord == (PIXELS_PER_LATCH - 1))
        {
            v |= (BIT_LAT | BIT_OE0);
        }

        // turn off OE after brightness value is reached when displaying MSBs
        // MSBs always output normal brightness
        // LSB (bitplane_ix == 0) outputs normal brightness as MSB from previous row is being displayed
        if ( ((bitplane_ix == 0) || (bitplane_ix > s_lsb_msb_transition_bit)) && (x_coord >= s_brightness_val) )
        {
            v |= BIT_OE0; // for Brightness
        }

        // special case for the bits *after* LSB through (s_lsb_msb_transition_bit) - OE is output
        // after data is shifted, so need to set OE to fractional brightness
        if (bitplane_ix && (bitplane_ix <= s_lsb_msb_transition_bit))
        {
            // divide brightness in half for each bit below s_lsb_msb_transition_bit
            int lsbBrightness = s_brightness_val >> (s_lsb_msb_transition_bit - bitplane_ix + 1);
            if (x_coord >= lsbBrightness)
            {
                v |= BIT_OE0; // for Brightness
            }
        }

        // When using the Adafruit drawPixel, we only have one pixel co-ordinate and colour to draw
        // (duh) so we can't paint a top and bottom half (or whatever row split the panel is) at the
        // same time.  Need to be smart and check the DMA buffer to see what the other half thinks
        // (pun intended) and persist this when we refresh.
        // The DMA buffer order has also been reversed (refer to the last code in this function) so
        // we have to check for this and check the correct position of the uint16_t data.
        int16_t tmp_x_coord = x_coord;
        if ((x_coord % 2) != 0)
        {
            tmp_x_coord -= 1;
        }
        else
        {
            tmp_x_coord += 1;
        } // end reordering

        uint8_t mask = BIT(bitplane_ix); // 8 bit color

        // need to copy what the RGB status is for the bottom pixels
        if (paint_top_half)
        {

           // Set the color of the pixel of interest
           if (green & mask) { v |= BIT_G1; }
           if (blue  & mask) { v |= BIT_B1; }
           if (red   & mask) { v |= BIT_R1; }

           // Persist what was painted to the other half of the frame equiv. pixel
           if (rowbits->pixel[tmp_x_coord] & BIT_R2) { v |= BIT_R2; }
           if (rowbits->pixel[tmp_x_coord] & BIT_G2) { v |= BIT_G2; }
           if (rowbits->pixel[tmp_x_coord] & BIT_B2) { v |= BIT_B2; }
        }
        // do it the other way around
        else
        {
            // color to set
            if (red   & mask) { v |= BIT_R2; }
            if (green & mask) { v |= BIT_G2; }
            if (blue  & mask) { v |= BIT_B2; }

            // copy
            if (rowbits->pixel[tmp_x_coord] & BIT_R1) { v |= BIT_R1; }
            if (rowbits->pixel[tmp_x_coord] & BIT_G1) { v |= BIT_G1; }
            if (rowbits->pixel[tmp_x_coord] & BIT_B1) { v |= BIT_B1; }

        } // paint


        // 16 bit parallel mode
        // save the calculated value to the bitplane memory in reverse order to account for I2S Tx FIFO mode1 ordering
        if( (x_coord % 2) != 0)
        {
            rowbits->pixel[tmp_x_coord] = v;
        }
        else
        {
            rowbits->pixel[tmp_x_coord] = v;
        } // end reordering

    } // color depth loop (8)
}

void leddisplay_pixel_fill_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
#if 0
    for (uint16_t y = 0; y < LEDDISPLAY_HEIGHT; y++)
    {
        for (uint16_t x = 0; x < LEDDISPLAY_WIDTH; x++)
        {
            leddisplay_pixel_xy_rgb(x, y, red, green, blue);
        }
    }
#else
#if CONFIG_LEDDISPLAY_CORR_BRIGHT_STRICT || CONFIG_LEDDISPLAY_CORR_BRIGHT_MODIFIED
    red   = val2pwm(red);
    green = val2pwm(green);
    blue  = val2pwm(blue);
#endif

    for (unsigned int y_coord = 0; y_coord < ROWS_PER_FRAME; y_coord++) // half height - 16 iterations
    {
        row_data_t *row_data = &s_frames[s_current_frame].rowdata[y_coord];

        for (int bitplane_ix = 0; bitplane_ix < COLOR_DEPTH_BITS; bitplane_ix++)  // color depth - 8 iterations
        {
            uint16_t mask = (1 << bitplane_ix); // 24 bit color

            // the destination for the pixel bitstream
            row_bit_t *rowbits = &row_data->rowbits[bitplane_ix]; //matrixUpdateFrames location to write to uint16_t's

            for (int x_coord = 0; x_coord < LEDDISPLAY_WIDTH; x_coord++) // row pixel width 64 iterations
            {
                int v = 0; // the output bitstream

                // if there is no latch to hold address, output ADDX lines directly to GPIO and latch data at end of cycle
                // normally output current rows ADDX, special case for LSB, output previous row's ADDX (as previous row is being displayed for one latch cycle)
                int gpioRowAddress = (bitplane_ix == 0) ? y_coord - 1 : y_coord;

                if (gpioRowAddress & BIT(0)) { v |= BIT_A; } // 1
                if (gpioRowAddress & BIT(1)) { v |= BIT_B; } // 2
                if (gpioRowAddress & BIT(2)) { v |= BIT_C; } // 4
                if (gpioRowAddress & BIT(3)) { v |= BIT_D; } // 8
#if LEDDISPLAY_NEED_E_GPIO
                if (gpioRowAddress & BIT(4)) { v |= BIT_E; } // 16
#endif
                // need to disable OE after latch to hide row transition
                if (x_coord == 0) { v |= BIT_OE0; }

                // drive latch while shifting out last bit of RGB data
                // need to turn off OE one clock before latch, otherwise can get ghosting
                if (x_coord == (PIXELS_PER_LATCH - 1)) { v |= (BIT_LAT | BIT_OE0); }

                // turn off OE after brightness value is reached when displaying MSBs
                // MSBs always output normal brightness
                // LSB (!bitplane_ix) outputs normal brightness as MSB from previous row is being displayed
                if ( ((bitplane_ix > s_lsb_msb_transition_bit) || !bitplane_ix) && (x_coord >= s_brightness_val) )
                {
                    v |= BIT_OE0; // For Brightness
                }

                // special case for the bits *after* LSB through (s_lsb_msb_transition_bit) - OE is output after data is shifted, so need to set OE to fractional brightness
                if (bitplane_ix && (bitplane_ix <= s_lsb_msb_transition_bit))
                {
                    // divide brightness in half for each bit below s_lsb_msb_transition_bit
                    int lsbBrightness = s_brightness_val >> (s_lsb_msb_transition_bit - bitplane_ix + 1);
                    if (x_coord >= lsbBrightness) { v |= BIT_OE0; } // For Brightness
                }

                // top and bottom half colours
                if (red    & mask) { v |= (BIT_R1 | BIT_R2); }
                if (green  & mask) { v |= (BIT_G1 | BIT_G2); }
                if (blue   & mask) { v |= (BIT_B1 | BIT_B2); }

                // 16 bit parallel mode
                // Save the calculated value to the bitplane memory in reverse order to account for I2S Tx FIFO mode1 ordering
                if ((x_coord % 2) != 0)
                {
                    rowbits->pixel[x_coord - 1] = v;
                }
                else
                {
                    rowbits->pixel[x_coord + 1] = v;
                } // end reordering

            } // end x_coord iteration
        } // colour depth loop (8)
    } // end row iteration
#endif
}

/* *********************************************************************************************** */

inline void leddisplay_frame_xy_rgb(leddisplay_frame_t *p_frame, uint16_t x_coord, uint16_t y_coord, uint8_t red, uint8_t green, uint8_t blue)
{
    #ifndef NUMBER_OF_PANELS
    if ( (x_coord >= LEDDISPLAY_WIDTH) || (y_coord >= LEDDISPLAY_HEIGHT) )
    {
        return;
    }
    #endif
    p_frame->yx[y_coord][x_coord][0] = red;
    p_frame->yx[y_coord][x_coord][1] = green;
    p_frame->yx[y_coord][x_coord][2] = blue;
}

inline void leddisplay_frame_get_rgb_xy(leddisplay_frame_t *p_frame, uint16_t x_coord, uint16_t y_coord, uint8_t *red, uint8_t *green, uint8_t *blue)
{
    #ifndef NUMBER_OF_PANELS
    if ( (x_coord >= LEDDISPLAY_WIDTH) || (y_coord >= LEDDISPLAY_HEIGHT) )
    {
        return;
    }
    #endif
    *red = p_frame->yx[y_coord][x_coord][0];
    *green = p_frame->yx[y_coord][x_coord][1];
    *blue = p_frame->yx[y_coord][x_coord][2];
}



inline void leddisplay_frame_xy_argb(leddisplay_frame_t *p_frame, uint16_t x_coord, uint16_t y_coord, uint8_t alpha, uint8_t red, uint8_t green, uint8_t blue)
{
    #ifndef NUMBER_OF_PANELS
    if ( (x_coord >= LEDDISPLAY_WIDTH) || (y_coord >= LEDDISPLAY_HEIGHT) )
    {
        return;
    }
    #endif
    /**
  	@note 
	The equation below is made from the original color mixing equation, to avoid using
    floating point (original equation has a devision by 255), I used a right shift by 8
    (devision by 256)
	!!!This equation assumes that background is always not transparent!!!
	Accuracy of this is very good. 95% of time the 8bits of the color components are the same.
	Difference can maybe occur in the last LSB bit.

    Edge cases: Normally after float devision, result is rounded, in this case, there is no need,
    but we are slightly off, the value is generally less, by only 1 LSB place.
    In lower bit-depth cases, this, where for example where the original float would be
    10011111,9, which rounds up to 10100000, could result in incorrect colour.
    To correct for this, the (255-fg.A) part in the equiation is modified to
    (256-fg.A). This component should be 0, when fg.A is 0, but value is so small,
    it is not noticable and makes a better result, then before.
	
	This function should make the following equation (with order of execution)
	________III.________
				_____II._____
				___I.___     __IV.__
	((fg*fg.A)+(256-fg.A)*bg)/256
	The final devide by 256 is supplemented by a shifting to right by 8.
	Numerical testing shows this method is accurate enough.
	NO need to use slow floating point here.
  
	Where fg is the layers color component (R, G or B), bg is the background color
	Alpha is the alpha component of layer, bg has no alpha (non transparent)
    */
    //take existing color values at coordinate

    uint8_t r = p_frame->yx[y_coord][x_coord][0];
    uint8_t g = p_frame->yx[y_coord][x_coord][1];
    uint8_t b = p_frame->yx[y_coord][x_coord][2];

    uint8_t ai = 256 - alpha;

    uint32_t R = ((red * alpha) + (ai * r)) >> 8;
	uint32_t G = ((green * alpha) + (ai * g)) >> 8;
	uint32_t B = ((blue * alpha) + (ai * b)) >> 8;

    p_frame->yx[y_coord][x_coord][0] = (uint8_t)R;
    p_frame->yx[y_coord][x_coord][1] = (uint8_t)G;
    p_frame->yx[y_coord][x_coord][2] = (uint8_t)B;
}

inline void leddisplay_frame_fill_rgb(leddisplay_frame_t *p_frame, uint8_t red, uint8_t green, uint8_t blue)
{
    if ( (red == green) && (red == blue) )
    {
        memset(p_frame, red, sizeof(*p_frame));
    }
    else
    {
        for (uint16_t ix = 0; ix < NUMOF(p_frame->ix); ix++)
        {
            p_frame->ix[ix][0] = red;
            p_frame->ix[ix][1] = green;
            p_frame->ix[ix][2] = blue;
        }
    }
}

inline void leddisplay_frame_clear(leddisplay_frame_t *p_frame)
{
    memset(p_frame, 0, sizeof(*p_frame));
}

IRAM_ATTR void leddisplay_frame_update(leddisplay_frame_t *p_frame) {
    
    // if necessary, block until current framebuffer memory becomes available
    //only if in single panel mode. Multy-panel mode semaphores are handled by the calling function
    xSemaphoreTake(s_shift_complete_sem, portMAX_DELAY);

#if 0
    pan_sel_t psel;
	psel.by = 0;
	
    for (uint16_t x = 0; x < LEDDISPLAY_WIDTH; x++)
    {
        for (uint16_t y = 0; y < LEDDISPLAY_HEIGHT; y++)
        {
            const uint8_t *p_rgb = p_frame->yx[y][x];
            leddisplay_pixel_xy_rgb(x, y, p_rgb[0], p_rgb[1], p_rgb[2]);
        }
    }
#else
/*
#if CONFIG_LEDDISPLAY_CORR_BRIGHT_STRICT || CONFIG_LEDDISPLAY_CORR_BRIGHT_MODIFIED
    for (uint16_t ix = 0; ix < NUMOF(p_frame->raw); ix++)
    {
        p_frame->raw[ix] = val2pwm(p_frame->raw[ix]);
    }
#endif
*/
    for (unsigned int y_coord = 0; y_coord < ROWS_PER_FRAME; y_coord++) // half height - 16 iterations
    {
        row_data_t *row_data = &s_frames[s_current_frame].rowdata[y_coord];

        for (int bitplane_ix = 0; bitplane_ix < COLOR_DEPTH_BITS; bitplane_ix++)  // color depth - 8 iterations
        {
            uint16_t mask = (1 << bitplane_ix); // 24 bit color

            // the destination for the pixel bitstream
            row_bit_t *rowbits = &row_data->rowbits[bitplane_ix]; //matrixUpdateFrames location to write to uint16_t's

            for (int x_coord = 0; x_coord < LEDDISPLAY_WIDTH; x_coord++) // row pixel width 64 iterations
            {
                int v = 0; // the output bitstream

                // if there is no latch to hold address, output ADDX lines directly to GPIO and latch data at end of cycle
                // normally output current rows ADDX, special case for LSB, output previous row's ADDX (as previous row is being displayed for one latch cycle)
                int gpioRowAddress = (bitplane_ix == 0) ? y_coord - 1 : y_coord;

                if (gpioRowAddress & BIT(0)) { v |= BIT_A; } // 1
                if (gpioRowAddress & BIT(1)) { v |= BIT_B; } // 2
                if (gpioRowAddress & BIT(2)) { v |= BIT_C; } // 4
                if (gpioRowAddress & BIT(3)) { v |= BIT_D; } // 8
#if LEDDISPLAY_NEED_E_GPIO
                if (gpioRowAddress & BIT(4)) { v |= BIT_E; } // 16
#endif
                // top half
                uint8_t *p_rgb_top = p_frame->yx[y_coord][x_coord];
                if (val2pwm(p_rgb_top[0]) & mask) { v |= BIT_R1; }
                if (val2pwm(p_rgb_top[1]) & mask) { v |= BIT_G1; }
                if (val2pwm(p_rgb_top[2]) & mask) { v |= BIT_B1; }

                // bottom half
                uint8_t *p_rgb_bot = p_frame->yx[y_coord + ROWS_PER_FRAME][x_coord];
                if (val2pwm(p_rgb_bot[0]) & mask) { v |= BIT_R2; }
                if (val2pwm(p_rgb_bot[1]) & mask) { v |= BIT_G2; }
                if (val2pwm(p_rgb_bot[2]) & mask) { v |= BIT_B2; }
                
                // need to disable OE after latch to hide row transition
                if (x_coord == 0) { v |= BIT_OE; }

                // turn off OE after brightness value is reached when displaying MSBs
                // MSBs always output normal brightness
                // LSB (!bitplane_ix) outputs normal brightness as MSB from previous row is being displayed
                if ( ((bitplane_ix > s_lsb_msb_transition_bit) || !bitplane_ix) && (x_coord >= s_brightness_val) )
                {
                    v |= BIT_OE; // For Brightness
                }

                // special case for the bits *after* LSB through (s_lsb_msb_transition_bit) - OE is output after data is shifted, so need to set OE to fractional brightness
                if (bitplane_ix && (bitplane_ix <= s_lsb_msb_transition_bit))
                {
                    // divide brightness in half for each bit below s_lsb_msb_transition_bit
                    int lsbBrightness = s_brightness_val >> (s_lsb_msb_transition_bit - bitplane_ix + 1);
                    if (x_coord >= lsbBrightness) { v |= BIT_OE; } // For Brightness
                }

                // drive latch while shifting out last bit of RGB data
                // need to turn off OE one clock before latch, otherwise can get ghosting
                //if (x_coord == (PIXELS_PER_LATCH - 1)) { v |= (BIT_LAT | BIT_OE0); }
                //if (x_coord == (PIXELS_PER_LATCH - 2)) { v |= (BIT_OE); }
                if (x_coord == (PIXELS_PER_LATCH - 1)) { v |= (BIT_LAT | BIT_OE); }

                // 16 bit parallel mode
                // Save the calculated value to the bitplane memory in reverse order to account for I2S Tx FIFO mode1 ordering
                if ((x_coord % 2) != 0)
                {
                    rowbits->pixel[x_coord - 1] = v;
                }
                else
                {
                    rowbits->pixel[x_coord + 1] = v;
                } // end reordering

            } // end x_coord iteration
        } // colour depth loop (8)
    } // end row iteration
#endif
    leddisplay_pixel_update(CONFIG_BLOCK_EN);
    //leddisplay_pixel_update_2();
}

IRAM_ATTR void leddisplay_push_single_frame(leddisplay_frame_t *p_frame) {
    
    // if necessary, block until current framebuffer memory becomes available
    //only if in single panel mode. Multy-panel mode semaphores are handled by the calling function
    //xSemaphoreTake(s_shift_complete_sem, portMAX_DELAY);

#if 0
    pan_sel_t psel;
	psel.by = 0;
	
    for (uint16_t x = 0; x < LEDDISPLAY_WIDTH; x++)
    {
        for (uint16_t y = 0; y < LEDDISPLAY_HEIGHT; y++)
        {
            const uint8_t *p_rgb = p_frame->yx[y][x];
            leddisplay_pixel_xy_rgb(x, y, p_rgb[0], p_rgb[1], p_rgb[2]);
        }
    }
#else
/*
#if CONFIG_LEDDISPLAY_CORR_BRIGHT_STRICT || CONFIG_LEDDISPLAY_CORR_BRIGHT_MODIFIED
    for (uint16_t ix = 0; ix < NUMOF(p_frame->raw); ix++)
    {
        p_frame->raw[ix] = val2pwm(p_frame->raw[ix]);
    }
#endif
*/
    for (unsigned int y_coord = 0; y_coord < ROWS_PER_FRAME; y_coord++) // half height - 16 iterations
    {
        row_data_t *row_data = &s_frames[s_current_frame].rowdata[y_coord];

        for (int bitplane_ix = 0; bitplane_ix < COLOR_DEPTH_BITS; bitplane_ix++)  // color depth - 8 iterations
        {
            uint16_t mask = (1 << bitplane_ix); // 24 bit color

            // the destination for the pixel bitstream
            row_bit_t *rowbits = &row_data->rowbits[bitplane_ix]; //matrixUpdateFrames location to write to uint16_t's

            for (int x_coord = 0; x_coord < LEDDISPLAY_WIDTH; x_coord++) // row pixel width 64 iterations
            {
                int v = 0; // the output bitstream

                // if there is no latch to hold address, output ADDX lines directly to GPIO and latch data at end of cycle
                // normally output current rows ADDX, special case for LSB, output previous row's ADDX (as previous row is being displayed for one latch cycle)
                int gpioRowAddress = (bitplane_ix == 0) ? y_coord - 1 : y_coord;

                if (gpioRowAddress & BIT(0)) { v |= BIT_A; } // 1
                if (gpioRowAddress & BIT(1)) { v |= BIT_B; } // 2
                if (gpioRowAddress & BIT(2)) { v |= BIT_C; } // 4
                if (gpioRowAddress & BIT(3)) { v |= BIT_D; } // 8
#if LEDDISPLAY_NEED_E_GPIO
                if (gpioRowAddress & BIT(4)) { v |= BIT_E; } // 16
#endif
                // top half
                uint8_t *p_rgb_top = p_frame->yx[y_coord][x_coord];
                if (val2pwm(p_rgb_top[0]) & mask) { v |= BIT_R1; }
                if (val2pwm(p_rgb_top[1]) & mask) { v |= BIT_G1; }
                if (val2pwm(p_rgb_top[2]) & mask) { v |= BIT_B1; }

                // bottom half
                uint8_t *p_rgb_bot = p_frame->yx[y_coord + ROWS_PER_FRAME][x_coord];
                if (val2pwm(p_rgb_bot[0]) & mask) { v |= BIT_R2; }
                if (val2pwm(p_rgb_bot[1]) & mask) { v |= BIT_G2; }
                if (val2pwm(p_rgb_bot[2]) & mask) { v |= BIT_B2; }
                
                // need to disable OE after latch to hide row transition
                if (x_coord == 0) { v |= BIT_OE; }

                // turn off OE after brightness value is reached when displaying MSBs
                // MSBs always output normal brightness
                // LSB (!bitplane_ix) outputs normal brightness as MSB from previous row is being displayed
                if ( ((bitplane_ix > s_lsb_msb_transition_bit) || !bitplane_ix) && (x_coord >= s_brightness_val) )
                {
                    v |= BIT_OE; // For Brightness
                }

                // special case for the bits *after* LSB through (s_lsb_msb_transition_bit) - OE is output after data is shifted, so need to set OE to fractional brightness
                if (bitplane_ix && (bitplane_ix <= s_lsb_msb_transition_bit))
                {
                    // divide brightness in half for each bit below s_lsb_msb_transition_bit
                    int lsbBrightness = s_brightness_val >> (s_lsb_msb_transition_bit - bitplane_ix + 1);
                    if (x_coord >= lsbBrightness) { v |= BIT_OE; } // For Brightness
                }

                // drive latch while shifting out last bit of RGB data
                // need to turn off OE one clock before latch, otherwise can get ghosting
                //if (x_coord == (PIXELS_PER_LATCH - 1)) { v |= (BIT_LAT | BIT_OE0); }
                //if (x_coord == (PIXELS_PER_LATCH - 2)) { v |= (BIT_OE); }
                if (x_coord == (PIXELS_PER_LATCH - 1)) { v |= (BIT_LAT | BIT_OE); }

                // 16 bit parallel mode
                // Save the calculated value to the bitplane memory in reverse order to account for I2S Tx FIFO mode1 ordering
                if ((x_coord % 2) != 0)
                {
                    rowbits->pixel[x_coord - 1] = v;
                }
                else
                {
                    rowbits->pixel[x_coord + 1] = v;
                } // end reordering

            } // end x_coord iteration
        } // colour depth loop (8)
    } // end row iteration
#endif
    //leddisplay_pixel_update(CONFIG_BLOCK_EN);
    leddisplay_pixel_update_2();
}
/* *********************************************************************************************** */
