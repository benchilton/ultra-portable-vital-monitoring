/*
 *  @name          bsp_display.c
 *  @date          26/11/22
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

/****************************** - Library Includes - *******************************/

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/pwm.h>

/******************************** - User Includes - ********************************/

#include "zephyr_typedefs.h"
#include "bsp_std_includes.h"
#include "bsp_display.h"

#include "fonts/times_16pt_rgb565.h"

/*********************************** - Defines - ***********************************/

#define AFL240320A0_DISPLAY_DEVICE       DEVICE_DT_GET(DT_NODELABEL(afl240320a0))
#define AFL240320A0_BACKLIGHT_GPIO       GPIO_DT_SPEC_GET( DT_CHILD( DT_NODELABEL( afl240320a0 ) , properties ) , blk_gpios )
#define AFL240320A0_BACKLIGHT_PWM        PWM_DT_SPEC_GET( DT_CHILD( DT_NODELABEL( afl240320a0 ) , properties ))

#define AFL240320A0_WAKE_UP_GPIO         GPIO_DT_SPEC_GET( DT_CHILD( DT_NODELABEL( afl240320a0 ) , properties ) , wake_gpios )
#define AFL240320A0_ENABLE_GPIO          GPIO_DT_SPEC_GET( DT_CHILD( DT_NODELABEL( afl240320a0 ) , properties ) , enable_gpios )

#define AFL240320A0_SCREEN_WIDTH         DT_PROP( DT_NODELABEL( afl240320a0 ) , width)
#define AFL240320A0_SCREEN_HEIGHT        DT_PROP( DT_NODELABEL( afl240320a0 ) , height)

#define BSP_DISPLAY_PIXEL_SCALE          (sizeof(uint16_t))
#define BSP_DISPLAY_CHAR_TAB             "    "
#define BSP_DISPLAY_MAX_BUFFER_SIZE      (1280)
#define BSP_DISPLAY_BACKLIGHT_MAX_PERIOD (100000U)
#define AFL240320A0_DEFAULT_BRIGHTNESS   250
#define BSP_DISPLAY_MAX_BRIGHTNESS       1000U

#define BSP_DISPLAY_FADE_STEP            K_MSEC(1)
#define BSP_DISPLAY_FADE_TIME            K_SECONDS(2)

/************************************ - Enums - ************************************/

typedef enum {
    BSP_DISPLAY_LIGHT_SLEEP_STATE_FULL = 0,
    BSP_DISPLAY_LIGHT_SLEEP_STATE_DIM,
    BSP_DISPLAY_LIGHT_SLEEP_STATE_OFF,
    BSP_DISPLAY_LIGHT_SLEEP_STATE_MAX,
} bsp_display_timer_sleep_state_e;

/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/

typedef struct {
    uint16_t y_pos;
    uint16_t x_pos;
    uint16_t page_width;
    uint16_t page_height;
    uint8_t  num_lines;
    bsp_display_rgb565_t colour;
} bsp_display_text_page_t;

typedef struct
{
    /**SPI related properties**/
    const zephyr_device_t*                   display_device;
    zephyr_display_capabilities_t            display_capabilities;
    bsp_gpio_prop_t                          back_light_gpio;
    zephyr_pwm_dt_spec_t                     back_light_pwm;
    uint16_t                                 screen_brightness;

    bsp_gpio_prop_t                          enable_gpio;

    bsp_display_text_page_t                  text_page;

    bsp_gpio_prop_t                          wake_up_gpio;
    zephyr_k_timer_t                         sleep_timer;
    volatile bsp_display_timer_sleep_state_e sleep_state;
    k_timeout_t                              dim_time;
    k_timeout_t                              off_time;

    zephyr_k_work_t                          display_work;

} bsp_afl240320a0_t;

/**************************** - Function Prototypes - ******************************/

static font_tImage_t* bsp_display_find_font( char c , const font_tFont_t* font );
static zephyr_err_t   bsp_display_print_char( char* chars , uint16_t num_chars );
static bool           bsp_display_handle_cmd_chars( char c );
static void           bsp_display_buffer_colour( uint8_t* image_buffer , struct display_buffer_descriptor* desc ,
                                                 bsp_display_rgb565_t colour );
static inline void    bsp_display_update_pos( zephyr_display_buffer_descriptor_t image_desc );
static zephyr_err_t   bsp_display_init_blk_gpio(void);
static void           bsp_display_blk_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins);
static void           bsp_display_wake_up_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins);
static void           bsp_display_sleep_timer_exp(zephyr_k_timer_t *timer);
static zephyr_err_t   bsp_display_display_wakeup_gpio_init(void);
static zephyr_err_t   bsp_display_tft_enable( uint8_t on_off );

static void           bsp_display_tft_enable_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins);
static zephyr_err_t   bsp_display_init_tft_enable_gpio(void);
static void           bsp_display_work_handler(zephyr_k_work_t* calling_work);

/********************************* - Constants - ***********************************/
/********************************* - Variables - ***********************************/

bsp_afl240320a0_t afl240320a0 = {

        .display_device = AFL240320A0_DISPLAY_DEVICE,

        .back_light_gpio = {
            .gpio_spec = AFL240320A0_BACKLIGHT_GPIO,
        },
        .back_light_pwm = AFL240320A0_BACKLIGHT_PWM,

        .screen_brightness = AFL240320A0_DEFAULT_BRIGHTNESS,


        .text_page = {
            .page_height = 200,
            .page_width  = 310,
            .x_pos = 0,
            .y_pos = 0,
            .num_lines = 4,
            .colour = BSP_DISPLAY_COLOUR_WHITE,/*Default to white text*/
        },

        .wake_up_gpio = {
            .gpio_spec =  AFL240320A0_WAKE_UP_GPIO,
        },

        .enable_gpio = {
            .gpio_spec = AFL240320A0_ENABLE_GPIO,
        },

        .sleep_state = BSP_DISPLAY_LIGHT_SLEEP_STATE_FULL,

};

/***************************** - Public Functions - ********************************/

zephyr_err_t bsp_display_init()
{

    afl240320a0.off_time = K_SECONDS(20);
    afl240320a0.dim_time = K_SECONDS(20);

    zephyr_err_t err = bsp_display_init_tft_enable_gpio();
    if(ZEPHYR_ERR_SUCCESS == err)
    {
        err = bsp_display_init_blk_gpio();
        if(ZEPHYR_ERR_SUCCESS == err)
        {
            err = bsp_display_display_wakeup_gpio_init();
            if(ZEPHYR_ERR_SUCCESS == err)
            {
                err = display_set_pixel_format(afl240320a0.display_device, PIXEL_FORMAT_RGB_565);
                if (ZEPHYR_ERR_SUCCESS == err)
                {
                    err = bsp_display_clear();
                    if (ZEPHYR_ERR_SUCCESS == err)
                    {
                        err = display_blanking_off(afl240320a0.display_device);
                        if (ZEPHYR_ERR_SUCCESS == err)
                        {
                            display_get_capabilities(afl240320a0.display_device, &afl240320a0.display_capabilities);
                            err = bsp_display_backlight_set_brightness( AFL240320A0_DEFAULT_BRIGHTNESS );
                            k_timer_init(&afl240320a0.sleep_timer , bsp_display_sleep_timer_exp , NULL );
                            k_work_init(&afl240320a0.display_work,bsp_display_work_handler);
                        }
                    }
                }
            }
        }
    }

    return err;
}

static void bsp_display_work_handler(zephyr_k_work_t* calling_work)
{
    if( afl240320a0.sleep_state == BSP_DISPLAY_LIGHT_SLEEP_STATE_FULL )
    {
        bsp_display_turn_on();
    }
    if( afl240320a0.sleep_state == BSP_DISPLAY_LIGHT_SLEEP_STATE_OFF )
    {
        bsp_display_turn_off();
    }
}

zephyr_err_t bsp_display_clear()
{
    return bsp_display_draw_rectangle( 0 , 0, AFL240320A0_SCREEN_WIDTH , AFL240320A0_SCREEN_HEIGHT , BSP_DISPLAY_COLOUR_BLACK);
}

zephyr_err_t bsp_display_draw( uint16_t x_pos , uint16_t y_pos , zephyr_display_buffer_descriptor_t* image_desc , uint8_t* image_buf)
{
    return display_write( afl240320a0.display_device , x_pos , y_pos , image_desc , image_buf );
}

#if 0
void bsp_display_test()
{
    bsp_display_init();

    bsp_display_clear();

    bsp_display_set_text_colour(  BSP_DISPLAY_COLOUR_RED );
    bsp_display_print_string("Welcome to the display driver\n");
    bsp_display_set_text_colour(  BSP_DISPLAY_COLOUR_GREEN );
    bsp_display_print_string("The print character function can be used just like\nprintk.\n");
    bsp_display_set_text_colour(  BSP_DISPLAY_COLOUR_MAGENTA);
    bsp_display_print_string("Even with format specifiers like %%f: %f\n" , 4.20f);
    bsp_display_set_text_colour(  BSP_DISPLAY_COLOUR_CYAN);
    bsp_display_print_string("%%s example: %s\n" , "Hello World" );
    bsp_display_set_text_colour(  BSP_DISPLAY_COLOUR_WHITE);
    bsp_display_print_string("%%u example: %u\n" ,  1000);

    bsp_display_backlight_set_brightness(0);

/* capture initial time stamp */

#if 1
    while (1)
    {
        bsp_display_backlight_fade( 0 , 250 , K_SECONDS(2) );
        bsp_display_backlight_fade( 250 , 0 , K_SECONDS(2));
    }
#endif
}
#endif

zephyr_err_t bsp_display_backlight_fade( int16_t start_brightness , int16_t end_brightness , k_timeout_t time )
{
    zephyr_err_t err = -EINVAL;
    if( (time.ticks != K_FOREVER.ticks) &&
        ( start_brightness <= BSP_DISPLAY_MAX_BRIGHTNESS ) &&
        ( end_brightness <= BSP_DISPLAY_MAX_BRIGHTNESS ) )
    {
        uint64_t time_step = time.ticks / ( BSP_DISPLAY_FADE_STEP.ticks );
        float    brightness_step = (((float) end_brightness) - ((float) start_brightness)) / ( (float) time_step );
        float    brightness = (float) start_brightness;

        for (uint64_t steps = 0; steps < time_step; steps++ )
        {
            err = bsp_display_backlight_set_brightness( (uint16_t) brightness );
            brightness = brightness + brightness_step;
            k_sleep( BSP_DISPLAY_FADE_STEP );
        }
    }

    return err;
}

/* Range 0 - 10000 ie scaled 0 to 100.00 with minimum step being 0.01 */
/*Document equation
 *
 * D = T1/(T1+T2)
 *
 * */
zephyr_err_t bsp_display_backlight_set_brightness( uint16_t duty_cycle )
{
    zephyr_err_t err = -EINVAL;
    if( duty_cycle < BSP_DISPLAY_MAX_BRIGHTNESS )
    {
        float duty_cycle_f = (float) duty_cycle * 10.0f;
        uint32_t pulse = (uint32_t)
                ((((10000.0f - 2*duty_cycle_f)) / (10000.0f - duty_cycle_f)) * ( (float) BSP_DISPLAY_BACKLIGHT_MAX_PERIOD));
        /**
         * T((1-d) - d)/(1-d)
         * **/
        err = pwm_set_dt(&afl240320a0.back_light_pwm, BSP_DISPLAY_BACKLIGHT_MAX_PERIOD ,  pulse );

        afl240320a0.screen_brightness = ( ZEPHYR_ERR_SUCCESS == err ) ? duty_cycle : afl240320a0.screen_brightness ;

    }

    return err;
}

zephyr_err_t bsp_display_draw_rectangle( uint16_t x_pos , uint16_t y_pos , uint16_t width , uint16_t height ,
                                         bsp_display_rgb565_t colour )
{
    zephyr_err_t err = -EINVAL;
    zephyr_display_buffer_descriptor_t rect_desc = {
            .height   = 2,
            .width    = width,
            .pitch    = width,
            .buf_size = BSP_DISPLAY_MAX_BUFFER_SIZE,
    };
    uint8_t* rect_buf = k_malloc( sizeof(uint8_t) * BSP_DISPLAY_MAX_BUFFER_SIZE );
    /*Fill the buffer with our colour*/
    uint8_t lb = 0x00FF & colour.colour;
    uint8_t ub = (0xFF00 & colour.colour) >> 8;

    for (uint32_t idx = 0; idx < BSP_DISPLAY_MAX_BUFFER_SIZE; idx += sizeof(uint16_t) )
    {
        rect_buf[idx]   = lb;
        rect_buf[idx+1] = ub;
    }
    for (uint16_t idx = 0; idx < height / 2; ++idx)
    {
        err = display_write( afl240320a0.display_device , x_pos , y_pos ,&rect_desc , rect_buf );
        y_pos+=2;
    }
    k_free( rect_buf );
    return err;
}

zephyr_err_t bsp_display_print_string( const char *fmt , ... )
{
    va_list arg;
    const uint16_t str_size = sizeof(char) * 2 * strlen(fmt);
    char* string_buffer = k_malloc( str_size );
    zephyr_err_t err;

    va_start(arg, fmt);
    vsnprintk( string_buffer , str_size , fmt , arg );
    va_end(arg);

    err = bsp_display_print_char( string_buffer , strlen(string_buffer) );
    k_free(string_buffer);

    return err;
}

zephyr_err_t bsp_display_set_text_pos( uint16_t x_pos , uint16_t y_pos )
{
    zephyr_err_t err = -EINVAL;
    if( (x_pos < afl240320a0.text_page.page_width) && (y_pos < afl240320a0.text_page.page_height) )
    {
        afl240320a0.text_page.x_pos = x_pos;
        afl240320a0.text_page.x_pos = y_pos;
        err = ZEPHYR_ERR_SUCCESS;
    }
    return err;
}

void bsp_display_set_text_colour( bsp_display_rgb565_t colour )
{
    afl240320a0.text_page.colour = colour;
}

zephyr_err_t bsp_display_turn_on()
{
    zephyr_err_t err = bsp_display_tft_enable( true );
    k_sleep(K_MSEC(20));
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        err = bsp_display_backlight_fade( 0 , AFL240320A0_DEFAULT_BRIGHTNESS ,
                                          BSP_DISPLAY_FADE_TIME );
    }
    return err;
}

zephyr_err_t bsp_display_turn_off()
{
    zephyr_err_t err = bsp_display_backlight_fade( (int16_t) afl240320a0.screen_brightness , 0 ,
                                       BSP_DISPLAY_FADE_TIME );
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        err = bsp_display_tft_enable( false );
    }
    return err;
}

/***************************** - Private Functions - *******************************/

static zephyr_err_t bsp_display_print_char( char* chars , uint16_t num_chars )
{
    zephyr_err_t                       err = ZEPHYR_ERR_SUCCESS;
    font_tImage_t*                     char_image = NULL;
    uint8_t*                           image_buffer = k_malloc(0);
    uint8_t*                           buffer_ptr = NULL;
    zephyr_display_buffer_descriptor_t image_desc;
    char                               prev_char = '\0';

    for (uint16_t idx = 0; (idx < num_chars) && (ZEPHYR_ERR_SUCCESS == err); idx++)
    {
        /*If there are any command chars then handle them here*/
        if( true == bsp_display_handle_cmd_chars(chars[idx]))
        {
            continue;
        }
        if (prev_char != chars[idx])
        {
            k_free(image_buffer);
            char_image          = bsp_display_find_font(chars[idx], &times16pt);
            image_desc.pitch    = char_image->width;
            image_desc.width    = char_image->width;
            image_desc.height   = char_image->height;
            image_desc.buf_size = BSP_DISPLAY_PIXEL_SCALE * image_desc.width * image_desc.height;
            image_buffer = k_malloc(sizeof(uint8_t) * image_desc.buf_size );
            /*If we want a colour other than white we need to apply the mask to change it colour*/
            if( afl240320a0.text_page.colour.colour != 0xFFFF )
            {
                memcpy( &image_buffer[0] , &char_image->data[0] , image_desc.buf_size );
                bsp_display_buffer_colour( image_buffer , &image_desc , afl240320a0.text_page.colour );
                buffer_ptr = image_buffer;
            }
            else
            {
                buffer_ptr = (uint8_t*) char_image->data;
            }
        }
        err = display_write(afl240320a0.display_device,
                            afl240320a0.text_page.x_pos, afl240320a0.text_page.y_pos,
                        &image_desc,buffer_ptr);
        bsp_display_update_pos( image_desc );

        prev_char = chars[idx];
    }
    return err;
}

static font_tImage_t* bsp_display_find_font( char c , const font_tFont_t* font )
{
    font_tChar_t* matching_idx = (font_tChar_t*) font->chars;
    font_tImage_t* ret_val = NULL;

    for (uint16_t idx = 0; idx < (uint16_t) font->length; ++idx)
    {
        if( (char) matching_idx->code == c )
        {
            ret_val = (font_tImage_t*) matching_idx->image;
        }
        matching_idx++;
    }
    return ret_val;
}

static bool bsp_display_handle_cmd_chars( char c )
{
    bool ret_val = true;
    switch ( c )
    {
        case '\n':
            afl240320a0.text_page.y_pos += 19;
            afl240320a0.text_page.x_pos = 0;
        break;
        case '\r':
            afl240320a0.text_page.x_pos = 0;
        break;
        case '\t':
            bsp_display_print_char(BSP_DISPLAY_CHAR_TAB, strlen(BSP_DISPLAY_CHAR_TAB));
        break;
        default:
            ret_val = false;
        break;
    }
    return ret_val;
}

static void bsp_display_buffer_colour( uint8_t* image_buffer , struct display_buffer_descriptor* desc ,
                                       bsp_display_rgb565_t colour )
{

    uint8_t lb = 0x00FF & colour.colour;
    uint8_t ub = (0xFF00 & colour.colour) >> 8;

    for (uint32_t idx = 0; idx < desc->buf_size; idx += sizeof(uint16_t) )
    {
        image_buffer[idx] &= lb;
        image_buffer[idx+1] &= ub;
    }

}

static inline void bsp_display_update_pos( zephyr_display_buffer_descriptor_t image_desc )
{
    if ((afl240320a0.text_page.x_pos + image_desc.width) >= afl240320a0.text_page.page_width)
    {
        afl240320a0.text_page.x_pos = 0;
        if ((afl240320a0.text_page.y_pos + image_desc.height) >= afl240320a0.text_page.page_height)
        {
            afl240320a0.text_page.y_pos = 0;
        }
        else
        {
            afl240320a0.text_page.y_pos += image_desc.height;
        }
    }
    else
    {
        afl240320a0.text_page.x_pos += image_desc.pitch;
    }
}

static zephyr_err_t bsp_display_init_blk_gpio(void)
{
    zephyr_err_t err = gpio_pin_configure_dt(&afl240320a0.back_light_gpio.gpio_spec, GPIO_OUTPUT);
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        /*Init start convert GPIO and callback*/
        gpio_init_callback(&afl240320a0.back_light_gpio.gpio_callback, bsp_display_blk_cb,
                           BIT(afl240320a0.back_light_gpio.gpio_spec.pin));
        err = gpio_add_callback(afl240320a0.back_light_gpio.gpio_spec.port, &afl240320a0.back_light_gpio.gpio_callback);
    }

    return err;
}

static void bsp_display_blk_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins)
{
    bsp_display_backlight_set_brightness(AFL240320A0_DEFAULT_BRIGHTNESS);
}


static zephyr_err_t bsp_display_display_wakeup_gpio_init(void)
{
    zephyr_err_t err = gpio_pin_configure_dt(&afl240320a0.wake_up_gpio.gpio_spec, GPIO_INPUT );
    if (ZEPHYR_ERR_SUCCESS == err)
    {
        err = gpio_pin_interrupt_configure_dt(&afl240320a0.wake_up_gpio.gpio_spec, GPIO_INT_EDGE_FALLING );
        if (ZEPHYR_ERR_SUCCESS == err)
        {
            /*Init busy GPIO and callback*/
            gpio_init_callback(&afl240320a0.wake_up_gpio.gpio_callback, bsp_display_wake_up_cb,
                               BIT(afl240320a0.wake_up_gpio.gpio_spec.pin));
            err = gpio_add_callback(afl240320a0.wake_up_gpio.gpio_spec.port, &afl240320a0.wake_up_gpio.gpio_callback);
        }
    }
    return err;
}


static void bsp_display_wake_up_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins)
{
    afl240320a0.sleep_state = BSP_DISPLAY_LIGHT_SLEEP_STATE_FULL;
    /*Turn on display and enable TFT_EN GPIO*/
    k_work_submit(&afl240320a0.display_work);
    k_timer_start(&afl240320a0.sleep_timer , afl240320a0.dim_time , afl240320a0.dim_time );
}

static void bsp_display_sleep_timer_exp(zephyr_k_timer_t *timer)
{
    switch (afl240320a0.sleep_state)
    {
        case BSP_DISPLAY_LIGHT_SLEEP_STATE_FULL:
            bsp_display_backlight_fade( (int16_t) afl240320a0.screen_brightness , 100 , K_SECONDS(2) );
            afl240320a0.sleep_state = BSP_DISPLAY_LIGHT_SLEEP_STATE_DIM;
            k_timer_start(&afl240320a0.sleep_timer , afl240320a0.off_time , afl240320a0.off_time );
        break;
        case BSP_DISPLAY_LIGHT_SLEEP_STATE_DIM:
            afl240320a0.sleep_state = BSP_DISPLAY_LIGHT_SLEEP_STATE_OFF;
            k_timer_stop(&afl240320a0.sleep_timer);
            /*Turn of display and disable TFT_EN GPIO*/
            k_work_submit(&afl240320a0.display_work);
        break;
        case BSP_DISPLAY_LIGHT_SLEEP_STATE_OFF:
            k_timer_stop(&afl240320a0.sleep_timer);
        break;
        case BSP_DISPLAY_LIGHT_SLEEP_STATE_MAX:
        default:
            break;
    }
}

static zephyr_err_t bsp_display_tft_enable( uint8_t on_off )
{
    return gpio_pin_set_dt( &afl240320a0.enable_gpio.gpio_spec , on_off );
}

static zephyr_err_t bsp_display_init_tft_enable_gpio(void)
{
    zephyr_err_t err = gpio_pin_configure_dt(&afl240320a0.enable_gpio.gpio_spec, GPIO_OUTPUT);
    if( ZEPHYR_ERR_SUCCESS == err )
    {
        /*Init start convert GPIO and callback*/
        gpio_init_callback(&afl240320a0.enable_gpio.gpio_callback, bsp_display_tft_enable_cb,
                           BIT(afl240320a0.enable_gpio.gpio_spec.pin));
        err = gpio_add_callback(afl240320a0.enable_gpio.gpio_spec.port, &afl240320a0.enable_gpio.gpio_callback);
        gpio_pin_set_dt(&afl240320a0.enable_gpio.gpio_spec,1);
    }

    return err;
}

static void bsp_display_tft_enable_cb(const zephyr_device_t* port, zephyr_gpio_callback_t* cb, gpio_port_pins_t pins)
{

}