/*
 *  @name          bsp_display.h
 *  @date          26/11/22
 *  @author        Benjamin Chilton
 *  @university_id bdc1g19
 *
 *
 */

#ifndef ULTRA_PORTABLE_VITAL_MONITORING_BSP_DISPLAY_H
#define ULTRA_PORTABLE_VITAL_MONITORING_BSP_DISPLAY_H

/****************************** - Library Includes - *******************************/
/******************************** - User Includes - ********************************/
/*********************************** - Defines - ***********************************/
/************************************ - Enums - ************************************/
/********************************** - Typedefs - ***********************************/
/********************************* - Structures - **********************************/
typedef union {
    uint16_t colour: 16;
#pragma pack(1)
    struct {
        uint8_t blue: 5;
        uint8_t green: 6;
        uint8_t red: 5;
    };
#pragma pack()
} bsp_display_rgb565_t;

#define BSP_DISPLAY_COLOUR_RED     ( (bsp_display_rgb565_t) { .colour = 0xF800 } )
#define BSP_DISPLAY_COLOUR_GREEN   ( (bsp_display_rgb565_t) { .colour = 0x07E0 } )
#define BSP_DISPLAY_COLOUR_BLUE    ( (bsp_display_rgb565_t) { .colour = 0x001F } )
#define BSP_DISPLAY_COLOUR_YELLOW  ( (bsp_display_rgb565_t) { .colour = 0xFFE0 } )
#define BSP_DISPLAY_COLOUR_CYAN    ( (bsp_display_rgb565_t) { .colour = 0x07FF } )
#define BSP_DISPLAY_COLOUR_MAGENTA ( (bsp_display_rgb565_t) { .colour = 0xF81F } )
#define BSP_DISPLAY_COLOUR_WHITE   ( (bsp_display_rgb565_t) { .colour = 0xFFFF } )
#define BSP_DISPLAY_COLOUR_BLACK   ( (bsp_display_rgb565_t) { .colour = 0x0000 } )

/**************************** - Function Prototypes - ******************************/
zephyr_err_t bsp_display_init();
zephyr_err_t bsp_display_clear();

void         bsp_display_test();

zephyr_err_t bsp_display_print_string( const char *fmt , ... );
void         bsp_display_set_text_colour( bsp_display_rgb565_t colour );
zephyr_err_t bsp_display_backlight_set_brightness( uint16_t duty_cycle );
zephyr_err_t bsp_display_set_text_pos( uint16_t x_pos , uint16_t y_pos );

zephyr_err_t bsp_display_draw_rectangle( uint16_t x_pos , uint16_t y_pos , uint16_t width , uint16_t height ,
                                         bsp_display_rgb565_t colour );
zephyr_err_t bsp_display_draw( uint16_t x_pos , uint16_t y_pos ,
                               zephyr_display_buffer_descriptor_t* image_desc , uint8_t* image_buf);
zephyr_err_t bsp_display_backlight_fade( int16_t start_brightness , int16_t end_brightness , k_timeout_t time );

zephyr_err_t bsp_display_turn_on();
zephyr_err_t bsp_display_turn_off();

/****************************** - Global Variables - *******************************/

#endif //ULTRA_PORTABLE_VITAL_MONITORING_BSP_DISPLAY_H
