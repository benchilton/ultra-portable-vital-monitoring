
#Define the headers
set(HEADER_FILES
        #STD_INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/zephyr_typedefs.h
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/bsp_std_includes.h
        #BSP
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/bsp.h
        #ADC Files
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/bsp_ad469x.h
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/bsp_ad469x_calibrations.h
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/bsp_adc.h
        #Data capture
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/bsp_ppg.h
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/bsp_respiration.h
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/bsp_accelerometer.h
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/bsp_thermometer.h
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/bsp_sensor_interface.h
        #Battery Gauge
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/bsp_battery_gauge.h
        #FFT
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/bsp_fft.h
        #Display
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/bsp_display.h
    )

set(COMP_SOURCES
        #BSP
        ${CMAKE_CURRENT_SOURCE_DIR}/bsp.c
        #ADC Files
        ${CMAKE_CURRENT_SOURCE_DIR}/bsp_ad469x.c
        ${CMAKE_CURRENT_SOURCE_DIR}/bsp_adc.c
        #Data capture
        ${CMAKE_CURRENT_SOURCE_DIR}/bsp_ppg.c
        ${CMAKE_CURRENT_SOURCE_DIR}/bsp_respiration.c
        ${CMAKE_CURRENT_SOURCE_DIR}/bsp_accelerometer.c
        ${CMAKE_CURRENT_SOURCE_DIR}/bsp_thermometer.c
        ${CMAKE_CURRENT_SOURCE_DIR}/bsp_sensor_interface.c
        #Battery Gauge
        ${CMAKE_CURRENT_SOURCE_DIR}/bsp_battery_gauge.c
        #FFT
        ${CMAKE_CURRENT_SOURCE_DIR}/bsp_fft.c
        #Display
        ${CMAKE_CURRENT_SOURCE_DIR}/bsp_display.c
    )

set(COMPONENT_HEADERS ${HEADER_FILES} PARENT_SCOPE)
set(COMPONENT_SOURCES ${COMP_SOURCES} PARENT_SCOPE)
