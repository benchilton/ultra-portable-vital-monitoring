
add_subdirectory(components)

set(HEADER_FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/main.h
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/vital_app.h
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/vital_defines.h
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/vital_bluetooth.h
        ${CMAKE_CURRENT_SOURCE_DIR}/includes/vital_gatt.h
        )

set(HEADER_LOCATIONS
        ${CMAKE_CURRENT_SOURCE_DIR}/includes
        )

#Add .c files to the project
target_sources(app PRIVATE
        ${HEADER_FILES}
        ${COMPONENT_HEADERS}
        ${COMPONENT_SOURCES}
        main.c
        vital_app.c
        vital_bluetooth.c
        vital_gatt.c
        ${CMAKE_CURRENT_SOURCE_DIR}/fonts/times_16pt_rgb565.h
    )
target_include_directories( app PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/includes
        ${CMAKE_CURRENT_SOURCE_DIR}/components/includes
    )
