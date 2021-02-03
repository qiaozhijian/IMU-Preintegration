find_package(OpenCV REQUIRED)

if (NOT OpenCV_FOUND)
    find_package(OpenCV 2.4.3 QUIET)
    if (NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif ()
endif ()

include_directories(${OpenCV_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES ${OpenCV_LIBS})