
cmake_minimum_required (VERSION 3.10.0)
project(mcbflash)

include_directories(./inc)

set(source_code
src/mcb_flash.c
src/xmodem.c
src/main.c
)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-pthread")
add_compile_options( -DQRC_USER_DRIVER )
add_executable(mcbflash ${source_code})
target_link_libraries(mcbflash pthread)
target_link_libraries(mcbflash qrc_udriver)
