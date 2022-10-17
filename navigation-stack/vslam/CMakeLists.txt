cmake_minimum_required(VERSION 2.8)

if(LU20_APPS)
    add_subdirectory(apps_lu20)
else()
    add_subdirectory(apps)
endif()

