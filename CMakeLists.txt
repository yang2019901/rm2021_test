cmake_minimum_required(VERSION 3.5)
project(RM2021_Adv)

set(OpenCV_DIR "/usr/local/share/OpenCV")   # <- it's to set the version of opencv which is to be used. 
                                            #    delete it if you only have single version. 
find_package( OpenCV REQUIRED )
find_package( Threads)
add_definitions(-w)
set(CMAKE_CXX_STANDARD 11)
include_directories(./misc)
include_directories(./armor) 
include_directories(./rune) 
include_directories(./gimbal)
add_executable(RM2021_Adv main.cpp )

target_link_libraries( RM2021_Adv ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT} )
