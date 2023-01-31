cmake_minimum_required(VERSION 3.24)
project(decoder)

set(CMAKE_CXX_STANDARD 14)

include_directories(a1bin)
include_directories(artcm)
include_directories(oxts)

add_executable(decoder
        a1bin/Message_def.h
        a1bin/Msg_B2A.c
        a1bin/Msg_B2A.h
        artcm/artcm.c
        artcm/artcm.h
        oxts/NComRxC.c
        oxts/NComRxC.h
        decoder.cpp)
