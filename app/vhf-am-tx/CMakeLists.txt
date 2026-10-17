# CMakefile
# 144MHz A2A/A3E Transmitter

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(vhf-am-tx)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Git Version
execute_process(COMMAND git describe --dirty --always --tags
                OUTPUT_VARIABLE GIT_VERSION
                OUTPUT_STRIP_TRAILING_WHITESPACE)
configure_file ("main/version.h.in" "../main/version.h")
