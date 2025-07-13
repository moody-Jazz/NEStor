cmake_minimum_required(VERSION 3.10)
project(NEStor)

set(CMAKE_CXX_STANDARD 17)

# Option to toggle test mode
option(BUILD_TESTS "Enable test build" OFF)
set(TEST_FILE "test/testfile.cpp" CACHE STRING "Path to test file containing main()")

# Include directories (for raylib headers)
include_directories(include)

# Library directory (where libraylib.a is located)
link_directories(lib)

# Collect all .cpp files in src/
file(GLOB ALL_SRC_FILES src/*.cpp)

# Filter out src/main.cpp
set(SRC_FILES "")
foreach(FILE ${ALL_SRC_FILES})
    if(NOT FILE MATCHES "src/main.cpp$")
        list(APPEND SRC_FILES ${FILE})
    endif()
endforeach()

# Select main file
if(BUILD_TESTS)
    message(STATUS "Test mode enabled")
    message(STATUS "Using ${TEST_FILE} as main")
    set(MAIN_FILE ${TEST_FILE})
else()
    message(STATUS "Using src/main.cpp as main")
    set(MAIN_FILE src/main.cpp)
endif()

# Add the main file to the build
list(APPEND SRC_FILES ${MAIN_FILE})

# Define the executable
add_executable(NEStor ${SRC_FILES})

# Link with raylib and system libraries (Windows specific)
target_link_libraries(NEStor
    raylib
    opengl32
    gdi32
    winmm
)
