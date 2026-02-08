cmake_minimum_required(VERSION 4.0)
project(parkme-unilidar)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -DNDEBUG") 

include_directories(include)

link_directories(lib/${CMAKE_SYSTEM_PROCESSOR})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR})

file(GLOB PARKME_SRC "src/*.cpp")

add_executable(parkme_unilidar
    ${PARKME_SRC}
)

find_package(yaml-cpp REQUIRED)

target_link_libraries(parkme_unilidar
    libunilidar_sdk2.a
    yaml-cpp
)

target_include_directories(parkme_unilidar PRIVATE include)

install(TARGETS parkme_unilidar RUNTIME DESTINATION bin)
