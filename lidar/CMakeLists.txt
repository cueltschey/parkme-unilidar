cmake_minimum_required(VERSION 4.0)
project(parkme-unilidar)

set(CMAKE_BUILD_TYPE "Debug")
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

include(FetchContent)

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)

FetchContent_MakeAvailable(nlohmann_json)


# === boost ===
set(Boost_NO_BOOST_CMAKE ON)
find_package(Boost REQUIRED COMPONENTS filesystem thread regex)

# === PCL ===
find_package(PCL REQUIRED COMPONENTS
    common
    io
    search
    segmentation
)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})


target_link_libraries(parkme_unilidar
    PRIVATE
    libunilidar_sdk2.a
    yaml-cpp
    nlohmann_json::nlohmann_json
    Boost::thread
    Boost::regex
    ${PCL_LIBRARIES}
)


target_include_directories(parkme_unilidar PRIVATE include)

install(TARGETS parkme_unilidar RUNTIME DESTINATION bin)
