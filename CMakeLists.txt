cmake_minimum_required( VERSION 3.0.0)
set(PROJECT_NAME K253154)

project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message("Setting build type to 'Release' as none as specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# set(CMAKE_C_COMPILER    "/home/tcspc/arm/aarch64-buildroot-linux-gnu_sdk-buildroot/bin/aarch64-buildroot-linux-gnu-gcc")
# set(CMAKE_CXX_COMPILER  "/home/tcspc/arm/aarch64-buildroot-linux-gnu_sdk-buildroot/bin/aarch64-buildroot-linux-gnu-g++")
# set(CMAKE_SYSROOT "/home/tcspc/arm/aarch64-buildroot-linux-gnu_sdk-buildroot/aarch64-buildroot-linux-gnu/sysroot")

# set(OpenCV_DIR /opt/hisi-linux/x86-arm/aarch64-himix100-linux/target/usr/opencv4/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)

# set(Open3D_DIR /opt/hisi-linux/x86-arm/aarch64-himix100-linux/target/usr/open3d/lib/cmake/open3d)
find_package(Open3D REQUIRED)



option(DEFAULT_LOG_LEVEL "default log level" INFO)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/task_module/header)

include_directories(${Open3D_INCLUDE_DIRS})

file(GLOB SRC_LIST ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/task_module/cpp/*.cpp
)

file(GLOB RK_LIB /usr/local/lib/libsdb-7316/*.so)
# file(GLOB OUTPUT_LIB ${CMAKE_CURRENT_SOURCE_DIR}/sd_lib/output/*.so)

set(LINK_LIB ${RK_LIB})

# link_directories(${CMAKE_CURRENT_SOURCE_DIR}/sd_lib/output)
link_directories(/usr/local/lib/libsdb-7316)

add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(${PROJECT_NAME}
        pthread dl m #标准库
        ${Open3D_LIBRARIES}
        ${LINK_LIB}
        ${OpenCV_LIBS}
        )

target_include_directories(${PROJECT_NAME} PUBLIC ${Open3D_INCLUDE_DIRS})