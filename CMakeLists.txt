cmake_minimum_required(VERSION 3.0)

project(px4ctrl_client)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -Wall -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

if(APPLE)
    message("Building for MacOS")
    find_package(OpenGL REQUIRED)
    find_library(COCOA_LIBRARY Cocoa REQUIRED)
    find_library(IOKIT_LIBRARY IOKit REQUIRED)
    find_library(COREVIDEO_LIBRARY CoreVideo REQUIRED)
    find_library(GLFW_LIBRARY glfw REQUIRED)
    include_directories(/usr/local/include)
elseif(UNIX)
    message("Building for Linux")
    find_package(OpenGL REQUIRED)
    find_package(glfw3 REQUIRED)
elseif(WIN32)
    message(FATAL_ERROR "Windows is not supported")
endif()
find_package(spdlog REQUIRED)

add_subdirectory(modules/yaml-cpp)
add_subdirectory(modules/libzmq)
option(CPPZMQ_BUILD_TESTS "" OFF)
add_subdirectory(modules/cppzmq)

set(IMGUI_DIR modules/imgui)

include_directories(
    include
    ${IMGUI_DIR} 
    ${IMGUI_DIR}/backends
)

add_library(imgui
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp 
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp 
    ${IMGUI_DIR}/imgui.cpp 
    ${IMGUI_DIR}/imgui_draw.cpp 
    ${IMGUI_DIR}/imgui_demo.cpp 
    ${IMGUI_DIR}/imgui_tables.cpp 
    ${IMGUI_DIR}/imgui_widgets.cpp
)

if(APPLE)
    target_link_libraries(imgui
    PUBLIC ${GLFW_LIBRARY}
    PUBLIC ${OPENGL_LIBRARIES}
    PUBLIC ${COCOA_LIBRARY}
    PUBLIC ${IOKIT_LIBRARY}
    PUBLIC ${COREVIDEO_LIBRARY})
  elseif(UNIX)
    set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")
    target_link_libraries(imgui 
        PUBLIC ${OPENGL_gl_LIBRARY} 
        PUBLIC glfw
    )
endif()

add_executable(px4client src/main.cpp src/client.cpp)

target_link_libraries(px4client
  PUBLIC imgui
  PUBLIC yaml-cpp
  PUBLIC cppzmq
  PUBLIC spdlog::spdlog $<$<BOOL:${MINGW}>:ws2_32>
)