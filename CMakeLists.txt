cmake_minimum_required(VERSION 3.0.0)
project(KinectCap VERSION 0.1.0)

# custom cmake scripts
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(./cmake/configure_movecmd.cmake)
include(./cmake/configure_optix.cmake)

# include path and lib path
include_directories(./includes)
include_directories(./imgui)
include_directories(./calib)
include_directories(./src)
link_directories(./lib)

# find OPENGL
find_package(OpenGL REQUIRED)
if(OPENGL_FOUND)
    include_directories(${OpenGL_INCLUDE_DIRS})
    link_directories(${OpenGL_LIBRARY_DIRS})
    add_definitions(${OpenGL_DEFINITIONS})
endif()

# find glfw3
find_package(GLFW3 REQUIRED)

# find static glew
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/includes) # for glew configuration
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib) # for glew configuration
set(GLEW_USE_STATIC_LIBS True)
find_package(GLEW REQUIRED)
if(GLEW_FOUND)
    link_directories(${GLEW_LIBRARY_DIRS})
    include_directories(${GLEW_INCLUDE_DIRS})
endif()

# find opencv
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# find openmp
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    add_definitions(-D EnableOMP)
    message(STATUS "OpenMP enabled")
endif()

# collect gl libs
set(GL_LIBS ${GLFW3_LIBRARY} ${GLEW_STATIC_LIBRARIES} ${OPENGL_LIBRARIES})

# set up windows compilation options
if(WIN32)
    add_definitions(-D _USE_MATH_DEFINES)
    add_definitions(-D _CRT_NO_VA_START_VALIDATION)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17 /bigobj")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4061 /wd4244 /wd4246 /wd4305 /wd4267 /wd4711 /wd4710 /wd4514 /wd4477")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
else()
    add_definitions(-std=c++17)
    add_definitions(-g)
endif()


include_directories("E:/SimpleClothSimulator/src")
add_definitions(-D USE_OPTIX)
set(CMAKE_MODULE_PATH
"${PROJECT_SOURCE_DIR}/cmake"
#  "${CMAKE_CURRENT_SOURCE_DIR}/../cmake"
${CMAKE_MODULE_PATH}
)

find_package(CUDA REQUIRED)
find_package(OptiX REQUIRED)
include_directories("C:\\Program Files\\NVIDIA GPU Computing Toolkit\\CUDA\\v11.1\\include")
include_directories(${CUDA_TOOLKIT_INCLUDE})

if (CUDA_TOOLKIT_ROOT_DIR)
  include_directories(${CUDA_TOOLKIT_ROOT_DIR}/include)
endif()
include_directories(${OptiX_INCLUDE})

if (WIN32)
add_definitions(-DNOMINMAX)
endif()


# add sources
add_subdirectory(./imgui)
add_subdirectory(./src/utils)
add_subdirectory(./src/render)
add_subdirectory(./src/kinect)
add_subdirectory(./src/raycast)

set(libs utils_lib imgui_lib kinect_lib raycast_lib render_lib ${GL_LIBS} ${OpenCV_LIBS})

add_executable(main ./src/main.cpp)
target_link_libraries(main ${libs} k4a)

add_executable(2calib ./src/2calib.cpp)
target_link_libraries(2calib ${libs} k4a nlopt)


if(WIN32)
    if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
        add_custom_command(
            TARGET main POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E rename
                ${CMAKE_CURRENT_BINARY_DIR}/Debug/main.exe
                ${CMAKE_SOURCE_DIR}/main.exe
        )
    else()
        add_custom_command(
            TARGET main POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E rename
                ${CMAKE_CURRENT_BINARY_DIR}/Release/main.exe
                ${CMAKE_SOURCE_DIR}/main.exe
        )
    endif()
else()
    add_custom_command(
        TARGET main POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E rename
            ${CMAKE_CURRENT_BINARY_DIR}/main        
            ${CMAKE_SOURCE_DIR}/main    
    )
endif()
if(WIN32)
    if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
        add_custom_command(
            TARGET 2calib POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E rename
                ${CMAKE_CURRENT_BINARY_DIR}/Debug/2calib.exe
                ${CMAKE_SOURCE_DIR}/2calib.exe
        )
    else()
        add_custom_command(
            TARGET 2calib POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E rename
                ${CMAKE_CURRENT_BINARY_DIR}/Release/2calib.exe
                ${CMAKE_SOURCE_DIR}/2calib.exe
        )
    endif()
else()
    add_custom_command(
        TARGET 2calib POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E rename
            ${CMAKE_CURRENT_BINARY_DIR}/2calib        
            ${CMAKE_SOURCE_DIR}/2calib    
    )
endif()