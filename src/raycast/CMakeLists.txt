
set(
    SRC
    Raycast.cpp
)

add_library(raycast_lib STATIC ${SRC})
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
    target_link_libraries(raycast_lib geo_libd sim_libd cam_libd)
else()
    target_link_libraries(raycast_lib geo_lib sim_lib cam_lib)
endif()
target_link_libraries(raycast_lib ${CUDA_LIBRARIES}
${CUDA_CUDA_LIBRARY})