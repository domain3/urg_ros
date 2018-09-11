cmake_minimum_required(VERSION 2.8.3)


###########
## Build ##
###########

add_executable(ground
    src/ground.cpp
    src/ground.hpp
    src/dead.cpp
    src/dead.hpp
    src/ransac_cuda.hpp
    src/rviz.cpp
    src/rviz.hpp
    src/particleFilter.cpp
    src/particleFilter.hpp
)

cuda_add_library(CUDA_DETECT ${LIB_TYPE} 
    src/ransac_cuda.cu

)

target_link_libraries(ground
   ${catkin_LIBRARIES}
   CUDA_DETECT
)

