cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

project(ISEPURE)

set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_BUILD_TYPE "RELEASE")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(${PROJECT_SOURCE_DIR}/thirdparty/GMM/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/ThreadPool/include)

add_library( isepure SHARED
   ./src/ISEPrepocessor.cpp
   ./src/ISEUtils.cpp
   ./src/ISETree.cpp
   ./src/ISEEncoder.cpp
)

add_library( threadpool SHARED
   ./thirdparty/ThreadPool/src/ThreadPool.cpp
)

find_library( gmm
   libgmm.so ${PROJECT_SOURCE_DIR}/lib/
)

add_executable(Test ./test/test.cpp)
target_link_libraries(Test isepure ${gmm} threadpool)

