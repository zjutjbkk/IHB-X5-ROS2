# main
cmake_minimum_required(VERSION 3.16.3)

PROJECT(IHB_DEMO)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Debug")

# add_subdirectory(src)
# add_subdirectory(include)

find_package(Threads REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)
add_executable(reset reset.cpp)
add_executable(forceTest forceTest.cpp)
add_executable(hallTest hallTest.cpp)
add_executable(serangle serAngle.cpp)
add_executable(canAngleThread canAngleTestThread.cpp)
add_executable(step_motor step_motor.cpp)
add_executable(statics_model_test statics_model_test.cpp)


# add_subdirectory(ihb_cpp)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../ihb_cpp BINARY_DIR)

# 查找ihb_cpp库
# find_library(IHB_CPP_LIB ihb_cpp)
# 输出查找结果
message(STATUS "ihb_cpp lib: ${IHB_CPP_LIB}")

# 添加ihb_cpp库链接
# target_link_libraries(${PROJECT_NAME} )



target_link_libraries(${PROJECT_NAME}
    ihb_cpp
    # sockcanpp
    # ${IHB_CPP_LIB}
    # ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries( reset
    ihb_cpp
    # sockcanpp
    # ${IHB_CPP_LIB}
    # ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries( forceTest
    ihb_cpp
    # sockcanpp
    # ${IHB_CPP_LIB}
    # ${CMAKE_THREAD_LIBS_INIT}
)

target_link_libraries( hallTest
    ihb_cpp
    # sockcanpp
    # ${IHB_CPP_LIB}
    # ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries( serangle
    ihb_cpp
    # sockcanpp
    # ${IHB_CPP_LIB}
    # ${CMAKE_THREAD_LIBS_INIT}
)
target_link_libraries( canAngleThread
    ihb_cpp
)
target_link_libraries(step_motor ihb_cpp)
target_link_libraries(statics_model_test ihb_cpp)

target_include_directories(${PROJECT_NAME} PRIVATE ${INC_DIRS})