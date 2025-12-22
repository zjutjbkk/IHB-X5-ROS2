
# main
cmake_minimum_required(VERSION 3.16.3)

PROJECT(arm)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Debug")

# add_subdirectory(src)
# add_subdirectory(include)

find_package(Threads REQUIRED)

add_executable(${PROJECT_NAME} arm.cpp)
add_executable(pid pid.cpp)



# add_subdirectory(ihb_cpp)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../ihb_cpp BINARY_DIR)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../pid BINARY_DIR/pid)


# 添加ihb_cpp库链接
# target_link_libraries(${PROJECT_NAME} )



target_link_libraries(${PROJECT_NAME}
    ihb_cpp
    minipid
    # sockcanpp
    # ${IHB_CPP_LIB}
    # ${CMAKE_THREAD_LIBS_INIT}
)

target_link_libraries(pid
    ihb_cpp
    minipid
)

target_include_directories(${PROJECT_NAME} PRIVATE ${INC_DIRS})