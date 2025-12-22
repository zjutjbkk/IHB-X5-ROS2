# lib
cmake_minimum_required(VERSION 3.16.3)

PROJECT(ihb_cpp)
set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_BUILD_TYPE "Debug")

# 查找必要的库
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_check_modules(GPIOD REQUIRED libgpiodcxx libgpiod)
# add_subdirectory(/path/to/libsockcanpprepo )
add_subdirectory(lib/libsockcanpp)

file(GLOB_RECURSE FILES ${CMAKE_CURRENT_SOURCE_DIR} src/*.cpp)

# header
file(GLOB_RECURSE HEADERS ${CMAKE_CURRENT_SOURCE_DIR} include/*.h)


add_library(${PROJECT_NAME} ${HEADERS} ${FILES})

# set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION}) # 设置动态库版本号
target_link_libraries(${PROJECT_NAME}
    ${GPIOD_LIBRARIES}
    sockcanpp
    Threads::Threads
    ${CMAKE_THREAD_LIBS_INIT}

)
target_include_directories(
    ${PROJECT_NAME}

    PUBLIC include/
    PUBLIC lib/libsockcanpp/include/
    PUBLIC ${GPIOD_INCLUDE_DIRS}
    
)

# include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME}  DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
install(DIRECTORY lib/libsockcanpp/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})

include(GNUInstallDirs)

# set(prefix      ${CMAKE_INSTALL_PREFIX})
# set(exec_prefix ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR})
# set(includedir  ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR})
# set(libdir      ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})