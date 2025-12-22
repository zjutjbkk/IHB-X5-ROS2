# lib
cmake_minimum_required(VERSION 3.16.3)

PROJECT(minipid)
set(CMAKE_CXX_STANDARD 20)
# set(CMAKE_BUILD_TYPE "Debug")

# add_subdirectory(/path/to/libsockcanpprepo )
# add_subdirectory(lib/libsockcanpp)


add_library(${PROJECT_NAME} MiniPID.cpp MiniPID.h)

# set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION}) # 设置动态库版本号
target_link_libraries(${PROJECT_NAME}
    ${CMAKE_THREAD_LIBS_INIT}

)
target_include_directories(
    ${PROJECT_NAME}

    PUBLIC ./

)

# include(GNUInstallDirs)
# install(TARGETS ${PROJECT_NAME}  DESTINATION ${CMAKE_INSTALL_LIBDIR})
# install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})
# install(DIRECTORY lib/libsockcanpp/include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME})



# set(prefix      ${CMAKE_INSTALL_PREFIX})
# set(exec_prefix ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR})
# set(includedir  ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR})
# set(libdir      ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR})