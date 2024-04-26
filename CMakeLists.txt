cmake_minimum_required(VERSION 3.22.1)
project(SC_3D_CollisionDetectionHelpers CXX)

set(SC_3D_CollisionDetectionHelpers_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/sc_sacd.cpp")
set(SC_3D_CollisionDetectionHelpers_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/src/sc_sacd.h")

add_library(SC_3D_CollisionDetectionHelpers ${SC_3D_CollisionDetectionHelpers_SOURCES})

set_target_properties(SC_3D_CollisionDetectionHelpers PROPERTIES VERSION 1.0.0 SOVERSION 1)

if(NOT DEFINED CMAKE_BUILD_TYPE OR "${CMAKE_BUILD_TYPE}" STREQUAL "")
    message("Defaulting to \"Debug\" build type.")
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type" FORCE)
else()
    message("Using build type \"${CMAKE_BUILD_TYPE}\".")
endif()

target_compile_options(SC_3D_CollisionDetectionHelpers PUBLIC
$<IF:$<CONFIG:Debug>,-Og,-fno-delete-null-pointer-checks -fno-strict-overflow -fno-strict-aliasing -ftrivial-auto-var-init=zero>
-Wall -Wformat -Wformat=2 -Wconversion -Wimplicit-fallthrough
-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=3
-D_GLIBCXX_ASSERTIONS
-fstrict-flex-arrays=3
-fstack-clash-protection -fstack-protector-strong
-fPIC
)

target_link_options(SC_3D_CollisionDetectionHelpers PUBLIC
$<IF:$<CONFIG:Debug>,-Og,-fno-delete-null-pointer-checks -fno-strict-overflow -fno-strict-aliasing -ftrivial-auto-var-init=zero>
-Wall -Wformat -Wformat=2 -Wconversion -Wimplicit-fallthrough
-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=3
-D_GLIBCXX_ASSERTIONS
-fstrict-flex-arrays=3
-fstack-clash-protection -fstack-protector-strong
-Wl,-z,noexecstack
-Wl,-z,relro -Wl,-z,now
-fPIC
)

add_executable(UnitTest src/test.cpp)
target_link_libraries(UnitTest SC_3D_CollisionDetectionHelpers)
target_include_directories(UnitTest PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_compile_features(UnitTest PUBLIC cxx_std_20)

target_compile_options(UnitTest PUBLIC
$<IF:$<CONFIG:Debug>,-Og,-fno-delete-null-pointer-checks -fno-strict-overflow -fno-strict-aliasing -ftrivial-auto-var-init=zero>
-Wall -Wformat -Wformat=2 -Wconversion -Wimplicit-fallthrough
-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=3
-D_GLIBCXX_ASSERTIONS
-fstrict-flex-arrays=3
-fstack-clash-protection -fstack-protector-strong
-fPIE
)

target_link_options(UnitTest PUBLIC
$<IF:$<CONFIG:Debug>,-Og,-fno-delete-null-pointer-checks -fno-strict-overflow -fno-strict-aliasing -ftrivial-auto-var-init=zero>
-Wall -Wformat -Wformat=2 -Wconversion -Wimplicit-fallthrough
-U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=3
-D_GLIBCXX_ASSERTIONS
-fstrict-flex-arrays=3
-fstack-clash-protection -fstack-protector-strong
-Wl,-z,nodlopen -Wl,-z,noexecstack
-Wl,-z,relro -Wl,-z,now
-fPIE
-pie
)

install(TARGETS SC_3D_CollisionDetectionHelpers DESTINATION "lib")
install(FILES ${SC_3D_CollisionDetectionHelpers_HEADERS} DESTINATION "include")
