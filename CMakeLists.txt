project(eecbs)

cmake_minimum_required (VERSION 2.6)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

include_directories("inc")
file(GLOB SOURCES "src/*.cpp")
add_executable(eecbs ${SOURCES})

include(CTest)
if(BUILD_TESTING)
    add_executable(clearance_model_test tests/ClearanceModelTest.cpp src/ClearanceModel.cpp)
    target_include_directories(clearance_model_test PRIVATE inc)
    add_test(NAME clearance_model_test COMMAND clearance_model_test)
endif()

# Find Boost
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )
target_link_libraries(eecbs ${Boost_LIBRARIES})
