cmake_minimum_required(VERSION 3.24)
project(Exo LANGUAGES CXX)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/bin")

if(MSVC)
  add_compile_options(/MP /arch:AVX2 /permissive-)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_supported OUTPUT error)
  if(ipo_supported)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
  endif()
endif()

find_package(OpenCV REQUIRED COMPONENTS core videoio highgui imgproc)
find_package(spdlog CONFIG REQUIRED)
find_package(tomlplusplus CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(apriltag CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)

find_path(OSCPACK_INCLUDE_DIR osc/OscOutboundPacketStream.h REQUIRED)
find_library(OSCPACK_LIBRARY oscpack REQUIRED)
if(NOT TARGET oscpack::oscpack)
  add_library(oscpack::oscpack UNKNOWN IMPORTED)
  set_target_properties(
    oscpack::oscpack
    PROPERTIES IMPORTED_LOCATION "${OSCPACK_LIBRARY}"
               INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${OSCPACK_INCLUDE_DIR}")
endif()

set(NVAR_ROOT "${CMAKE_SOURCE_DIR}/third_party/ARSDK")

add_library(NvARProxy STATIC "${NVAR_ROOT}/nvar/src/nvARProxy.cpp"
                             "${NVAR_ROOT}/nvar/src/nvCVImageProxy.cpp")

target_compile_features(NvARProxy PRIVATE cxx_std_20)

if(MSVC)
  target_compile_options(NvARProxy PRIVATE /W0)
endif()

target_include_directories(
  NvARProxy SYSTEM
  PUBLIC "${NVAR_ROOT}/nvar/include"
         "${NVAR_ROOT}/features/nvarbodyposeestimation/include"
         "${NVAR_ROOT}/features/nvarbodydetection/include"
         "${NVAR_ROOT}/features/nvarfaceboxdetection/include"
         "${NVAR_ROOT}/features/nvarfaceexpressions/include"
         "${NVAR_ROOT}/features/nvargazeredirection/include"
         "${NVAR_ROOT}/features/nvarlandmarkdetection/include")

add_executable(Exo)

file(GLOB_RECURSE EXO_SOURCES CONFIGURE_DEPENDS "src/*.cpp" "src/*.hpp")
target_sources(Exo PRIVATE ${EXO_SOURCES})
source_group(
  TREE "${CMAKE_CURRENT_SOURCE_DIR}/src"
  PREFIX "src"
  FILES ${EXO_SOURCES})

target_compile_features(Exo PRIVATE cxx_std_20)
target_include_directories(Exo PRIVATE "${CMAKE_SOURCE_DIR}/src")


get_target_property(EIGEN_INC Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(Exo SYSTEM PRIVATE ${EIGEN_INC})

if(MSVC)
  target_compile_options(Exo PRIVATE /W4)
endif()

target_precompile_headers(
  Exo
  PRIVATE
  <vector>
  <string>
  <Eigen/Core>
  <Eigen/Geometry>
  <memory>
  <thread>
  <mutex>
  <chrono>
  <opencv2/opencv.hpp>
  <spdlog/spdlog.h>
  <glm/glm.hpp>
  <toml++/toml.hpp>)

target_link_libraries(
  Exo
  PRIVATE NvARProxy
          opencv_core
          opencv_videoio
          opencv_highgui
          opencv_imgproc
          opencv_calib3d
          oscpack::oscpack
          spdlog::spdlog
          tomlplusplus::tomlplusplus
          glm::glm
          apriltag::apriltag
		  Eigen3::Eigen
          ws2_32
          winmm)

target_compile_definitions(Exo PRIVATE EXO_REL_SDK_PATH="arsdk")

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_custom_command(
    TARGET Exo
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:Exo>/arsdk"
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${NVAR_ROOT}"
            "$<TARGET_FILE_DIR:Exo>/arsdk"
    COMMENT "Debug: Mirroring NVIDIA ZIP structure into /arsdk..."
    VERBATIM)
endif()
