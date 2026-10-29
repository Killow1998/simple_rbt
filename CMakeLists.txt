cmake_minimum_required(VERSION 3.22)
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/packaging/VERSION" SIMPLE_RBT_RELEASE_VERSION LIMIT_COUNT 1)
project(simple_rbt_native VERSION ${SIMPLE_RBT_RELEASE_VERSION} LANGUAGES CXX)
include(CTest)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(SIMPLE_RBT_CUDA "Compile optional CUDA LiDAR acceleration" ON)
if(SIMPLE_RBT_CUDA)
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_ARCHITECTURES "75-real;80-real;86-real;89-real;90-real;100-real;120" CACHE STRING "Supported CUDA GPU architectures")
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
endif()
set(PROJECT_WORKSPACE "${CMAKE_CURRENT_SOURCE_DIR}")
set(SIMPLE_RBT_ROS "NONE" CACHE STRING "Core is ROS-free; adapters build independently")
set_property(CACHE SIMPLE_RBT_ROS PROPERTY STRINGS NONE)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/ros_backend.txt" CONTENT "${SIMPLE_RBT_ROS}\n")
if(NOT SIMPLE_RBT_ROS STREQUAL "NONE")
  message(FATAL_ERROR "Core has no ROS dependencies. Build adapters separately with cmake -S wrappers or the official Dockerfiles.")
endif()
file(GLOB MUJOCO_SEARCH "${PROJECT_WORKSPACE}/.venv/lib/python*/site-packages/mujoco")
if(NOT MUJOCO_SEARCH)
  message(FATAL_ERROR "MuJoCo SDK missing. Run ./setup.sh to prepare the locked SDK libraries.")
endif()
list(GET MUJOCO_SEARCH 0 MUJOCO_DIR)
file(GLOB MUJOCO_LIB "${MUJOCO_DIR}/libmujoco.so.*")
list(GET MUJOCO_LIB 0 MUJOCO_LIBRARY)
file(GLOB ORT_LIB "${PROJECT_WORKSPACE}/.venv/lib/python*/site-packages/onnxruntime/capi/libonnxruntime.so.*")
if(NOT ORT_LIB)
  message(FATAL_ERROR "ONNX Runtime shared library missing. Run ./setup.sh first.")
endif()
list(GET ORT_LIB 0 ONNXRUNTIME_LIBRARY)
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
file(CREATE_LINK "${ONNXRUNTIME_LIBRARY}" "${CMAKE_CURRENT_BINARY_DIR}/lib/libonnxruntime.so.1" SYMBOLIC)
set(CMAKE_BUILD_RPATH "${CMAKE_CURRENT_BINARY_DIR}/lib;${MUJOCO_DIR}")
add_library(mujoco SHARED IMPORTED)
set_target_properties(mujoco PROPERTIES IMPORTED_LOCATION "${MUJOCO_LIBRARY}" INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include")
add_library(onnxruntime SHARED IMPORTED)
set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}" INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/third_party/onnxruntime")
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)
find_path(TINYXML2_INCLUDE_DIR tinyxml2.h REQUIRED)
find_library(TINYXML2_LIBRARY tinyxml2 REQUIRED)
add_library(tinyxml2::tinyxml2 SHARED IMPORTED)
set_target_properties(tinyxml2::tinyxml2 PROPERTIES IMPORTED_LOCATION "${TINYXML2_LIBRARY}" INTERFACE_INCLUDE_DIRECTORIES "${TINYXML2_INCLUDE_DIR}")
find_package(glfw3 REQUIRED)
find_package(PNG REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EGL REQUIRED IMPORTED_TARGET egl)
pkg_check_modules(GL REQUIRED IMPORTED_TARGET gl)

add_subdirectory(diagnostics)

add_library(simple_rbt_robots src/robots.cpp)
target_include_directories(simple_rbt_robots PUBLIC include)
target_compile_options(simple_rbt_robots PRIVATE -Wall -Wextra -Wpedantic)

add_library(simple_rbt_random src/random.cpp)
target_include_directories(simple_rbt_random PUBLIC include)

add_library(simple_rbt_core
  src/core.cpp src/model_export.cpp src/scene.cpp src/user_scene.cpp
  src/scene_geometry.cpp src/scene_primitives.cpp src/scene_polygon.cpp src/scene_xml.cpp src/scene_connections.cpp src/scene_openings.cpp src/sensor_devices.cpp src/tracking.cpp src/controller.cpp
  src/controller_policy.cpp
  src/controllers/go2.cpp src/controllers/go2w.cpp
  src/controllers/arm.cpp
  src/controllers/flight.cpp
  src/flight_payload.cpp
  src/controllers/diffdrive.cpp src/controllers/scout_mini.cpp
  src/controllers/fr_mini.cpp src/controllers/ridgeback.cpp
  src/controllers/ranger_mini.cpp)
target_include_directories(simple_rbt_core PUBLIC include third_party)
target_compile_options(simple_rbt_core PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
target_link_libraries(simple_rbt_core PUBLIC mujoco simple_rbt_robots PRIVATE simple_rbt_random onnxruntime tinyxml2::tinyxml2 OpenSSL::Crypto)

add_library(simple_rbt_lidar src/lidar.cpp src/lidar_scene.cpp)
if(SIMPLE_RBT_CUDA)
  target_sources(simple_rbt_lidar PRIVATE src/lidar_cuda.cu)
  target_link_libraries(simple_rbt_lidar PRIVATE CUDA::cudart)
else()
  target_sources(simple_rbt_lidar PRIVATE src/lidar_cuda_stub.cpp)
endif()
target_include_directories(simple_rbt_lidar PUBLIC include)
target_compile_options(simple_rbt_lidar PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall;-Wextra;-Wpedantic;-ffp-contract=off> $<$<COMPILE_LANGUAGE:CUDA>:--fmad=false>)
target_link_libraries(simple_rbt_lidar PUBLIC mujoco PRIVATE Threads::Threads)

add_library(simple_rbt_meshoptimizer STATIC
  third_party/meshoptimizer/allocator.cpp third_party/meshoptimizer/simplifier.cpp)
add_library(simple_rbt_cameras src/cameras.cpp src/camera_context.cpp src/camera_render.cpp
  src/camera_mesh.cpp src/camera_mesh_data.cpp src/camera_lighting.cpp
  src/camera_planes.cpp)
target_include_directories(simple_rbt_cameras PUBLIC include PRIVATE third_party)
target_compile_options(simple_rbt_cameras PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
target_link_libraries(simple_rbt_cameras PUBLIC mujoco PRIVATE simple_rbt_meshoptimizer PkgConfig::EGL PkgConfig::GL Threads::Threads)

add_library(simple_rbt_course src/course.cpp src/course_geometry.cpp)
target_include_directories(simple_rbt_course PUBLIC include)
target_compile_options(simple_rbt_course PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
target_link_libraries(simple_rbt_course PUBLIC simple_rbt_core)

add_library(simple_rbt_runtime src/runtime.cpp src/runtime_imu.cpp src/runtime_lidar.cpp
  src/runtime_cameras.cpp src/runtime_camera_worker.cpp src/runtime_info.cpp
  src/artifacts.cpp src/resources.cpp src/viewer.cpp)
target_compile_definitions(simple_rbt_runtime PRIVATE
  SIMPLE_RBT_VERSION="${PROJECT_VERSION}"
  SIMPLE_RBT_CUDA_COMPILED=$<BOOL:${SIMPLE_RBT_CUDA}>)
target_include_directories(simple_rbt_runtime PUBLIC include)
target_compile_options(simple_rbt_runtime PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
target_link_libraries(simple_rbt_runtime PUBLIC simple_rbt_diagnostics simple_rbt_core simple_rbt_lidar simple_rbt_cameras PRIVATE glfw PkgConfig::GL PNG::PNG Threads::Threads)
add_subdirectory(ipc)
add_subdirectory(tutorials)
add_library(simple_rbt_session src/session.cpp src/session_control.cpp src/session_view.cpp)
target_link_libraries(simple_rbt_session PUBLIC simple_rbt_runtime simple_rbt_ipc)
target_compile_options(simple_rbt_session PRIVATE -Wall -Wextra -Wpedantic)
add_executable(simple-rbt-sim src/main.cpp)
target_compile_options(simple-rbt-sim PRIVATE -Wall -Wextra -Wpedantic -ffp-contract=off)
target_link_libraries(simple-rbt-sim PRIVATE simple_rbt_session)

option(SIMPLE_RBT_DESKTOP "Build native MyGo desktop console" ON)
if(SIMPLE_RBT_DESKTOP)
  file(READ "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mygo/manifest.json" MYGO_LOCK)
  string(JSON GO_VERSION GET "${MYGO_LOCK}" go version)
  find_program(SIMPLE_RBT_GO go HINTS
    "${CMAKE_CURRENT_SOURCE_DIR}/build/toolchains/${GO_VERSION}/go/bin" REQUIRED)
  file(GLOB_RECURSE CONSOLE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/console/*.go")
  file(GLOB MYGO_PATCHES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mygo/patches/*.patch")
  set(CONSOLE_BINARY "${CMAKE_CURRENT_BINARY_DIR}/simple-rbt-console")
  add_custom_command(OUTPUT "${CONSOLE_BINARY}"
    COMMAND "${CMAKE_COMMAND}" -E env CGO_ENABLED=0 GOTOOLCHAIN=local
      "GOMODCACHE=${CMAKE_CURRENT_SOURCE_DIR}/build/go-modules"
      "GOCACHE=${CMAKE_CURRENT_SOURCE_DIR}/build/go-cache"
      "${SIMPLE_RBT_GO}" build -trimpath "-ldflags=-s -w -X simple-rbt/console/session.Version=${PROJECT_VERSION}" -o "${CONSOLE_BINARY}" .
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/console"
    DEPENDS ${CONSOLE_SOURCES} console/go.mod console/go.sum packaging/VERSION
      third_party/mygo/manifest.json ${MYGO_PATCHES}
    COMMENT "Building native Go/MyGo console")
  add_custom_target(simple_rbt_console ALL DEPENDS "${CONSOLE_BINARY}")
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Install.cmake")
  include(cmake/Install.cmake)
endif()

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
