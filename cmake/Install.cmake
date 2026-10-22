# Installed runtime is independent of the preparation virtualenv and checkout.
include(GNUInstallDirs)
set(SIMPLE_RBT_DATA_DIR "${CMAKE_INSTALL_DATADIR}/simple-rbt")
# Use one stable, architecture-independent application library directory.
set(SIMPLE_RBT_LIB_DIR "lib/simple-rbt")
foreach(APP simple-rbt-sim)
  if(TARGET ${APP})
    set_target_properties(${APP} PROPERTIES
      INSTALL_RPATH "$ORIGIN/../lib/simple-rbt"
      INSTALL_RPATH_USE_LINK_PATH FALSE)
    install(TARGETS ${APP} RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
      COMPONENT Runtime)
  endif()
endforeach()
set(SIMPLE_RBT_CONSOLE_BINARY "${CMAKE_CURRENT_BINARY_DIR}/simple-rbt-console"
  CACHE FILEPATH "Externally built native desktop console executable")
if(SIMPLE_RBT_CONSOLE_BINARY)
  install(PROGRAMS "${SIMPLE_RBT_CONSOLE_BINARY}"
    DESTINATION "${CMAKE_INSTALL_BINDIR}" RENAME sprobo COMPONENT Runtime)
endif()

function(simple_rbt_install_library SOURCE SONAME)
  get_filename_component(REAL_LIBRARY "${SOURCE}" REALPATH)
  get_filename_component(FILENAME "${REAL_LIBRARY}" NAME)
  install(FILES "${REAL_LIBRARY}" DESTINATION "${SIMPLE_RBT_LIB_DIR}"
    COMPONENT Runtime)
  if(NOT FILENAME STREQUAL SONAME)
    install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink \"${FILENAME}\" \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${SIMPLE_RBT_LIB_DIR}/${SONAME}\" COMMAND_ERROR_IS_FATAL ANY)"
      COMPONENT Runtime)
  endif()
endfunction()
get_filename_component(MUJOCO_SONAME "${MUJOCO_LIBRARY}" NAME)
simple_rbt_install_library("${MUJOCO_LIBRARY}" "${MUJOCO_SONAME}")
simple_rbt_install_library("${ONNXRUNTIME_LIBRARY}" "libonnxruntime.so.1")
if(TARGET CUDA::cudart)
  get_target_property(SIMPLE_RBT_CUDART CUDA::cudart IMPORTED_LOCATION)
  simple_rbt_install_library("${SIMPLE_RBT_CUDART}" "libcudart.so.12")
endif()
get_filename_component(ORT_LIBRARY_DIR "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
if(EXISTS "${ORT_LIBRARY_DIR}/libonnxruntime_providers_shared.so")
  install(FILES "${ORT_LIBRARY_DIR}/libonnxruntime_providers_shared.so"
    DESTINATION "${SIMPLE_RBT_LIB_DIR}" COMPONENT Runtime)
endif()

install(DIRECTORY "${PROJECT_WORKSPACE}/assets/"
  DESTINATION "${SIMPLE_RBT_DATA_DIR}/assets" COMPONENT Runtime)
# Install exactly the mesh/texture files named by shipped model XML, rather
# than a developer's entire .vendor directory. Keep their original paths.
file(GLOB MODEL_XML_FILES "${PROJECT_WORKSPACE}/assets/models/*.xml")
set(VENDOR_RUNTIME_FILES)
foreach(MODEL IN LISTS MODEL_XML_FILES)
  file(READ "${MODEL}" XML)
  set(MESH_DIR "")
  string(REGEX MATCH "<compiler[^>]*meshdir=\"([^\"]+)\"" COMPILER "${XML}")
  if(COMPILER)
    set(MESH_DIR "${CMAKE_MATCH_1}")
  endif()
  string(REGEX MATCHALL "<(mesh|texture|hfield|skin|include)[^>]*file=\"[^\"]+\"" REFERENCES "${XML}")
  foreach(REFERENCE IN LISTS REFERENCES)
    string(REGEX REPLACE ".*file=\"([^\"]+)\".*" "\\1" ASSET "${REFERENCE}")
    if(NOT ASSET MATCHES "^@PROJECT_ROOT@/")
      set(ASSET "${MESH_DIR}/${ASSET}")
    endif()
    string(REPLACE "@PROJECT_ROOT@/" "" ASSET "${ASSET}")
    if(ASSET MATCHES "^\\.vendor/")
      list(APPEND VENDOR_RUNTIME_FILES "${ASSET}")
    endif()
  endforeach()
endforeach()
file(READ "${PROJECT_WORKSPACE}/assets/catalog.json" CATALOG)
string(REGEX MATCHALL "\"policy\"[ \r\n\t]*:[ \r\n\t]*\"\\.vendor/[^\"]+\"" POLICIES "${CATALOG}")
foreach(POLICY IN LISTS POLICIES)
  string(REGEX REPLACE ".*:[ \r\n\t]*\"([^\"]+)\"" "\\1" ASSET "${POLICY}")
  list(APPEND VENDOR_RUNTIME_FILES "${ASSET}")
endforeach()
list(REMOVE_DUPLICATES VENDOR_RUNTIME_FILES)
foreach(ASSET IN LISTS VENDOR_RUNTIME_FILES)
  if(NOT EXISTS "${PROJECT_WORKSPACE}/${ASSET}")
    message(FATAL_ERROR "Required runtime asset missing: ${ASSET}. Run setup.sh first.")
  endif()
  get_filename_component(ASSET_DIR "${ASSET}" DIRECTORY)
  install(FILES "${PROJECT_WORKSPACE}/${ASSET}"
    DESTINATION "${SIMPLE_RBT_DATA_DIR}/${ASSET_DIR}" COMPONENT Runtime)
endforeach()

install(FILES "${PROJECT_WORKSPACE}/packaging/sprobo.desktop"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/applications" COMPONENT Runtime)
install(FILES "${PROJECT_WORKSPACE}/packaging/sprobo.svg"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps" COMPONENT Runtime)
install(FILES "${PROJECT_WORKSPACE}/packaging/copyright"
  "${PROJECT_WORKSPACE}/packaging/README.md"
  DESTINATION "${CMAKE_INSTALL_DATADIR}/doc/sprobo" COMPONENT Runtime)
set(LICENSE_DIR "${CMAKE_INSTALL_DATADIR}/doc/sprobo/licenses")
install(FILES "${PROJECT_WORKSPACE}/third_party/mujoco_classic/LICENSE"
  "${PROJECT_WORKSPACE}/third_party/mujoco_classic/README.md"
  "${PROJECT_WORKSPACE}/third_party/mujoco_classic/manifest.json"
  DESTINATION "${LICENSE_DIR}/mujoco_classic" COMPONENT Runtime)
install(FILES "${PROJECT_WORKSPACE}/third_party/meshoptimizer/LICENSE.md"
  "${PROJECT_WORKSPACE}/third_party/meshoptimizer/README.md"
  "${PROJECT_WORKSPACE}/third_party/meshoptimizer/manifest.json"
  DESTINATION "${LICENSE_DIR}/meshoptimizer" COMPONENT Runtime)
file(GLOB MUJOCO_LICENSE_DIR "${PROJECT_WORKSPACE}/.venv/lib/python*/site-packages/mujoco-*.dist-info/licenses")
foreach(DIR IN LISTS MUJOCO_LICENSE_DIR)
  install(DIRECTORY "${DIR}/" DESTINATION "${LICENSE_DIR}/mujoco" COMPONENT Runtime)
endforeach()
install(FILES "${ORT_LIBRARY_DIR}/../LICENSE" "${ORT_LIBRARY_DIR}/../ThirdPartyNotices.txt"
  DESTINATION "${LICENSE_DIR}/onnxruntime" COMPONENT Runtime)
if(TARGET CUDA::cudart)
  get_filename_component(CUDA_LIBRARY_DIR "${SIMPLE_RBT_CUDART}" DIRECTORY)
  find_file(CUDA_RUNTIME_EULA NAMES EULA.txt copyright HINTS "${CUDAToolkit_ROOT}"
    "${CUDAToolkit_LIBRARY_ROOT}" "${CUDA_LIBRARY_DIR}/.."
    "${CUDA_LIBRARY_DIR}/../../.."
    "/usr/share/doc/cuda-cudart-${CUDAToolkit_VERSION_MAJOR}-${CUDAToolkit_VERSION_MINOR}"
    NO_DEFAULT_PATH REQUIRED)
  install(FILES "${CUDA_RUNTIME_EULA}"
    DESTINATION "${LICENSE_DIR}/cuda" RENAME EULA.txt COMPONENT Runtime)
endif()
foreach(VENDOR rl_sar-go2 rl_sar-go2w MuJoCo-LiDAR)
  install(FILES "${PROJECT_WORKSPACE}/.vendor/${VENDOR}/LICENSE"
    DESTINATION "${LICENSE_DIR}/${VENDOR}" COMPONENT Runtime)
endforeach()
if(EXISTS "${PROJECT_WORKSPACE}/console/licenses")
  install(DIRECTORY "${PROJECT_WORKSPACE}/console/licenses/"
    DESTINATION "${LICENSE_DIR}/console" COMPONENT Runtime)
endif()
install(FILES "${PROJECT_WORKSPACE}/third_party/nlohmann/LICENSE.MIT"
  DESTINATION "${LICENSE_DIR}/nlohmann" COMPONENT Runtime)

# Optional ROS image preparation is a closed build context shipped as data;
# the native host programs never compile or load these files at runtime.
set(ROS_CONTEXT "${SIMPLE_RBT_DATA_DIR}/ros-adapters")
foreach(ROS ros1 ros2)
  install(DIRECTORY "${PROJECT_WORKSPACE}/deployment/${ROS}/"
    DESTINATION "${ROS_CONTEXT}/deployment/${ROS}" COMPONENT Runtime
    FILES_MATCHING PATTERN "Dockerfile" PATTERN "*.xml")
  install(PROGRAMS "${PROJECT_WORKSPACE}/deployment/${ROS}/entrypoint.sh"
    DESTINATION "${ROS_CONTEXT}/deployment/${ROS}" COMPONENT Runtime)
endforeach()
install(DIRECTORY "${PROJECT_WORKSPACE}/wrappers/"
  DESTINATION "${ROS_CONTEXT}/wrappers" COMPONENT Runtime
  FILES_MATCHING PATTERN "*.cpp" PATTERN "*.hpp" PATTERN "*.inc" PATTERN "CMakeLists.txt")
install(DIRECTORY "${PROJECT_WORKSPACE}/ipc/"
  DESTINATION "${ROS_CONTEXT}/ipc" COMPONENT Runtime
  FILES_MATCHING PATTERN "*.cpp" PATTERN "*.hpp" PATTERN "CMakeLists.txt")
install(DIRECTORY "${PROJECT_WORKSPACE}/diagnostics/"
  DESTINATION "${ROS_CONTEXT}/diagnostics" COMPONENT Runtime
  FILES_MATCHING PATTERN "*.cpp" PATTERN "*.hpp" PATTERN "CMakeLists.txt")
foreach(HEADER ipc transport types robots event_log)
  install(FILES "${PROJECT_WORKSPACE}/include/simple_rbt/${HEADER}.hpp"
    DESTINATION "${ROS_CONTEXT}/include/simple_rbt" COMPONENT Runtime)
endforeach()
install(FILES "${PROJECT_WORKSPACE}/src/robots.cpp"
  DESTINATION "${ROS_CONTEXT}/src" COMPONENT Runtime)
install(DIRECTORY "${PROJECT_WORKSPACE}/third_party/nlohmann/"
  DESTINATION "${ROS_CONTEXT}/third_party/nlohmann" COMPONENT Runtime
  FILES_MATCHING PATTERN "json.hpp" PATTERN "LICENSE.MIT")
install(DIRECTORY "${PROJECT_WORKSPACE}/interfaces/livox_ros_driver2/"
  DESTINATION "${ROS_CONTEXT}/interfaces/livox_ros_driver2" COMPONENT Runtime
  FILES_MATCHING PATTERN "CMakeLists.txt" PATTERN "package.xml" PATTERN "*.msg"
  PATTERN "LICENSE.txt" PATTERN "README.md")
