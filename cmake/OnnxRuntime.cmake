set(ONNXRUNTIME_ROOT "" CACHE PATH "Optional ONNX Runtime root")

if(NOT ONNXRUNTIME_ROOT AND DEFINED ENV{ONNXRUNTIME_ROOT})
  file(TO_CMAKE_PATH "$ENV{ONNXRUNTIME_ROOT}" ONNXRUNTIME_ROOT)
endif()

set(_hints)
if(ONNXRUNTIME_ROOT)
  list(APPEND _hints "${ONNXRUNTIME_ROOT}")
endif()
if(APPLE)
  list(APPEND _hints /opt/homebrew/opt/onnxruntime /usr/local/opt/onnxruntime /opt/homebrew /usr/local)
elseif(UNIX)
  list(APPEND _hints /usr/local /opt/onnxruntime)
endif()

find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(PC_ONNXRUNTIME QUIET libonnxruntime)
endif()

find_path(ONNXRUNTIME_INCLUDE_DIR
  onnxruntime_cxx_api.h
  HINTS ${_hints} ${PC_ONNXRUNTIME_INCLUDE_DIRS}
  PATH_SUFFIXES include include/onnxruntime
)
find_library(ONNXRUNTIME_LIBRARY
  onnxruntime
  HINTS ${_hints} ${PC_ONNXRUNTIME_LIBRARY_DIRS}
  PATH_SUFFIXES lib lib64
)

if(NOT ONNXRUNTIME_INCLUDE_DIR OR NOT ONNXRUNTIME_LIBRARY)
  message(FATAL_ERROR
    "ONNX Runtime not found.\n"
    "Install it first, for example:\n"
    "  brew install onnxruntime\n"
    "or place it where pkg-config/CMake can find it."
  )
endif()

add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
set_target_properties(onnxruntime::onnxruntime PROPERTIES
  IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIR}"
)

get_filename_component(_ort_lib_dir "${ONNXRUNTIME_LIBRARY}" DIRECTORY)
if(APPLE)
  set_property(TARGET onnxruntime::onnxruntime APPEND PROPERTY INTERFACE_LINK_OPTIONS "LINKER:-rpath,${_ort_lib_dir}")
endif()
