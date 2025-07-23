cmake_minimum_required(VERSION 3.10)
project(gripper VERSION 1.0)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) 

add_subdirectory(src)
add_subdirectory(examples)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)


write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/gripper-config-version.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  "cmake/gripper-config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/gripper-config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gripper
)

install(TARGETS gripper
  EXPORT gripper-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/gripper/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gripper
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/gripper-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/gripper-config-version.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gripper
)

install(EXPORT gripper-targets
  FILE gripper-targets.cmake
  NAMESPACE gripper::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/gripper
)
