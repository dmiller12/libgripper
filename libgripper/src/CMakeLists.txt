include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        ee4f6ac9c6391523876836fcf6f276411b7f3670
)
FetchContent_MakeAvailable(moteus)
add_compile_definitions(CANFD_FDF)
add_compile_definitions(MJBOTS_MOTEUS_ENABLE_SOCKETCAN)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG "0.8.0"
)
FetchContent_MakeAvailable(yaml-cpp)

add_library(gripper SHARED
    barrett/barrett_hand.cpp
    barrett/barrett_hand_driver.cpp
    barrett/position_controller.cpp
    barrett/velocity_controller.cpp
    magnum_opus/magnum_gripper.cpp
    magnum_opus/magnum_gripper_driver.cpp
    utils.cpp
    communication/serial_communicator.cpp
)

find_package(Boost 1.71.0 REQUIRED COMPONENTS system thread filesystem)

target_link_libraries(gripper
    PRIVATE
        moteus::cpp
        yaml-cpp
        Boost::system
        Boost::thread
        Boost::filesystem
)

target_include_directories(gripper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:$<TARGET_PROPERTY:moteus::cpp,INTERFACE_INCLUDE_DIRECTORIES>>
        $<BUILD_INTERFACE:$<TARGET_PROPERTY:yaml-cpp,INTERFACE_INCLUDE_DIRECTORIES>>
)