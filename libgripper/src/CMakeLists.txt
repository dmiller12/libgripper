add_library(gripper
    barrett/barrett_hand.cpp
    barrett/barrett_hand_driver.cpp
    barrett/position_controller.cpp
    barrett/velocity_controller.cpp
    communication/serial_communicator.cpp
)

find_package(Boost 1.71.0 REQUIRED COMPONENTS system thread)

target_link_libraries(gripper
    PRIVATE
        Boost::system
        Boost::thread
)

target_include_directories(gripper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
