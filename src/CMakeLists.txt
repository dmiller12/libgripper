add_library(libgripper
    barrett/barrett_hand.cpp
    barrett/barrett_hand_driver.cpp
    communication/serial_communicator.cpp
)

find_package(Boost 1.71.0 REQUIRED COMPONENTS system thread)

target_link_libraries(libgripper
    PUBLIC
        Boost::system
        Boost::thread
)

target_include_directories(libgripper
    PUBLIC
        ../include
)

add_library(gripper::lib ALIAS libgripper)
