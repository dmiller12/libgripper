add_executable(test_driver
    test_driver.cpp
)

target_link_libraries(test_driver
    PRIVATE
        libgripper
)

add_executable(test_bhand
    test_bhand.cpp
)

target_link_libraries(test_bhand
    PRIVATE
        libgripper
)

add_executable(test_bhand_keyboard
    test_bhand_keyboard.cpp
)

target_link_libraries(test_bhand_keyboard
    PRIVATE
        libgripper
)

add_executable(static_friction_estimate
    static_friction_estimate.cpp
)

target_link_libraries(static_friction_estimate
    PRIVATE
        libgripper
)

add_executable(viscous_friction_estimate
    viscous_friction_estimate.cpp
)

target_link_libraries(viscous_friction_estimate
    PRIVATE
        libgripper
)
