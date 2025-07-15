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
