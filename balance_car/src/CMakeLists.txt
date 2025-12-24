set(SOURCES
        main.cpp
        car.cpp
        motor.cpp
        pid.cpp
        can_receive.cpp
        LCD_driver.cpp
        LCD_gui.cpp
        adxl345.cpp
        i2c.cpp
        pac9557.cpp
        font/font8.cpp
        font/font12.cpp
        font/font16.cpp
        font/font20.cpp
        font/font24.cpp
)


add_executable(balance_car_app ${SOURCES})


target_include_directories(balance_car_app PRIVATE
        ${PROJECT_SOURCE_DIR}/inc
)


target_link_libraries(balance_car_app PRIVATE
        Threads::Threads
)
